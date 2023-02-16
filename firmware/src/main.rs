#![no_std]
#![no_main]
#![allow(unreachable_code, unused)] // TODO: remove!

mod gnss;
mod radio_hal;
mod spi;

use core::cell::RefCell;
use cortex_m::delay;
use cortex_m::peripheral::itm;
use cortex_m::{iprint, iprintln};
use cortex_m_rt::entry;
use cortex_m_semihosting::{hprint, hprintln};
use panic_itm as _;
use radio::Transmit;
use radio_hal::RadioHal;
use radio_sx128x::base::Hal;
use spi::Spi;
use stm32g4::stm32g4a1 as pac;
use stm32g4::stm32g4a1::syscfg::swpr::PAGE0_WP_R;
use stm32g4::stm32g4a1::{gpioa, gpiob};

const HSI16_CLOCK_FREQUENCY: u32 = 16_000_000;
const AHB_CLOCK_FREQUENCY: u32 = HSI16_CLOCK_FREQUENCY;
const LED_COUNTER_PERIOD: u32 = 16_000;

#[macro_export]
macro_rules! pin_mode_input {
    ($gpio:expr, $num:literal, $pull:expr) => {
        paste::paste! {
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().input());
        }
    };
}

#[macro_export]
macro_rules! pin_mode_output {
    ($gpio:expr, $num:literal, $type:expr, $speed:expr, $state:expr) => {
        paste::paste! {
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.odr.modify(|_, w| w.[<odr $num>]().bit($state));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().output());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().floating());
        }
    };
}

#[macro_export]
macro_rules! pin_mode_alternate {
    ($gpio:expr, $num:literal, $type:expr, $pull:expr, $speed:expr, $alt:expr) => {
        paste::paste! {
            $gpio.afrl.modify(|_, w| w.[<afrl $num>]().variant($alt));
            // $gpio.afrh.modify(|_, w| w.[<afrh $num>]().variant($alt));
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().alternate());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
        }
    };
}

fn init_led(dp: &pac::Peripherals) {
    use gpiob::afrl::AFRL0_A::{Af10, Af2};
    use gpiob::ospeedr::OSPEEDR0_A::LowSpeed;
    use gpiob::otyper::OT0_A::OpenDrain;
    use gpiob::pupdr::PUPDR0_A::Floating;

    // enable IO port B clock
    dp.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

    // enable TIM3 timer clock
    dp.RCC.apb1enr1.modify(|_, w| w.tim3en().enabled());

    pin_mode_alternate!(dp.GPIOB, 4, OpenDrain, Floating, LowSpeed, Af2);
    pin_mode_alternate!(dp.GPIOB, 5, OpenDrain, Floating, LowSpeed, Af2);
    pin_mode_alternate!(dp.GPIOB, 7, OpenDrain, Floating, LowSpeed, Af10);

    const FREQUENCY: u32 = 1_000;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * LED_COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16; // TODO: reverse calculation, since the rounding error can be big

    // set prescaler period
    dp.TIM3.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    dp.TIM3.arr.modify(|_, w| w.arr().variant(LED_COUNTER_PERIOD - 1));

    // set duty cycle
    dp.TIM3.ccr1().modify(|_, w| w.ccr().variant(0));
    dp.TIM3.ccr2().modify(|_, w| w.ccr().variant(0));
    dp.TIM3.ccr4().modify(|_, w| w.ccr().variant(0));

    // set PWM mode 1
    dp.TIM3.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    dp.TIM3.ccmr1_output().modify(|_, w| w.oc2m().bits(0b110));
    dp.TIM3.ccmr2_output().modify(|_, w| w.oc4m().bits(0b110));

    // select active low polarity
    dp.TIM3.ccer.modify(|_, w| w.cc1p().set_bit());
    dp.TIM3.ccer.modify(|_, w| w.cc2p().set_bit());
    dp.TIM3.ccer.modify(|_, w| w.cc4p().set_bit());

    // enable output
    dp.TIM3.ccer.modify(|_, w| w.cc1e().set_bit());
    dp.TIM3.ccer.modify(|_, w| w.cc2e().set_bit());
    dp.TIM3.ccer.modify(|_, w| w.cc4e().set_bit());

    // enable counter
    dp.TIM3.cr1.modify(|_, w| w.cen().set_bit());
}

fn led(p: &pac::Peripherals, red: u32, green: u32, blue: u32) {
    // set duty cycle
    p.TIM3.ccr1().modify(|_, w| w.ccr().variant(red));
    p.TIM3.ccr2().modify(|_, w| w.ccr().variant(green));
    p.TIM3.ccr4().modify(|_, w| w.ccr().variant(blue));
}

// main motor: PB6 - AF1: TIM16_CH1N, AF2: TIM4_CH1, AF5: TIM8_CH1
// tail motor: PA1 - AF1: TIM2_CH2, AF9: TIM15_CH1N

fn servos(p: &pac::Peripherals, left_percent: u8, right_percent: u8, back_percent: u8) {
    // enable IO port A clock
    p.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());

    // enable timer clocks
    p.RCC.apb1enr1.modify(|_, w| w.tim2en().enabled());
    p.RCC.apb1enr1.modify(|_, w| w.tim4en().enabled());

    // TODO: wait for the clock to become active?

    // disable pull-up and pull-down resistors
    p.GPIOA.pupdr.modify(|_, w| w.pupdr2().floating());
    p.GPIOA.pupdr.modify(|_, w| w.pupdr11().floating());
    p.GPIOA.pupdr.modify(|_, w| w.pupdr12().floating());

    // set alternate function
    p.GPIOA.afrl.modify(|_, w| w.afrl2().af1()); // AF1:  TIM2_CH3, AF9: TIM15_CH1
    p.GPIOA.afrh.modify(|_, w| w.afrh11().af10()); // AF10: TIM4_CH1, AF11: TIM1_CH4, AF6: TIM1_CH1N
    p.GPIOA.afrh.modify(|_, w| w.afrh12().af10()); // AF10: TIM4_CH2, AF6: TIM1_CH2N, AF1: TIM16_CH1

    // set alternate mode
    p.GPIOA.moder.modify(|_, w| w.moder2().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder11().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder12().alternate());

    // // test
    // p.GPIOA.moder.modify(|_, w| w.moder2().output());
    // p.GPIOA.moder.modify(|_, w| w.moder11().output());
    // p.GPIOA.moder.modify(|_, w| w.moder12().output());
    // p.GPIOA.odr.modify(|_, w| w.odr2().high());
    // p.GPIOA.odr.modify(|_, w| w.odr11().high());
    // p.GPIOA.odr.modify(|_, w| w.odr12().high());

    const COUNTER_PERIOD: u32 = 100;
    const FREQUENCY: u32 = 450;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16;

    // set prescaler period
    p.TIM2.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    p.TIM4.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    p.TIM2.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    p.TIM4.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));

    const MIN_DUTY_PERCENT: u32 = 50; // measured: 49%
    const MAX_DUTY_PERCENT: u32 = 90; // measured: 87%

    let left_duty_percent =
        u32::from(left_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let right_duty_percent =
        u32::from(right_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let back_duty_percent =
        u32::from(back_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;

    // set duty cycle
    p.TIM2.ccr3().modify(|_, w| w.ccr().variant(left_duty_percent * COUNTER_PERIOD / 100));
    p.TIM4.ccr1().modify(|_, w| w.ccr().variant(right_duty_percent * COUNTER_PERIOD / 100));
    p.TIM4.ccr2().modify(|_, w| w.ccr().variant(back_duty_percent * COUNTER_PERIOD / 100));

    // set PWM mode 1
    p.TIM2.ccmr2_output().modify(|_, w| w.oc3m().bits(0b110));
    p.TIM4.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    p.TIM4.ccmr1_output().modify(|_, w| w.oc2m().bits(0b110));

    // select active high polarity
    p.TIM2.ccer.modify(|_, w| w.cc3p().clear_bit());
    p.TIM4.ccer.modify(|_, w| w.cc1p().clear_bit());
    p.TIM4.ccer.modify(|_, w| w.cc2p().clear_bit());

    // enable output
    p.TIM2.ccer.modify(|_, w| w.cc3e().set_bit());
    p.TIM4.ccer.modify(|_, w| w.cc1e().set_bit());
    p.TIM4.ccer.modify(|_, w| w.cc2e().set_bit());

    // enable counter
    p.TIM2.cr1.modify(|_, w| w.cen().set_bit());
    p.TIM4.cr1.modify(|_, w| w.cen().set_bit());
}

fn accel_read_multiple(spi: &mut Spi, delay: &mut delay::Delay, address: u8, data: &mut [u8]) {
    spi.activate_peripheral(spi::Peripheral::Imu, delay);

    // 5 ns needed before clock goes low - TODO: make this nicer
    cortex_m::asm::nop();

    spi.write(1 << 7 | address);
    spi.read();

    for x in data {
        spi.write(0);
        *x = spi.read();
    }

    // 20 ns needed before clock goes high - TODO: make this nicer
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    spi.deactivate_peripheral();
}

fn accel_write_multiple(spi: &mut Spi, delay: &mut delay::Delay, address: u8, data: &[u8]) {
    spi.activate_peripheral(spi::Peripheral::Imu, delay);

    spi.write(address);
    spi.read();

    for x in data {
        spi.write(*x);
        spi.read();
    }

    spi.deactivate_peripheral();
}

fn accel_read(spi: &mut Spi, delay: &mut delay::Delay, address: u8) -> u8 {
    let mut data = [0];
    accel_read_multiple(spi, delay, address, &mut data);
    data[0]
}

fn accel_write(spi: &mut Spi, delay: &mut delay::Delay, address: u8, data: u8) {
    let data = [data];
    accel_write_multiple(spi, delay, address, &data);
}

struct AccelerometerData {
    acceleration: (i16, i16, i16),
    gyroscope: (i16, i16, i16),
}

fn read_accelerometer_data(spi: &mut Spi, delay: &mut delay::Delay) -> AccelerometerData {
    let mut data = [0; 12];

    accel_read_multiple(spi, delay, 0x22, &mut data);

    let gx = (u16::from(data[1]) << 8 | u16::from(data[0])) as i16;
    let gy = (u16::from(data[3]) << 8 | u16::from(data[2])) as i16;
    let gz = (u16::from(data[5]) << 8 | u16::from(data[4])) as i16;
    let ax = (u16::from(data[7]) << 8 | u16::from(data[6])) as i16;
    let ay = (u16::from(data[9]) << 8 | u16::from(data[8])) as i16;
    let az = (u16::from(data[11]) << 8 | u16::from(data[10])) as i16;

    AccelerometerData {
        acceleration: (ax, ay, az),
        gyroscope: (gx, gy, gz),
    }
}

fn spi_radio_transmit(spi: &mut Spi, delay: &mut delay::Delay, data: &mut [u8]) {
    spi.activate_peripheral(spi::Peripheral::Radio, delay);

    for x in data {
        spi.write(*x);
        // hprintln!("out {:02x}", *x);
        *x = spi.read();
        // hprintln!("in {:02x}", *x);
    }

    spi.deactivate_peripheral();
}

fn print_response1(stim: &mut itm::Stim, response: u8) {
    let circuit = response >> 5;
    let status = (response >> 2) & 0x7;
    iprintln!(stim, " - {:x} {:x}", circuit, status);
}

fn print_response(stim: &mut itm::Stim, label: &str, data: &[u8]) {
    iprintln!(stim, "{}:", label);
    for x in data {
        print_response1(stim, *x);
    }
}

// fn bluetooth() {

//     // SetStandby(STDBY_RC)
//     let mut data = [0x80, 0];
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetStandby", &data);

//     // SetPacketType(PACKET_TYPE_BLE)
//     let mut data = [0x8a, 0x04];
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetPacketType", &data);

//     // SetRfFrequency(rfFrequency)
//     let mut data = [0x86, 0xB8, 0x9D, 0x89]; // TODO: this frequency is probably incorrect
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetRfFrequency", &data);

//     // SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
//     let mut data = [0x8F, 0x80, 0x00];
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetBufferBaseAddress", &data);

//     // SetModulationParams(BLE_BR_1_000_BW_1_2, MOD_IND_0_5, BT_0_5)
//     let mut data = [0x8B, 0x45, 0x01, 0x20];
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetModulationParams", &data);

//     // SetPacketParams(packetParam[0], packetParam[1], packetParam[2], packetParam[3])
//     // Although this command can accept up to 7 arguments, in BLE mode SetPacketParams can accept only 4. However the 3 remaining arguments must be set to 0 and sent to the radio.
//     let mut data = [0x8C, ...];
//     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
//     print_response(stim, "SetPacketParams", &data);

//     // ....
// }

fn init_radio<'a>(
    spi: &mut Spi,
    delay: &'a RefCell<delay::Delay>,
    stim: &'a RefCell<&'a mut itm::Stim>,
) {
    // loop {
    //     let mut data = [0x80, 0x00];
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    //     if data[0] & 0b11111100 == 0b01000100 {
    //         led(&dp, 0, LED_COUNTER_PERIOD / 2, 0);
    //     } else {
    //         led(&dp, LED_COUNTER_PERIOD / 2, 0, 0);
    //     }
    //     delay.borrow_mut().delay_ms(10);
    // }

    // // wait for radio to become available
    // loop {
    //     let mut data = [0xc0, 0]; // GetStatus
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
    //     print_response(stim, "GetStatus", &data);
    //     if data[1] & 0b00011100 ==  {
    //         iprintln!(&mut stim.borrow_mut(), "radio firmware: 0x{:04x}", firmware);
    //         break;
    //     }

    //     delay.delay_ms(10); // not necessary
    // }

    // wait for radio to become available
    iprintln!(&mut stim.borrow_mut(), "waiting for radio...");
    let mut success_count = 0;
    loop {
        let mut data = [0x80, 0x00];
        spi_radio_transmit(spi, &mut delay.borrow_mut(), &mut data);
        if data[0] & 0b11111100 == 0b01000100 {
            // let address: u16 = 0x0153;
            // let mut data = [0x19, (address >> 8) as u8, address as u8, 0, 0, 0];
            // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
            // print_response1(&mut stim.borrow_mut(), data[0]);
            // let firmware = u16::from(data[4]) << 8 | u16::from(data[5]);
            // if firmware & 0xff00 == 0xa900 && (data[0] >> 2) & 0x7 != 0x7 {
            success_count += 1;
            if success_count == 10 {
                // iprintln!(&mut stim.borrow_mut(), "SX128x firmware: {:04x}", firmware);
                break;
            }
        } else {
            success_count = 0;
        }
        delay.borrow_mut().delay_ms(10);
    }
}

fn radio<'a>(
    spi: &'a RefCell<Spi<'a>>,
    delay: &'a RefCell<delay::Delay>,
    stim: &'a RefCell<&'a mut itm::Stim>,
) {
    // loop {
    //     let mut data = [0x80, 0x00];
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    //     if data[0] & 0b11111100 == 0b01000100 {
    //         led(&dp, 0, LED_COUNTER_PERIOD / 2, 0);
    //     } else {
    //         led(&dp, LED_COUNTER_PERIOD / 2, 0, 0);
    //     }
    //     delay.borrow_mut().delay_ms(10);
    // }

    // // wait for radio to become available
    // loop {
    //     let mut data = [0xc0, 0]; // GetStatus
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
    //     print_response(stim, "GetStatus", &data);
    //     if data[1] & 0b00011100 ==  {
    //         iprintln!(&mut stim.borrow_mut(), "radio firmware: 0x{:04x}", firmware);
    //         break;
    //     }

    //     delay.delay_ms(10); // not necessary
    // }

    // wait for radio to become available
    iprintln!(&mut stim.borrow_mut(), "waiting for radio...");
    let mut success_count = 0;
    loop {
        let mut data = [0x80, 0x00];
        spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
        if data[0] & 0b11111100 == 0b01000100 {
            // let address: u16 = 0x0153;
            // let mut data = [0x19, (address >> 8) as u8, address as u8, 0, 0, 0];
            // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
            // print_response1(&mut stim.borrow_mut(), data[0]);
            // let firmware = u16::from(data[4]) << 8 | u16::from(data[5]);
            // if firmware & 0xff00 == 0xa900 && (data[0] >> 2) & 0x7 != 0x7 {
            success_count += 1;
            if success_count == 10 {
                // iprintln!(&mut stim.borrow_mut(), "SX128x firmware: {:04x}", firmware);
                break;
            }
        } else {
            success_count = 0;
        }
        delay.borrow_mut().delay_ms(10);
    }

    // led(&dp, 0, LED_COUNTER_PERIOD / 2, 0);
    // delay.borrow_mut().delay_ms(100);

    // // test radio reading and writing
    // loop {
    //     let address: u16 = 0x9ce;
    //     let mut data = [0x18, (address >> 8) as u8, address as u8, 1, 2, 3, 4, 5];
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
    //     // iprintln!(&mut stim.borrow_mut(), "status: {:?}", &data);

    //     let address: u16 = 0x9ce;
    //     let mut data = [0x19, (address >> 8) as u8, address as u8, 0, 0, 0, 0, 0, 0];
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay, &mut data);
    //     let data = &data[4..];
    //     // iprintln!(&mut stim.borrow_mut(), "data: {:?}", data);
    //     if data == [1, 2, 3, 4, 5] {
    //         break;
    //     }

    //     delay.delay_ms(10); // not necessary
    // }

    let radio_config = radio_sx128x::Config::gfsk();
    let mut radio = radio_sx128x::Sx128x::new(
        RadioHal {
            spi,
            delay,
            stim: &stim,
        },
        &radio_config,
    )
    .unwrap();
    // radio.calibrate(radio_sx128x::device::CalibrationParams::all()).unwrap();
    // delay.borrow_mut().delay_ms(100);
    let data = [1, 2, 3];
    radio.start_transmit(&data).unwrap();
    loop {}

    // self.set_regulator_mode(config.regulator_mode)?;
    // self.config.regulator_mode = config.regulator_mode;

    // // Update modem and channel configuration
    // self.set_channel(&config.channel)?;
    // self.config.channel = config.channel.clone();

    // self.configure_modem(&config.modem)?;
    // self.config.modem = config.modem.clone();

    // // Update power amplifier configuration
    // self.set_power_ramp(config.pa_config.power, config.pa_config.ramp_time)?;
    // self.config.pa_config = config.pa_config.clone();

    // TODO: ??????
    // To perform the calibration the Calibrate( calibParam ) function, opcode 0x89, must be used with the calibration parameters configured as follows:
    // calibParam.ADCBulkPEnable = 1; calibParam.ADCBulkNEnable = 1; calibParam.ADCPulseEnable = 1; calibParam.PLLEnable = 1; calibParam.RC13MEnable = 1; calibParam.RC64KEnable = 1;
    // Then we call the calibration function:
    // Radio.Calibrate( calibParam );

    // loop {
    // SetStandby(STDBY_RC)
    let mut data = [0x80, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetStandby", &data);

    // TODO: do we need to calibrate the internal RC oscillator?
    // Calibrate()
    // let mut data = [0x89, 0b00111111];
    let mut data = [0x89, 0b00001010];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "Calibrate", &data);

    // wait for calibration to finish - this is not documented
    loop {
        let mut data = [0xc0, 0]; // GetStatus
        spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
        print_response(&mut stim.borrow_mut(), "GetStatus", &data);
        if (data[0] >> 2) & 0x7 != 0 {
            break;
        }
    }

    // SetPacketType(PACKET_TYPE_GFSK)
    let mut data = [0x8a, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetPacketType", &data);

    // SetRfFrequency(rfFrequency)
    let mut data = [0x86, 0xb8, 0x9d, 0x89]; // frequency = parameter * 203125 / 1024
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetRfFrequency", &data);

    // SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
    let mut data = [0x8f, 0x80, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetBufferBaseAddress", &data);

    // TODO: good values?
    // SetModulationParams(...)
    let mut data = [0x8b, 0x04, 0x00, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetModulationParams", &data);

    // SetPacketParams(...)
    let mut data = [0x8c, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x08];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetPacketParams", &data);

    // SetTxParams(power, ramptime)
    let mut data = [0x8e, 0x1f, 0xe0];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetTxParams", &data);

    // SetStandby(STDBY_RC)
    let mut data = [0x80, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetStandby", &data);

    // send

    // // SetDioIrqParams(...)
    // let mut data = [0x8d, 64, 65, 64, 65, 0, 0, 0, 0];
    // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    // print_response(&mut stim.borrow_mut(), "SetDioIrqParams", &data);

    // // clear IRQ status. TODO: correct mask?
    // let mut data = [0x97, 0xff, 0xff];
    // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    // iprintln!(&mut stim.borrow_mut(), "response: {:?}", data);

    // SetTx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
    let mut data = [0x83, 0x00, 0x00, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetTx", &data);

    // GetPacketStatus()
    let mut data = [0x1d, 0, 0, 0, 0, 0, 0];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "GetPacketStatus", &data);
}

fn accel<'a>(
    spi: &mut Spi,
    dp: &'a pac::Peripherals,
    delay: &'a RefCell<delay::Delay>,
    stim: &'a RefCell<&'a mut itm::Stim>,
) {
    accel_write(spi, &mut delay.borrow_mut(), 0x10, 0b10100000); // CTRL1_XL, 6.66 kHz
    accel_write(spi, &mut delay.borrow_mut(), 0x11, 0b10100000); // CTRL2_G, 6.66 kHz

    let mut v = (0.0, 0.0, 0.0);
    let mut g = (0.0, 0.0, 0.0);

    loop {
        let data = read_accelerometer_data(spi, &mut delay.borrow_mut());
        v.0 += f32::from(data.acceleration.0) / 10000.0;
        v.1 += f32::from(data.acceleration.1) / 10000.0;
        v.2 += f32::from(data.acceleration.2) / 10000.0;
        let s = 0.000000014;
        g.0 += f32::from(data.gyroscope.0) * s;
        g.1 += f32::from(data.gyroscope.1) * s;
        g.2 += f32::from(data.gyroscope.2) * s;

        while g.0 < -0.5 {
            g.0 += 1.0;
        }
        while g.1 < -0.5 {
            g.1 += 1.0;
        }
        while g.2 < -0.5 {
            g.2 += 1.0;
        }
        while g.0 >= 0.5 {
            g.0 -= 1.0;
        }
        while g.1 >= 0.5 {
            g.1 -= 1.0;
        }
        while g.2 >= 0.5 {
            g.2 -= 1.0;
        }

        // iprintln!(&mut stim.borrow_mut(), "{:10.3} {:10.3} {:10.3}", v.0, v.1, v.2);
        iprintln!(
            &mut stim.borrow_mut(),
            "{:10.3} {:10.3} {:10.3}",
            g.0,
            g.1,
            g.2
        );

        let (a, b, c) = (g.0 * g.0, g.1 * g.1, g.2 * g.2);
        led(
            &dp,
            (a * LED_COUNTER_PERIOD as f32) as u32,
            (b * LED_COUNTER_PERIOD as f32) as u32,
            (b * LED_COUNTER_PERIOD as f32) as u32,
        );
    }
}

fn pull_up_unconnected_pins(dp: &pac::Peripherals) {
    use gpioa::pupdr::PUPDR0_A::PullUp;

    pin_mode_input!(dp.GPIOA, 0, PullUp);
    pin_mode_input!(dp.GPIOA, 8, PullUp);
    pin_mode_input!(dp.GPIOA, 9, PullUp);
    pin_mode_input!(dp.GPIOA, 15, PullUp);
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    pull_up_unconnected_pins(&dp);

    let stim = RefCell::new(&mut cp.ITM.stim[0]);
    let delay = RefCell::new(cortex_m::delay::Delay::new(cp.SYST, AHB_CLOCK_FREQUENCY));

    init_led(&dp);
    let mut spi = RefCell::new(Spi::new(&dp));

    led(&dp, LED_COUNTER_PERIOD / 2, 0, 0);

    init_radio(&mut spi.borrow_mut(), &delay, &stim);

    led(&dp, LED_COUNTER_PERIOD / 2, LED_COUNTER_PERIOD / 8, 0);

    // wait for accelerometer to become available
    iprintln!(&mut stim.borrow_mut(), "waiting for accelerometer...");
    while accel_read(&mut spi.borrow_mut(), &mut delay.borrow_mut(), 0x0f) != 0x6b {
        delay.borrow_mut().delay_ms(1);
    }

    led(&dp, 0, LED_COUNTER_PERIOD / 2, 0);

    // radio(&spi, &delay, &stim);
    // accel(&mut spi.borrow_mut(), &dp, &delay, &stim);

    gnss::gnss(
        &mut spi.borrow_mut(),
        &mut delay.borrow_mut(),
        &mut stim.borrow_mut(),
    );

    loop {}
}
