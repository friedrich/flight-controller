#![no_std]
#![no_main]

use panic_halt as _;

use core::ptr;
use cortex_m::delay;
use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use paste::paste;
use stm32g4::stm32g4a1 as pac;
use stm32g4::stm32g4a1::{gpioa, gpiob};

const HSI16_CLOCK_FREQUENCY: u32 = 16_000_000;
const AHB_CLOCK_FREQUENCY: u32 = HSI16_CLOCK_FREQUENCY;
const LED_COUNTER_PERIOD: u32 = 16_000;

macro_rules! pin_mode_input {
    ($gpio:expr, $num:literal, $pull:expr) => {
        paste! {
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().input());
        }
    };
}

macro_rules! pin_mode_output {
    ($gpio:expr, $num:literal, $type:expr, $speed:expr, $state:expr) => {
        paste! {
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.odr.modify(|_, w| w.[<odr $num>]().bit($state));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().output());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().floating());
        }
    };
}

macro_rules! pin_mode_alternate {
    ($gpio:expr, $num:literal, $type:expr, $pull:expr, $speed:expr, $alt:expr) => {
        paste! {
            $gpio.afrl.modify(|_, w| w.[<afrl $num>]().variant($alt));
            // $gpio.afrh.modify(|_, w| w.[<afrh $num>]().variant($alt));
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().alternate());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
        }
    };
}

fn init_led(p: &pac::Peripherals) {
    // enable IO port B clock
    p.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

    // enable TIM3 timer clock
    p.RCC.apb1enr1.modify(|_, w| w.tim3en().enabled());

    // TODO: wait for the clock to become active?

    // set open drain output type
    p.GPIOB.otyper.modify(|_, w| w.ot4().open_drain());
    p.GPIOB.otyper.modify(|_, w| w.ot5().open_drain());
    p.GPIOB.otyper.modify(|_, w| w.ot7().open_drain());

    // disable pull-up and pull-down resistors
    p.GPIOB.pupdr.modify(|_, w| w.pupdr4().floating());
    p.GPIOB.pupdr.modify(|_, w| w.pupdr5().floating());
    p.GPIOB.pupdr.modify(|_, w| w.pupdr7().floating());

    // set alternate function
    p.GPIOB.afrl.modify(|_, w| w.afrl4().af2()); // TIM3_CH1
    p.GPIOB.afrl.modify(|_, w| w.afrl5().af2()); // TIM3_CH2
    p.GPIOB.afrl.modify(|_, w| w.afrl7().af10()); // TIM3_CH4

    // set alternate mode
    p.GPIOB.moder.modify(|_, w| w.moder4().alternate());
    p.GPIOB.moder.modify(|_, w| w.moder5().alternate());
    p.GPIOB.moder.modify(|_, w| w.moder7().alternate());

    const FREQUENCY: u32 = 1_000;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * LED_COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16; // TODO: reverse calculation, since the rounding error can be big

    // set prescaler period
    p.TIM3.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    p.TIM3.arr.modify(|_, w| w.arr().variant(LED_COUNTER_PERIOD - 1));

    // set duty cycle
    p.TIM3.ccr1().modify(|_, w| w.ccr().variant(0));
    p.TIM3.ccr2().modify(|_, w| w.ccr().variant(0));
    p.TIM3.ccr4().modify(|_, w| w.ccr().variant(0));

    // set PWM mode 1
    p.TIM3.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    p.TIM3.ccmr1_output().modify(|_, w| w.oc2m().bits(0b110));
    p.TIM3.ccmr2_output().modify(|_, w| w.oc4m().bits(0b110));

    // select active low polarity
    p.TIM3.ccer.modify(|_, w| w.cc1p().set_bit());
    p.TIM3.ccer.modify(|_, w| w.cc2p().set_bit());
    p.TIM3.ccer.modify(|_, w| w.cc4p().set_bit());

    // enable output
    p.TIM3.ccer.modify(|_, w| w.cc1e().set_bit());
    p.TIM3.ccer.modify(|_, w| w.cc2e().set_bit());
    p.TIM3.ccer.modify(|_, w| w.cc4e().set_bit());

    // enable counter
    p.TIM3.cr1.modify(|_, w| w.cen().set_bit());
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

// PB0: !GNSS_CS, max 10 MHz

// fn init_clocks(dp: &pac::Peripherals, delay: &mut delay::Delay) {
//     // enable IO port clocks
//     dp.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());
//     dp.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

//     // enable timer clocks
//     dp.RCC.apb1enr1.modify(|_, w| w.tim2en().enabled());
//     dp.RCC.apb1enr1.modify(|_, w| w.tim3en().enabled());
//     dp.RCC.apb1enr1.modify(|_, w| w.tim4en().enabled());
// }

fn blub() {
    // The debug pins are in AF pull-up/pull-down after reset:
    // - PA15: JTDI in pull-up
    // - PA14: JTCK/SWCLK in pull-down
    // - PA13: JTMS/SWDAT in pull-up
    // - PB4: NJTRST in pull-up
    // - PB3: JTDO in floating state no pull-up/pull-down
    // PB8/BOOT0 is in input mode during the reset until at least the end of the option byte loading phase. See Section 9.3.15: Using PB8 as GPIO.
}

fn init_spi(dp: &pac::Peripherals) {
    use gpioa::afrl::AFRL0_A::Af5;
    use gpioa::ospeedr::OSPEEDR0_A::VeryHighSpeed as AVeryHighSpeed;
    use gpioa::otyper::OT0_A::PushPull as APushPull;
    use gpioa::pupdr::PUPDR0_A::Floating as AFloating;
    use gpiob::ospeedr::OSPEEDR0_A::VeryHighSpeed as BVeryHighSpeed;
    use gpiob::otyper::OT0_A::PushPull as BPushPull;

    // enable clocks
    dp.RCC.apb2enr.modify(|_, w| w.spi1en().enabled());
    dp.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());
    dp.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

    // configure CS pins
    pin_mode_output!(dp.GPIOA, 3, APushPull, AVeryHighSpeed, true);
    pin_mode_output!(dp.GPIOA, 4, APushPull, AVeryHighSpeed, true);
    pin_mode_output!(dp.GPIOB, 0, BPushPull, BVeryHighSpeed, true);

    // configure SPI pins
    pin_mode_alternate!(dp.GPIOA, 5, APushPull, AFloating, AVeryHighSpeed, Af5);
    pin_mode_alternate!(dp.GPIOA, 6, APushPull, AFloating, AVeryHighSpeed, Af5);
    pin_mode_alternate!(dp.GPIOA, 7, APushPull, AFloating, AVeryHighSpeed, Af5);

    // configure SPI
    dp.SPI1.cr1.modify(|_, w| w.rxonly().clear_bit().bidimode().clear_bit()); // full duplex mode
    dp.SPI1.cr1.modify(|_, w| w.lsbfirst().clear_bit()); // MSB first
    dp.SPI1.cr1.modify(|_, w| w.ssm().set_bit()); // software chip select management
    dp.SPI1.cr1.modify(|_, w| w.ssi().set_bit()); // enable internal chip select
    dp.SPI1.cr1.modify(|_, w| w.mstr().set_bit()); // act as controller
    dp.SPI1.cr2.modify(|_, w| w.ds().variant(0b0111)); // 8 bit data size
    dp.SPI1.cr2.modify(|_, w| w.ssoe().clear_bit()); // disable chip select output
    dp.SPI1.cr2.modify(|_, w| w.frxth().set_bit()); // RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
}

// baud rate = f_PCLK / 2^(baud_rate_setting+1)
// baud_rate_setting <= 7
fn enable_spi(dp: &pac::Peripherals, baud_rate_setting: u8, cpol: bool, cpha: bool) {
    // configure SPI
    dp.SPI1.cr1.modify(|_, w| w.br().variant(baud_rate_setting));
    dp.SPI1.cr1.modify(|_, w| w.cpol().bit(cpol));
    dp.SPI1.cr1.modify(|_, w| w.cpha().bit(cpha));

    // enable SPI
    dp.SPI1.cr1.modify(|_, w| w.spe().set_bit());
}

fn disable_spi(dp: &pac::Peripherals) {
    while dp.SPI1.sr.read().ftlvl() != 0 {}
    while dp.SPI1.sr.read().bsy().bit_is_set() {}
    dp.SPI1.cr1.modify(|_, w| w.spe().clear_bit());
    while dp.SPI1.sr.read().frlvl() != 0 {
        // TODO: read data
    }
}

const SPI_FREQUENCY_SETTING_ACCEL: u8 = 0;

fn accel_read_multiple(dp: &pac::Peripherals, address: u8, data: &mut [u8]) {
    enable_spi(dp, SPI_FREQUENCY_SETTING_ACCEL, true, true); // max 10 MHz for accelerometer

    dp.GPIOA.odr.modify(|_, w| w.odr4().low());

    // 5 ns needed before clock goes low - TODO: make this nicer
    cortex_m::asm::nop();

    // need to read and write single bytes
    let dr = dp.SPI1.dr.as_ptr() as *mut u8;

    unsafe { ptr::write_volatile(dr, 1 << 7 | address) };
    while dp.SPI1.sr.read().frlvl() == 0 {}
    unsafe { ptr::read_volatile(dr) };

    for x in data {
        unsafe { ptr::write_volatile(dr, 0) };
        while dp.SPI1.sr.read().frlvl() == 0 {}
        *x = unsafe { ptr::read_volatile(dr) };
    }

    // 20 ns needed before clock goes high - TODO: make this nicer
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    dp.GPIOA.odr.modify(|_, w| w.odr4().high());

    disable_spi(dp);
}

fn accel_write_multiple(dp: &pac::Peripherals, address: u8, data: &[u8]) {
    enable_spi(dp, SPI_FREQUENCY_SETTING_ACCEL, true, true); // max 10 MHz for accelerometer

    dp.GPIOA.odr.modify(|_, w| w.odr4().low());
    // TODO: delay? (5 ns needed before clock goes low)

    // need to read and write single bytes
    let dr = dp.SPI1.dr.as_ptr() as *mut u8;

    unsafe { ptr::write_volatile(dr, address) };

    for x in data {
        while !dp.SPI1.sr.read().txe().bit() {}
        unsafe { ptr::write_volatile(dr, *x) };
    }

    while dp.SPI1.sr.read().bsy().bit() {}
    while dp.SPI1.sr.read().frlvl() != 0 {
        unsafe { ptr::read_volatile(dr) };
    }

    // TODO: delay? (20 ns needed after clock goes high)
    dp.GPIOA.odr.modify(|_, w| w.odr4().high());

    disable_spi(dp);
}

fn accel_read(p: &pac::Peripherals, address: u8) -> u8 {
    let mut data = [0];
    accel_read_multiple(p, address, &mut data);
    data[0]
}

fn accel_write(p: &pac::Peripherals, address: u8, data: u8) {
    let data = [data];
    accel_write_multiple(p, address, &data);
}

struct AccelerometerData {
    acceleration: (i16, i16, i16),
    gyroscope: (i16, i16, i16),
}

fn read_accelerometer_data(dp: &pac::Peripherals) -> AccelerometerData {
    let mut data = [0; 12];

    accel_read_multiple(&dp, 0x22, &mut data);

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

fn spi_radio_transmit(dp: &pac::Peripherals, delay: &mut delay::Delay, data: &mut [u8]) {
    enable_spi(dp, 0, false, false); // max 18 MHz according to datasheet of SX1280

    // need to read and write single bytes
    let dr = dp.SPI1.dr.as_ptr() as *mut u8;

    dp.GPIOA.odr.modify(|_, w| w.odr3().low());
    delay.delay_us(125_u32); // TODO: correct?

    for x in data {
        unsafe { ptr::write_volatile(dr, *x) };
        while dp.SPI1.sr.read().frlvl() == 0 {}
        *x = unsafe { ptr::read_volatile(dr) };
    }

    // TODO: delay?
    dp.GPIOA.odr.modify(|_, w| w.odr3().high());

    disable_spi(dp);
}

#[entry]
fn main() -> ! {
    let mut cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, AHB_CLOCK_FREQUENCY);

    init_led(&dp);
    led(&dp, LED_COUNTER_PERIOD / 2, 0, 0);

    init_spi(&dp);

    // wait for accelerometer to become available
    while accel_read(&dp, 0x0f) != 0x6b {
        delay.delay_ms(1);
    }

    accel_write(&dp, 0x10, 0b10100000); // CTRL1_XL, 6.66 kHz
    accel_write(&dp, 0x11, 0b10100000); // CTRL2_G, 6.66 kHz

    // wait for radio to become available
    loop {
        let address: u16 = 0x9ce;
        let mut data = [0x18, (address >> 8) as u8, address as u8, 1, 2, 3, 4, 5];
        spi_radio_transmit(&dp, &mut delay, &mut data);
        // hprintln!("status: {:?}", &data);

        let address: u16 = 0x9ce;
        let mut data = [0x19, (address >> 8) as u8, address as u8, 0, 0, 0, 0, 0, 0];
        spi_radio_transmit(&dp, &mut delay, &mut data);
        let data = &data[4..];
        // hprintln!("data: {:?}", data);
        if data == [1, 2, 3, 4, 5] {
            break;
        }

        delay.delay_ms(10); // not necessary
    }

    // let mut prev = None;
    let mut cycle = 0;
    let mut min_diff = 1000000;

    cp.DWT.enable_cycle_counter();
    // cortex_m::peripheral::DWT::enable_cycle_counter(&mut cp.DWT);

    // loop {
    //     let mut values = [0; 10];
    //     let mut times = [0; 10];

    //     cp.DWT.set_cycle_count(0);
    //     for i in 0..values.len() {
    //         let accel = read_accelerometer_data(&dp);
    //         values[i] = accel.acceleration.0;
    //         times[i] = cortex_m::peripheral::DWT::cycle_count() / (HSI16_CLOCK_FREQUENCY / 1000000);
    //     }
    //     hprintln!("{:?}", values);
    //     hprintln!("{:?}", times);
    // }

    //

    //    match prev {
    //     Some(p) if p != x => {
    //         let new_cycle = cortex_m::peripheral::DWT::cycle_count();
    //         let diff = new_cycle - cycle;
    //         cycle = new_cycle;
    //         prev = Some(x);

    //         if diff < min_diff {
    //             hprintln!("{}", diff);
    //             min_diff = diff;
    //         }
    //     }
    //     None => {
    //         prev = Some(x);
    //     }
    //     _ => {}
    // }

    // servos(&dp, 40, 40, 40);

    let mut angle = 0;

    loop {
        let v = read_accelerometer_data(&dp).acceleration;
        led(
            &dp,
            (v.0.abs() / 10) as u32,
            (v.1.abs() / 10) as u32,
            (v.2.abs() / 10) as u32,
        );

        // let v = read_accelerometer_data(&dp).gyroscope;
        // angle += v.2;
        // led(
        //     &dp, 0, 0,
        //     (angle.abs() / 10) as u32,
        // );
        // hprintln!("{}", angle as f32 / 45000.0);
    }

    // led(&dp, 0, 10, 0);
    // loop {
    //     led(&dp, 0, 10, 0);
    //     servos(&dp, 40, 40, 40);
    //     for x in 0..100_000 { }
    //     led(&dp, 0, 0, 10);
    //     servos(&dp, 40, 60, 40);
    //     for x in 0..100_000 { }
    // }
}
