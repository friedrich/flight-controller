#![no_std]
#![no_main]

mod gnss;
mod radio;
mod spi;

use core::cell::RefCell;
use cortex_m::delay;
use cortex_m::iprintln;
use cortex_m::peripheral::itm;
use cortex_m_rt::entry;
use panic_itm as _;
use radio::{init_radio, radio};
use spi::Spi;
use stm32g4::stm32g4a1 as pac;
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
macro_rules! pin_mode_alternate_l {
    ($gpio:expr, $num:literal, $type:expr, $pull:expr, $speed:expr, $alt:expr) => {
        paste::paste! {
            $gpio.afrl.modify(|_, w| w.[<afrl $num>]().variant($alt));
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

    pin_mode_alternate_l!(dp.GPIOB, 4, OpenDrain, Floating, LowSpeed, Af2);
    pin_mode_alternate_l!(dp.GPIOB, 5, OpenDrain, Floating, LowSpeed, Af2);
    pin_mode_alternate_l!(dp.GPIOB, 7, OpenDrain, Floating, LowSpeed, Af10);

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

fn accel_read_multiple(spi: &mut Spi, delay: &mut delay::Delay, address: u8, data: &mut [u8]) {
    spi.activate_peripheral(spi::Peripheral::Imu, delay);

    // 5 ns needed before clock goes low - TODO: make this nicer
    cortex_m::asm::nop();

    spi.write(1 << 7 | address);
    spi.read();

    data.fill(0);
    spi.transfer(data);

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
            dp,
            (a * LED_COUNTER_PERIOD as f32) as u32,
            (b * LED_COUNTER_PERIOD as f32) as u32,
            (c * LED_COUNTER_PERIOD as f32) as u32,
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

    // initialize LEDs early to get some feedback
    init_led(&dp);
    led(&dp, LED_COUNTER_PERIOD / 2, 0, 0);

    pull_up_unconnected_pins(&dp);

    let stim = RefCell::new(&mut cp.ITM.stim[0]);
    let delay = RefCell::new(cortex_m::delay::Delay::new(cp.SYST, AHB_CLOCK_FREQUENCY));
    let spi = RefCell::new(Spi::new(&dp));

    // Communication with the radio peripheral has to happen very soon after
    // reset in order for it to detect SPI communication and not to switch to
    // UART and disturbe the SPI bus. This is currently only achieved when
    // compiling in release mode.
    init_radio(&mut spi.borrow_mut(), &delay);
    iprintln!(&mut stim.borrow_mut(), "radio found");

    led(&dp, LED_COUNTER_PERIOD / 2, LED_COUNTER_PERIOD / 8, 0);

    while accel_read(&mut spi.borrow_mut(), &mut delay.borrow_mut(), 0x0f) != 0x6b {
        delay.borrow_mut().delay_ms(1);
    }
    iprintln!(&mut stim.borrow_mut(), "IMU found");

    led(&dp, 0, LED_COUNTER_PERIOD / 2, 0);

    radio(&spi, &delay, &stim);
    accel(&mut spi.borrow_mut(), &dp, &delay, &stim);

    gnss::gnss(
        &dp,
        &mut spi.borrow_mut(),
        &mut delay.borrow_mut(),
        &mut stim.borrow_mut(),
    );

    #[allow(clippy::empty_loop)]
    loop {}
}
