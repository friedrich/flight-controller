#![no_std]
#![no_main]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use stm32g4::stm32g4a1 as pac;
use core::ptr;

const INTERNAL_CLOCK_FREQUENCY: u32 = 16_000_000;
const LED_COUNTER_PERIOD: u32 = 16_000;

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
    const PRESCALER_PERIOD: u16 = (INTERNAL_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16; // TODO: reverse calculation, since the rounding error can be big

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

fn led(p: &pac::Peripherals, red: u16, green: u16, blue: u16) {
    // set duty cycle
    p.TIM3.ccr1().modify(|_, w| w.ccr().variant(u32::from(red)));
    p.TIM3.ccr2().modify(|_, w| w.ccr().variant(u32::from(green)));
    p.TIM3.ccr4().modify(|_, w| w.ccr().variant(u32::from(blue)));
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
    p.GPIOA.afrl.modify(|_, w| w.afrl2().af1());   // AF1:  TIM2_CH3, AF9: TIM15_CH1
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
    const PRESCALER_PERIOD: u16 = (INTERNAL_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16;

    // set prescaler period
    p.TIM2.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    p.TIM4.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    p.TIM2.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    p.TIM4.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));

    const MIN_DUTY_PERCENT: u32 = 50; // measured: 49%
    const MAX_DUTY_PERCENT: u32 = 90; // measured: 87%

    let left_duty_percent = u32::from(left_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let right_duty_percent = u32::from(right_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let back_duty_percent = u32::from(back_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;

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

// PA4: !ACCEL_CS
// PA3: !RADIO_CS
// PB0: !GNSS_CS, max 10 MHz

// PA5: SCK
// PA6: CONTROLLER_SDI
// PA7: CONTROLLER_SDO

fn init_spi(p: &pac::Peripherals) {
    // enable IO port A clock
    p.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());

    // enable SPI1 clock
    p.RCC.apb2enr.modify(|_, w| w.spi1en().enabled());

    // TODO: wait for the clock to become active?

    // 1. Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins.

    // disable pull-up and pull-down resistors
    p.GPIOA.pupdr.modify(|_, w| w.pupdr5().floating());
    p.GPIOA.pupdr.modify(|_, w| w.pupdr6().floating());
    p.GPIOA.pupdr.modify(|_, w| w.pupdr7().floating());

    // set push pull output type
    p.GPIOA.otyper.modify(|_, w| w.ot5().push_pull());
    p.GPIOA.otyper.modify(|_, w| w.ot6().push_pull());
    p.GPIOA.otyper.modify(|_, w| w.ot7().push_pull());

    // set very high speed
    p.GPIOA.ospeedr.modify(|_, w| w.ospeedr5().very_high_speed());
    p.GPIOA.ospeedr.modify(|_, w| w.ospeedr6().very_high_speed());
    p.GPIOA.ospeedr.modify(|_, w| w.ospeedr7().very_high_speed());

    // set alternate mode
    p.GPIOA.moder.modify(|_, w| w.moder5().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder6().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder7().alternate());

    // set alternate function
    p.GPIOA.afrl.modify(|_, w| w.afrl5().af5()); // SPI1_SCK
    p.GPIOA.afrl.modify(|_, w| w.afrl6().af5()); // SPI1_MISO
    p.GPIOA.afrl.modify(|_, w| w.afrl7().af5()); // SPI1_MOSI

    // 2. Write to the SPI_CR1 register:

    p.SPI1.cr1.modify(|_, w| w.br().variant(0b111)); // slow baut rate
    p.SPI1.cr1.modify(|_, w| w.cpol().set_bit()); // clock is high when idle
    p.SPI1.cr1.modify(|_, w| w.cpha().set_bit()); // the second clock transition is the first data capture edge
    p.SPI1.cr1.modify(|_, w| w.rxonly().clear_bit().bidimode().clear_bit()); // full duplex mode
    p.SPI1.cr1.modify(|_, w| w.lsbfirst().clear_bit()); // MSB first
    p.SPI1.cr1.modify(|_, w| w.ssm().set_bit()); // software chip select management
    p.SPI1.cr1.modify(|_, w| w.ssi().set_bit()); // enable internal chip select
    p.SPI1.cr1.modify(|_, w| w.mstr().set_bit()); // master configuration

    // 3. Write to SPI_CR2 register:

    // p.SPI1.cr2.modify(|_, w| w.ds().variant(0b1111)); // 16 bit data size
    p.SPI1.cr2.modify(|_, w| w.ds().variant(0b0111)); // 8 bit data size
    p.SPI1.cr2.modify(|_, w| w.ssoe().clear_bit()); // disable chip select output
    // p.SPI1.cr2.modify(|_, w| w.frxth().clear_bit()); // RXNE event is generated if the FIFO level is greater than or equal to 1/2 (16-bit)
    p.SPI1.cr2.modify(|_, w| w.frxth().set_bit()); // RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)

    // enable SPI
    p.SPI1.cr1.modify(|_, w| w.spe().set_bit());
    
    // ACCEL CS - TODO: switch to hardware select

    // disable pull-up and pull-down resistors
    p.GPIOA.pupdr.modify(|_, w| w.pupdr4().floating());

    // set state
    p.GPIOA.odr.modify(|_, w| w.odr4().high());

    // set output mode
    p.GPIOA.moder.modify(|_, w| w.moder4().output());
}

fn accel_read_multiple(p: &pac::Peripherals, address: u8, data: &mut [u8]) {
    p.GPIOA.odr.modify(|_, w| w.odr4().low());
    // TODO: delay? (5 ns needed before clock goes low)

    // need to read and write single bytes
    let dr = p.SPI1.dr.as_ptr() as *mut u8;

    unsafe { ptr::write_volatile(dr, 1 << 7 | address) };
    while !p.SPI1.sr.read().rxne().bit() {}
    unsafe { ptr::read_volatile(dr) };

    for x in data {
        unsafe { ptr::write_volatile(dr, 0) };
        while !p.SPI1.sr.read().rxne().bit() {}
        *x = unsafe { ptr::read_volatile(dr) };
    }

    // TODO: delay? (20 ns needed after clock goes high)
    p.GPIOA.odr.modify(|_, w| w.odr4().high());
}

fn accel_write_multiple(p: &pac::Peripherals, address: u8, data: &[u8]) {
    p.GPIOA.odr.modify(|_, w| w.odr4().low());
    // TODO: delay? (5 ns needed before clock goes low)

    // need to read and write single bytes
    let dr = p.SPI1.dr.as_ptr() as *mut u8;

    unsafe { ptr::write_volatile(dr, address) };

    for x in data {
        while !p.SPI1.sr.read().txe().bit() {}
        unsafe { ptr::write_volatile(dr, *x) };
    }

    while p.SPI1.sr.read().bsy().bit() {}
    while p.SPI1.sr.read().rxne().bit() {
        unsafe { ptr::read_volatile(dr) };
    }

    // TODO: delay? (20 ns needed after clock goes high)
    p.GPIOA.odr.modify(|_, w| w.odr4().high());
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

#[entry]
fn main() -> ! {
    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = pac::Peripherals::take().unwrap();

    let mut delay = cortex_m::delay::Delay::new(cp.SYST, 16_000_000); // TODO: is this frequency exposed in some library?

    init_led(&dp);
    led(&dp, 8000, 0, 0);

    init_spi(&dp);

    // let mut a = [0, 0, 0];
    // loop {
    //     accel_read_multiple(&dp, 0x0e, &mut a);
    //     // hprintln!("{:?}", a);
    //     // while p.SPI1.sr.read().bsy().bit() {}

    //     // for x in 0..10_000 { }
    // }

    delay.delay_us(200); // needs to be about 100us for the accelerometer to respond properly

    if accel_read(&dp, 0x0f) != 0x6b {
        panic!("invalid response from accelerometer");
    }

    // The accelerometer is activated from power-down by writing ODR_XL[3:0] in CTRL1_XL (10h) while the gyroscope is activated from power-down by writing ODR_G[3:0] in CTRL2_G (11h). For combo-mode the ODRs are totally independent.
    accel_write(&dp, 0x10, 0b10100000); // CTRL1_XL, 6.66 kHz
    accel_write(&dp, 0x11, 0b10100000); // CTRL2_G, 6.66 kHz

    servos(&dp, 40, 40, 40);
    led(&dp, 0, 10, 0);

    loop {
        let x = (u16::from(accel_read(&dp, 0x29)) << 8 | u16::from(accel_read(&dp, 0x28))) as i16; // x
        let y = (u16::from(accel_read(&dp, 0x2b)) << 8 | u16::from(accel_read(&dp, 0x2a))) as i16; // y
        let z = (u16::from(accel_read(&dp, 0x2d)) << 8 | u16::from(accel_read(&dp, 0x2c))) as i16; // z
        led(&dp, (x.abs() / 10) as u16, (y.abs() / 10) as u16, (z.abs() / 10) as u16);
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
