use core::ptr;

use cortex_m::delay;

use crate::pac;
use crate::pac::{gpioa, gpiob};
use crate::{pin_mode_alternate_l, pin_mode_output};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Peripheral {
    Gnss,
    Radio,
    Imu,
}

pub struct Spi<'a> {
    current_peripheral: Option<Peripheral>,
    dp: &'a pac::Peripherals,
}

impl<'a> Spi<'a> {
    pub fn new(dp: &'a pac::Peripherals) -> Spi<'a> {
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
        pin_mode_alternate_l!(dp.GPIOA, 5, APushPull, AFloating, AVeryHighSpeed, Af5);
        pin_mode_alternate_l!(dp.GPIOA, 6, APushPull, AFloating, AVeryHighSpeed, Af5);
        pin_mode_alternate_l!(dp.GPIOA, 7, APushPull, AFloating, AVeryHighSpeed, Af5);

        // configure SPI
        dp.SPI1.cr1.modify(|_, w| w.rxonly().clear_bit().bidimode().clear_bit()); // full duplex mode
        dp.SPI1.cr1.modify(|_, w| w.lsbfirst().clear_bit()); // MSB first
        dp.SPI1.cr1.modify(|_, w| w.ssm().set_bit()); // software chip select management
        dp.SPI1.cr1.modify(|_, w| w.ssi().set_bit()); // enable internal chip select
        dp.SPI1.cr1.modify(|_, w| w.mstr().set_bit()); // act as controller
        dp.SPI1.cr2.modify(|_, w| w.ds().variant(0b0111)); // 8 bit data size
        dp.SPI1.cr2.modify(|_, w| w.ssoe().clear_bit()); // disable chip select output
        dp.SPI1.cr2.modify(|_, w| w.frxth().set_bit()); // RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)

        Spi {
            current_peripheral: None,
            dp,
        }
    }

    pub fn activate_peripheral(&mut self, peripheral: Peripheral, delay: &mut delay::Delay) {
        self.switch_peripheral(peripheral);

        match peripheral {
            Peripheral::Gnss => {
                self.dp.GPIOB.odr.modify(|_, w| w.odr0().low());
            }
            Peripheral::Radio => {
                self.dp.GPIOA.odr.modify(|_, w| w.odr3().low());
                delay.delay_us(125_u32); // TODO: correct?
            }
            Peripheral::Imu => {
                self.dp.GPIOA.odr.modify(|_, w| w.odr4().low());
            }
        }
        // TODO: delay?
    }

    pub fn deactivate_peripheral(&mut self) {
        // TODO: delay?
        match self.current_peripheral {
            Some(Peripheral::Gnss) => {
                self.dp.GPIOB.odr.modify(|_, w| w.odr0().high());
            }
            Some(Peripheral::Radio) => {
                self.dp.GPIOA.odr.modify(|_, w| w.odr3().high());
            }
            Some(Peripheral::Imu) => {
                self.dp.GPIOA.odr.modify(|_, w| w.odr4().high());
            }
            None => {}
        }
    }

    pub fn read(&self) -> u8 {
        while self.dp.SPI1.sr.read().frlvl() == 0 {}
        unsafe { ptr::read_volatile(self.dp.SPI1.dr.as_ptr() as *mut u8) }
    }

    pub fn write(&mut self, value: u8) {
        while self.dp.SPI1.sr.read().txe().bit_is_clear() {}
        unsafe { ptr::write_volatile(self.dp.SPI1.dr.as_ptr() as *mut u8, value) };
    }

    pub fn transfer(&mut self, data: &mut [u8]) {
        for x in data {
            self.write(*x);
            *x = self.read();
        }
    }

    fn switch_peripheral(&mut self, peripheral: Peripheral) {
        if self.current_peripheral == Some(peripheral) {
            return;
        }

        disable_spi(self.dp);

        match peripheral {
            Peripheral::Gnss => {
                // max 5.5 MHz for ZOE-M8Q
                enable_spi(self.dp, 7, false, false);
            }
            Peripheral::Radio => {
                // max 18 MHz for SX1280
                enable_spi(self.dp, 0, false, false);
            }
            Peripheral::Imu => {
                // max 10 MHz for LSM6DSR
                enable_spi(self.dp, 0, true, true);
            }
        }

        self.current_peripheral = Some(peripheral);
    }
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
        dp.SPI1.dr.read().bits();
    }
}
