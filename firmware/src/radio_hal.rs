use core::cell::RefCell;

use cortex_m::{iprintln, delay};
use cortex_m::peripheral::itm;
use embedded_hal::digital::PinState;
use radio_sx128x::base::Hal;
use stm32g4::stm32g4a1 as pac;

use crate::{spi_radio_transmit, print_response1};

pub struct RadioHal<'a> {
    pub dp: &'a pac::Peripherals,
    pub delay: &'a RefCell<delay::Delay>,
    pub stim: &'a RefCell<&'a mut itm::Stim>,
}

impl<'a> Hal for RadioHal<'a> {
    type CommsError = ();

    type PinError = ();

    type DelayError = ();

    fn reset(
        &mut self,
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(&mut self.stim.borrow_mut(), "reset()");
        Ok(())
    }

    fn get_busy(
        &mut self,
    ) -> Result<PinState, radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>>
    {
        unimplemented!("get_busy");
    }

    fn get_dio(
        &mut self,
    ) -> Result<PinState, radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>>
    {
        unimplemented!("get_dio");
    }

    fn delay_ms(&mut self, _ms: u32) -> Result<(), Self::DelayError> {
        unimplemented!("delay_ms");
    }

    fn delay_us(&mut self, _us: u32) -> Result<(), Self::DelayError> {
        unimplemented!("delay_us");
    }

    fn write_cmd(
        &mut self,
        command: u8,
        data: &[u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "write_cmd(0x{:02x}, {:?})",
            command,
            data
        );

        let mut buffer = [0; 256];
        let mut buffer = &mut buffer[0..data.len() + 1];

        buffer[0] = command;
        buffer[1..].copy_from_slice(data);

        spi_radio_transmit(self.dp, &mut self.delay.borrow_mut(), &mut buffer);

        // print_response1(&mut self.stim.borrow_mut(), buffer[0]);
        for x in buffer {
            print_response1(&mut self.stim.borrow_mut(), *x);
        }

        Ok(())
    }

    fn read_cmd(
        &mut self,
        command: u8,
        data: &mut [u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "read_cmd(0x{:02x}, {:?})",
            command,
            data
        );

        let mut buffer = [0; 256];
        let mut buffer = &mut buffer[0..data.len() + 1];

        buffer[0] = command;
        buffer[1..].copy_from_slice(data);

        spi_radio_transmit(self.dp, &mut self.delay.borrow_mut(), &mut buffer);
        data.copy_from_slice(&buffer[1..]);

        for x in buffer {
            print_response1(&mut self.stim.borrow_mut(), *x);
        }

        Ok(())
    }

    fn write_regs(
        &mut self,
        reg: u16,
        data: &[u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "write_regs(0x{:04x}, {:?})",
            reg,
            data
        );
        Ok(())
    }

    fn read_regs(
        &mut self,
        reg: u16,
        data: &mut [u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "read_regs(0x{:04x}, {:?})",
            reg,
            data
        );
        data[0] = 0xa9;
        data[1] = 0xb5;
        Ok(())
    }

    fn write_buff(
        &mut self,
        offset: u8,
        data: &[u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "write_buff(0x{:02x}, {:?})",
            offset,
            data
        );
        Ok(())
    }

    fn read_buff(
        &mut self,
        offset: u8,
        data: &mut [u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "read_buff(0x{:02x}, {:?})",
            offset,
            data
        );
        Ok(())
    }

    fn prefix_read(
        &mut self,
        prefix: &[u8],
        data: &mut [u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "prefix_read({:?}, {:?})",
            prefix,
            data
        );
        unimplemented!("prefix_read");
    }

    fn prefix_write(
        &mut self,
        prefix: &[u8],
        data: &[u8],
    ) -> Result<(), radio_sx128x::Error<Self::CommsError, Self::PinError, Self::DelayError>> {
        iprintln!(
            &mut self.stim.borrow_mut(),
            "prefix_write({:?}, {:?})",
            prefix,
            data
        );
        unimplemented!("prefix_write");
    }
}
