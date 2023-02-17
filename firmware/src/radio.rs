use core::cell::RefCell;
use cortex_m::{delay, iprintln, peripheral::itm};
use radio::Transmit;

use crate::{
    led, pac,
    radio_hal::RadioHal,
    spi::{self, Spi},
    LED_COUNTER_PERIOD,
};

pub fn spi_radio_transmit(spi: &mut Spi, delay: &mut delay::Delay, data: &mut [u8]) {
    spi.activate_peripheral(spi::Peripheral::Radio, delay);
    spi.transfer(data);
    spi.deactivate_peripheral();
}

pub fn print_response1(stim: &mut itm::Stim, response: u8) {
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

pub fn init_radio<'a>(
    spi: &mut Spi,
    delay: &'a RefCell<delay::Delay>,
    stim: &'a RefCell<&'a mut itm::Stim>,
) {
    // wait for radio to become available
    // let mut success_count = 0;
    loop {
        let mut data = [0x80, 0x00];
        spi_radio_transmit(spi, &mut delay.borrow_mut(), &mut data);
        if data[0] & 0b11111100 == 0b01000100 {
            break;
            // success_count += 1;
            // if success_count == 10 {
            //     break;
            // }
        } // else {
          //     success_count = 0;
          // }
        delay.borrow_mut().delay_ms(1);
    }
}

pub fn radio<'a>(
    dp: &'a pac::Peripherals,
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

    match radio.start_transmit(&data) {
        Ok(_) => led(&dp, 0, 0, LED_COUNTER_PERIOD / 2),
        Err(_) => led(&dp, LED_COUNTER_PERIOD / 2, 0, LED_COUNTER_PERIOD / 2),
    }

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
