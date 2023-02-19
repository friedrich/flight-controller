use core::cell::RefCell;
use cortex_m::{delay, iprintln, peripheral::itm};

const BLE_FREQUENCY_CH37: u32 = 2_402_000_000;
#[allow(unused)]
const BLE_FREQUENCY_CH38: u32 = 2_426_000_000;
#[allow(unused)]
const BLE_FREQUENCY_CH39: u32 = 2_480_000_000;

#[allow(unused)]
const ADV_IND: u8 = 0b0000;
#[allow(unused)]
const ADV_DIRECT_IND: u8 = 0b0001;
const ADV_NONCONN_IND: u8 = 0b0010;
#[allow(unused)]
const ADV_SCAN_IND: u8 = 0b0110;

use crate::spi::{self, Spi};

pub fn spi_radio_transmit(spi: &mut Spi, delay: &mut delay::Delay, data: &mut [u8]) {
    spi.activate_peripheral(spi::Peripheral::Radio, delay);
    spi.transfer(data);
    spi.deactivate_peripheral();
}

fn print_response(stim: &mut itm::Stim, label: &str, data: &[u8]) {
    iprintln!(stim, "{}:", label);
    let circuit = data[0] >> 5;
    let status = (data[0] >> 2) & 0x7;
    iprintln!(stim, " - {:x} {:x} - {:02x?}", circuit, status, &data[1..]);
}

pub fn init_radio(spi: &mut Spi, delay: &RefCell<delay::Delay>) {
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

    // let radio_config = radio_sx128x::Config::gfsk();

    // // write_cmd(0x80, [00])
    // // write_cmd(0x96, [00])
    // // write_cmd(0x86, [bb, b1, 3b])
    // // write_cmd(0x8a, [00])
    // // write_cmd(0x8b, [c7, 00, 00])
    // // write_cmd(0x8c, [70, 08, 10, 20, ff, 00, 08])
    // // write_cmd(0x8e, [1c, e0])
    // // write_cmd(0x80, [00])
    // let mut radio = radio_sx128x::Sx128x::new(
    //     RadioHal {
    //         spi,
    //         delay,
    //         stim: &stim,
    //     },
    //     &radio_config,
    // )
    // .unwrap();

    // // radio.calibrate(radio_sx128x::device::CalibrationParams::all()).unwrap();
    // // delay.borrow_mut().delay_ms(100);

    // let data = [1, 2, 3];
    // // write_cmd(0x80, [00])
    // // read_cmd(0xc0, [00])
    // // write_cmd(0x8c, [70, 08, 10, 20, 03, 00, 08])
    // // write_cmd(0x8f, [00, 00])
    // // write_buff(0x00, [01, 02, 03])
    // // write_cmd(0x8d, [40, 41, 40, 41, 00, 00, 00, 00])
    // // write_cmd(0x83, [00, 00, 00])
    // // read_cmd(0xc0, [00])
    // match radio.start_transmit(&data) {
    //     Ok(_) => led(&dp, 0, 0, LED_COUNTER_PERIOD / 2),
    //     Err(_) => led(&dp, LED_COUNTER_PERIOD / 2, 0, LED_COUNTER_PERIOD / 2),
    // }

    // loop {}

    // TODO: ??????
    // To perform the calibration the Calibrate( calibParam ) function, opcode 0x89, must be used with the calibration parameters configured as follows:
    // calibParam.ADCBulkPEnable = 1; calibParam.ADCBulkNEnable = 1; calibParam.ADCPulseEnable = 1; calibParam.PLLEnable = 1; calibParam.RC13MEnable = 1; calibParam.RC64KEnable = 1;
    // Then we call the calibration function:
    // Radio.Calibrate( calibParam );

    // common transceiver settings

    // 1. If not in STDBY_RC mode, then go to this mode

    // SetStandby(STDBY_RC)
    let mut data = [0x80, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetStandby", &data);

    // // TODO: do we need to calibrate the internal RC oscillator?
    // // Calibrate()
    // // let mut data = [0x89, 0b00111111];
    // let mut data = [0x89, 0b00001010];
    // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    // print_response(&mut stim.borrow_mut(), "Calibrate", &data);

    // // wait for calibration to finish - this is not documented
    // loop {
    //     let mut data = [0xc0, 0]; // GetStatus
    //     spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    //     print_response(&mut stim.borrow_mut(), "GetStatus", &data);
    //     if (data[0] >> 2) & 0x7 != 0 {
    //         break;
    //     }
    // }

    // 2. Define BLE packet

    // SetPacketType(PACKET_TYPE_BLE)
    let mut data = [0x8a, 0x04];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetPacketType", &data);

    // 3. Define the RF frequency

    // SetRfFrequency(rfFrequency)
    let frequency = BLE_FREQUENCY_CH37;
    let arg: u64 = u64::from(frequency) * 1024 / 203125;
    let mut data = [0x86, (arg >> 16) as u8, (arg >> 8) as u8, arg as u8];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetRfFrequency", &data);

    // 4. Indicate the addresses where the packet handler will read or write

    const TX_BASE_ADDRESS: u8 = 0x80;

    // SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
    let mut data = [0x8f, TX_BASE_ADDRESS, 0x00];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetBufferBaseAddress", &data);

    // 5. Define the modulation parameter

    // SetModulationParams(BLE_BR_1_000_BW_1_2, MOD_IND_0_5, BT_0_5)
    let bitrate_and_bandwidth = 0x45; // 1 Mb/s, 1.2 MHz
    let modulation_index = 0x01; // 0.5
    let pulse_shape = 0x20; // 0.5
    let mut data = [0x8b, bitrate_and_bandwidth, modulation_index, pulse_shape];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetModulationParams", &data);

    // 6. Define the packet parameters

    // SetPacketParams(...)
    let max_payload_length = 0x00; // 31 bytes
    let crc = 0x00; // off - TODO!
                    // let crc = 0x10; // 3 bytes
    let whitening = 0x00; // on
    let mut data = [
        0x8c,
        max_payload_length,
        crc,
        0x00,
        whitening,
        0x00,
        0x00,
        0x00,
    ];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetPacketParams", &data);

    // 7. Define the Access Address value
    // WriteRegister(Sync Address 1 Byte 3 .. Byte 0, 0x8e89bed6)
    let address = 0x9cf;
    let magic: u32 = 0x8e89bed6;
    let mut data = [
        0x18,
        (address >> 8) as u8,
        address as u8,
        (magic >> 24) as u8,
        (magic >> 16) as u8,
        (magic >> 8) as u8,
        magic as u8,
    ];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "WriteRegister", &data);

    // TODO: CRC init?

    // tx settings

    // 1. Define the BLE Access Address accessAddress - already done

    // 2. Define the output power and ramp time

    // SetTxParams(power, ramptime)
    let power_param = 31; // tx power = power_param - 18
    let power_amplifier_ramp_time = 0xe0; // 20 us
    let mut data = [0x8e, power_param, power_amplifier_ramp_time];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetTxParams", &data);

    // // SetStandby(STDBY_RC)
    // let mut data = [0x80, 0x00];
    // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    // print_response(&mut stim.borrow_mut(), "SetStandby", &data);

    // 3. Send the payload to the data buffer by issuing the command

    // WriteBuffer(offset, data)
    let header0 = ADV_NONCONN_IND << 4 | 0b0011; // TODO?
    let payload_length = 10;
    // TODO: randomize device address
    let mut data = [
        0x1a,
        TX_BASE_ADDRESS,
        header0,
        payload_length << 2,
        1,
        2,
        3,
        4,
        5,
        6,
        1,
        2,
        3,
        4,
    ];
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "WriteBuffer", &data);

    // 4. Configure the DIOs and Interrupt sources (IRQs)

    // SetDioIrqParams(...)
    let mut data = [0x8d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]; // TODO?
    spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
    print_response(&mut stim.borrow_mut(), "SetDioIrqParams", &data);

    loop {
        // 5. Once configured, set the transceiver in transmitter mode to start transmission

        // SetTx(periodBase, periodBaseCount[15:8], periodBaseCount[7:0])
        let timeout_period_base = 0x00; // 15.625 μs
        let timeout_period_count = 0;
        let mut data = [
            0x83,
            timeout_period_base,
            (timeout_period_count >> 8) as u8,
            timeout_period_count as u8,
        ];
        spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
        print_response(&mut stim.borrow_mut(), "SetTx", &data);

        // 6. Optionally check the packet status to make sure that the packet has been sent properly

        loop {
            // GetPacketStatus()
            let mut data = [0x1d, 0, 0, 0, 0, 0, 0];
            spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
            let status = data[5];
            if status == 0x01 {
                break;
            }
            print_response(&mut stim.borrow_mut(), "GetPacketStatus", &data);

            delay.borrow_mut().delay_ms(100);
        }

        // 7. Clear TxDone or RxTxTimeout IRQ

        // // ClearIrqStatus()
        // let mut data = [0x97, 0xff, 0xff]; // TODO
        // spi_radio_transmit(&mut spi.borrow_mut(), &mut delay.borrow_mut(), &mut data);
        // print_response(&mut stim.borrow_mut(), "GetIrqStatus", &data);

        delay.borrow_mut().delay_ms(1000);
    }
}
