use crate::{pac, spi, Spi};
use cortex_m::{delay, iprint};
use cortex_m::{iprintln, peripheral::itm};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum UbxEncodeError {
    BufferTooSmall,
    PayloadTooBig,
}

const UBX_SYNC_CHAR_1: u8 = 0xb5;
const UBX_SYNC_CHAR_2: u8 = 0x62;

fn gnss_transmit(
    spi: &mut Spi,
    delay: &mut delay::Delay,
    stim: &mut itm::Stim,
    message_type: UbxMessageType,
    payload: &[u8],
    buffer: &mut [u8],
) -> Result<(), UbxEncodeError> {
    spi.activate_peripheral(spi::Peripheral::Gnss, delay);

    let len = ubx_encode(message_type, payload, buffer)?.len();
    buffer[len..].fill(0xff);

    spi.transfer(buffer);

    Ok(())
}

fn gnss_receive(
    spi: &mut Spi,
    delay: &mut delay::Delay,
    stim: &mut itm::Stim,
    buffer: &mut [u8],
) -> Result<(), UbxEncodeError> {
    spi.activate_peripheral(spi::Peripheral::Gnss, delay);

    buffer.fill(0xff);
    spi.transfer(buffer);

    Ok(())
}

enum ParseState {
    Idle,
    Ubx(usize),
    Nmea,
}

fn gnss_parse(stim: &mut itm::Stim, dp: &pac::Peripherals, data: &[u8]) {
    let mut state = ParseState::Idle;
    let mut message_start = 0;
    let mut message_type_id = 0;
    let mut message_size = 0;

    for pos in 0..data.len() {
        let x = data[pos];
        let peek = data.get(pos + 1).cloned().unwrap_or(0xff);

        match state {
            ParseState::Idle => {
                if x == UBX_SYNC_CHAR_1 && peek == UBX_SYNC_CHAR_2 {
                    message_start = pos;
                    message_size = 0;

                    // iprintln!(stim, "UBX message start at {}", pos);
                    state = ParseState::Ubx(1);
                } else if x == '$' as u8 {
                    message_start = pos;

                    // iprintln!(stim, "NMEA message start at {}", pos);
                    state = ParseState::Nmea;
                }
            }
            ParseState::Ubx(pos_in_message) => {
                if pos_in_message == message_size - 1 {
                    let message = &data[message_start..=pos];
                    let payload = &message[6..message.len() - 2];
                    iprint!(stim, "UBX message at {}: ", message_start);

                    match UbxMessageType::from_u16(message_type_id) {
                        Some(UbxMessageType::CfgNmea) => {
                            iprintln!(stim, "NMEA config: {:02x?}", payload);
                        }
                        Some(UbxMessageType::AckAck) => {
                            iprintln!(stim, "ack");
                        }
                        Some(UbxMessageType::AckNak) => {
                            iprintln!(stim, "nack");
                        }
                        Some(UbxMessageType::CfgPrt) => {
                            iprintln!(stim, "port config: {:02x?}", payload);
                        }
                        Some(UbxMessageType::CfgMsg) => {
                            iprintln!(stim, "message rate: {:02x?}", payload);
                        }
                        Some(UbxMessageType::NavStatus) => {
                            let itow = u32::from(payload[0])
                                | u32::from(payload[1]) << 8
                                | u32::from(payload[2]) << 16
                                | u32::from(payload[3]) << 24;
                            let fix = payload[4];
                            let flags = payload[5];
                            let fix_stat = payload[6];
                            let flags2 = payload[7];
                            let ttff = u32::from(payload[8])
                                | u32::from(payload[9]) << 8
                                | u32::from(payload[10]) << 16
                                | u32::from(payload[11]) << 24;
                            let msss = u32::from(payload[12])
                                | u32::from(payload[13]) << 8
                                | u32::from(payload[14]) << 16
                                | u32::from(payload[15]) << 24;

                            if fix == 0 {
                                crate::led(dp, crate::LED_COUNTER_PERIOD / 2, 0, 0);
                            } else {
                                crate::led(dp, 0, 0, crate::LED_COUNTER_PERIOD / 2);
                            }

                            iprintln!(
                                stim,
                                "nav status: fix {}, flags {:08b}, time since reset {}",
                                payload[4],
                                flags,
                                msss / 1000
                            );
                        }
                        _ => {
                            iprintln!(stim, "unknown message {:04x}", message_type_id);
                        }
                    }

                    state = ParseState::Idle;
                    continue;
                }

                if pos_in_message == 5 {
                    message_type_id = u16::from(data[message_start + 2]) << 8
                        | u16::from(data[message_start + 3]);
                    message_size = usize::from(data[message_start + 5]) << 8
                        | usize::from(data[message_start + 4]) + 8;
                }

                state = ParseState::Ubx(pos_in_message + 1);
            }
            ParseState::Nmea => {
                if x == '\n' as u8 {
                    let message = &data[message_start..=pos - 2];
                    match core::str::from_utf8(message) {
                        Ok(text) => iprintln!(stim, "NMEA message at {}: {}", message_start, text),
                        Err(e) => iprintln!(
                            stim,
                            "invalid NMEA message at {}: {:02x?} {}",
                            message_start,
                            message,
                            e
                        ),
                    }

                    state = ParseState::Idle;
                } else if x == 0xff {
                    iprintln!(stim, "interrupted NMEA message");
                    state = ParseState::Idle;
                }
            }
        }
    }
}

fn ubx_encode<'a>(
    message_type: UbxMessageType,
    payload: &[u8],
    buffer: &'a mut [u8],
) -> Result<&'a [u8], UbxEncodeError> {
    if payload.len() > 0xffff {
        return Err(UbxEncodeError::PayloadTooBig);
    }

    let ret = buffer.get_mut(0..payload.len() + 8).ok_or(UbxEncodeError::BufferTooSmall)?;

    ret[0] = UBX_SYNC_CHAR_1;
    ret[1] = UBX_SYNC_CHAR_2;
    ret[2] = (message_type as u16 >> 8) as u8;
    ret[3] = message_type as u8;
    ret[4] = payload.len() as u8;
    ret[5] = (payload.len() >> 8) as u8;
    ret[6..payload.len() + 6].copy_from_slice(payload);

    let mut checksum_a: u8 = 0;
    let mut checksum_b: u8 = 0;
    for x in &ret[2..ret.len() - 2] {
        checksum_a = checksum_a.wrapping_add(*x);
        checksum_b = checksum_b.wrapping_add(checksum_a);
    }

    ret[ret.len() - 2] = checksum_a;
    ret[ret.len() - 1] = checksum_b;

    Ok(ret)
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum GnssReceiveState {
    Idle,
    UbxMessage(usize),
    NmeaMessage,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq, FromPrimitive)]
enum UbxMessageType {
    AckAck = 0x0501,
    AckNak = 0x0500,
    CfgMsg = 0x0601,
    CfgNmea = 0x0617,
    CfgPrt = 0x0600,
    CfgRate = 0x0608,
    LogInfo = 0x2108,
    MonMsgpp = 0x0a06,
    NavStatus = 0x0103,
}

pub fn gnss<'a>(dp: &'a pac::Peripherals, spi: &'a mut Spi, delay: &'a mut delay::Delay, stim: &'a mut itm::Stim) {
    iprintln!(stim, "GNSS");

    let mut data = [0; 1024];

    // disable recurrent NMEA messages
    for i in 0x00..=0x05 {
        gnss_transmit(
            spi,
            delay,
            stim,
            UbxMessageType::CfgMsg,
            &[0xf0, i, 0],
            &mut data,
        );
    }

    gnss_transmit(
        spi,
        delay,
        stim,
        UbxMessageType::CfgMsg,
        &[
            (UbxMessageType::NavStatus as u16 >> 8) as u8,
            UbxMessageType::NavStatus as u8,
            1,
        ],
        &mut data,
    );

    // gnss_transmit(spi, delay, stim, UbxMessageType::CfgPrt, &[0x04], &mut data);
    // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::MonMsgpp, &[], &mut ret);
    // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::CfgNmea, &[], &mut ret);
    // let size = gnss_transmit(spi, delay, stim, UbxMessageType::CfgNmea, &[
    //     0x00, // filter (default 0x00)
    //     0x40, // nmeaVersion
    //     0x00, // numSV
    //     0x0a, // flags (default: 0x02)
    //     0, 0, 0, 0, 0, 0, 0,
    //     0x01, // message version
    //     0, 0, 0, 0, 0, 0, 0, 0], &mut data);
    // let size = gnss_transmit(spi, delay, stim, UbxMessageType::CfgMsg, &[
    //     0xf0, // msgClass
    //     0x0a, // msgID
    //     0x00, // rate
    // ], &mut data);
    // let size = gnss_transmit(spi, delay, stim, UbxMessageType::CfgPrt, &[
    //     0x04, // port id
    //     0x00, // reserved
    //     0x00, // tx ready
    //     0x00, // tx ready
    //     0x00, // SPI mode
    //     0x32, // SPI mode (default 0x32)
    //     0x00, // SPI mode
    //     0x00, // SPI mode
    //     0, 0, 0, 0, // reserved
    //     0b00000111, // inProtoMask (default 0b00000111)
    //     0b00000000, // inProtoMask (default 0b00000000)
    //     0b00000011, // outProtoMask (default 0b00000011)
    //     0b00000000, // outProtoMask (default 0b00000000)
    //     0x00, // flags
    //     0x00, // flags
    //     0, 0 // reserved
    // ], &mut data);
    // let size = gnss_transmit(spi, delay, stim, UbxMessageType::CfgPrt, &[0x04], &mut data);
    // let size = gnss_transmit(
    //     spi,
    //     delay,
    //     stim,
    //     UbxMessageType::CfgRate,
    //     &[25, 0, 1, 0, 0, 0],
    //     &mut data,
    // );

    loop {
        let size = gnss_receive(spi, delay, stim, &mut data);
        gnss_parse(stim, dp, &data);
    }
}
