use crate::{spi, Spi};
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

    for x in buffer {
        spi.write(*x);
        *x = spi.read();
    }

    Ok(())
}

enum ParseState {
    Idle,
    Ubx(usize),
    Nmea,
}

fn gnss_parse(stim: &mut itm::Stim, data: &[u8]) {
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
                    iprint!(stim, "UBX message at {}: ", message_start);

                    match UbxMessageType::from_u16(message_type_id) {
                        Some(UbxMessageType::CfgNmea) => {
                            iprintln!(stim, "NMEA config");
                        }
                        Some(UbxMessageType::AckAck) => {
                            iprintln!(stim, "ack");
                        }
                        Some(UbxMessageType::AckNak) => {
                            iprintln!(stim, "nack");
                        }
                        Some(UbxMessageType::CfgPrt) => {
                            iprintln!(stim, "port config");
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
    CfgPrt = 0x0600,
    CfgMsg = 0x0601,
    CfgNmea = 0x0617,
    LogInfo = 0x2108,
    MonMsgpp = 0x0a06,
}

pub fn gnss<'a>(spi: &'a mut Spi, delay: &'a mut delay::Delay, stim: &'a mut itm::Stim) {
    iprintln!(stim, "GNSS");

    loop {
        let mut data = [0; 1024];
        gnss_transmit(spi, delay, stim, UbxMessageType::CfgPrt, &[0x04], &mut data);
        // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::MonMsgpp, &[], &mut ret);
        // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::CfgNmea, &[], &mut ret);
        // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::CfgNmea, &[
        //     0x00, // filter (default 0x00)
        //     0x40, // nmeaVersion
        //     0x00, // numSV
        //     0x02, // flags (default: 0x02)
        //     0, 0, 0, 0, 0, 0, 0,
        //     0x01, // message version
        //     0, 0, 0, 0, 0, 0, 0, 0], &mut ret);
        // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::CfgMsg, &[
        //     0x00, // msgClass
        //     0x00, // msgID
        //     0x00, // rate
        // ], &mut ret);
        // let size = spi_gnss_transmit(spi, stim, delay, UbxMessageType::CfgPrt, &[0x04], &mut ret);
        // let size = spi_gnss_transmit(dp, stim, delay, UbxMessageType::CfgPrt, &[
        //     0x04, // port id
        //     0x00, // reserved
        //     0x00, // tx ready
        //     0x00, // tx ready
        //     0x00, // SPI mode
        //     0x32, // SPI mode (default 0x32)
        //     0x00, // SPI mode
        //     0x00, // SPI mode
        //     0, 0, 0, 0, // reserved
        //     0b00000011, // inProtoMask (default 0b00000111)
        //     0b00000000, // inProtoMask (default 0b00000000)
        //     0b00000011, // outProtoMask (default 0b00000011)
        //     0b00000000, // outProtoMask (default 0b00000000)
        //     0x00, // flags
        //     0x00, // flags
        //     0, 0 // reserved
        // ], &mut ret);

        // let ret = &ret[0..size];
        // if !ret.is_empty() {
        //     for x in ret {
        //         iprint!(stim, "{}", *x as char);
        //     }
        //     iprintln!(stim, "\n-----");
        // }
        gnss_parse(stim, &data);
    }
}
