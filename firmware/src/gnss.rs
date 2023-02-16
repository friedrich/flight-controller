use cortex_m::{peripheral::itm, iprintln};
use cortex_m::{delay, iprint};
use num_derive::FromPrimitive;
use num_traits::FromPrimitive;
use crate::{Spi, spi};

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
enum UbxEncodeError {
    BufferTooSmall,
    PayloadTooBig,
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

    ret[0] = 0xb5;
    ret[1] = 0x62;
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

fn spi_gnss_transmit(
    spi: &mut Spi,
    stim: &mut itm::Stim,
    delay: &mut delay::Delay,
    message_type: UbxMessageType,
    payload: &[u8],
    ret: &mut [u8],
) -> usize {
    spi.activate_peripheral(spi::Peripheral::Gnss, delay);

    let mut buffer = [0; 2560];
    let data = ubx_encode(message_type, payload, &mut buffer).unwrap();

    // for x in data_send.iter() {
    //     iprint!(stim, "{:02x} ", x);
    // }
    // iprintln!(stim, "");

    let mut size = 0;

    let mut state = GnssReceiveState::Idle;
    let mut message_size = 0;
    let mut message_type_number = 0;

    let mut blub = 0;
    loop {
        // for i in 0..ret.len() {
        if blub == 0 {
            iprintln!(stim, "sending...");
        } else if blub == data.len() {
            iprintln!(stim, "sending complete");
        }

        let y = data.get(blub).cloned().unwrap_or(0xff);
        spi.write(y);
        let x = spi.read();

        // iprintln!(stim, "out={:02x} in={:02x}", y, x);

        let prev_state = state;

        match state {
            GnssReceiveState::Idle => {
                if x == 0xb5 {
                    ret[0] = x;
                    state = GnssReceiveState::UbxMessage(1);
                } else if x == '$' as u8 {
                    state = GnssReceiveState::NmeaMessage;
                    iprintln!(stim, "NMEA Message");
                }
            }
            GnssReceiveState::UbxMessage(i) => {
                // iprintln!(stim, "i={}", i);
                // iprintln!(stim, "{}", i - message_size as usize + 1);
                if i >= ret.len() {
                    iprintln!(stim, "message too long {:?}", &ret[0..i]);
                    state = GnssReceiveState::Idle;
                } else {
                    ret[i] = x;
                    state = GnssReceiveState::UbxMessage(i + 1);
                    if i == 5 {
                        if ret[0] == 0xb5 && ret[1] == 0x62 {
                            message_type_number = u16::from(ret[2]) << 8 | u16::from(ret[3]);
                            message_size = u16::from(ret[5]) << 8 | u16::from(ret[4]) + 8;
                            // iprintln!(stim, "message length: {}", message_size);
                        } else {
                            state = GnssReceiveState::Idle;
                            iprintln!(stim, "invalid message {:?}", &ret[0..i + 1]);
                        }
                    } else if i > 5 && i == message_size as usize - 1 {
                        state = GnssReceiveState::Idle;

                        match UbxMessageType::from_u16(message_type_number) {
                            Some(UbxMessageType::CfgNmea) => {
                                iprintln!(
                                    stim,
                                    "config message: {:?}",
                                    &ret[0..message_size as usize]
                                );
                            }
                            Some(UbxMessageType::AckAck) => {
                                iprintln!(
                                    stim,
                                    "ack message: {:?}",
                                    &ret[0..message_size as usize]
                                );
                            }
                            Some(UbxMessageType::AckNak) => {
                                iprintln!(
                                    stim,
                                    "nack message: {:?}",
                                    &ret[0..message_size as usize]
                                );
                            }
                            Some(UbxMessageType::CfgPrt) => {
                                iprintln!(
                                    stim,
                                    "port config message: {:02x?}",
                                    &ret[0..message_size as usize]
                                );
                            }
                            _ => {
                                iprintln!(stim, "unknown message: {:04x}", message_type_number);
                            }
                        }
                        // ret[2] = (message_type as u16 >> 8) as u8;
                        // ret[3] = message_type as u8;
                    }
                }
            }
            GnssReceiveState::NmeaMessage => {
                if x == 0xff || x == '\n' as u8 {
                    // TODO: should 0xff lead to idle here as well?
                    state = GnssReceiveState::Idle;
                }
            }
        }

        if state != prev_state {
            if let GnssReceiveState::UbxMessage(_) = prev_state {
                // iprintln!(stim, "{:?}", ret);
            } else {
                // iprintln!(stim, "{:?}", state);
            }
        }

        // if i >= ret.len() && x == 0xff {
        //     break;
        // }
        ret.get_mut(blub).and_then(|v| {
            *v = x;
            None::<()>
        });
        if x != 0xff {
            size = blub;
        }

        blub += 1;
        if blub == 100000 {
            blub = 0;
        }
    }

    if let GnssReceiveState::UbxMessage(_) = state {
        iprintln!(stim, "{:?}", ret);
    }

    size
}

pub fn gnss<'a>(spi: &'a mut Spi, delay: &'a mut delay::Delay, stim: &'a mut itm::Stim) {
    iprintln!(stim, "GNSS");

    loop {
        let mut ret = [0; 256];
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
        let size = spi_gnss_transmit(spi, stim, delay, UbxMessageType::CfgPrt, &[0x04], &mut ret);
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

        let ret = &ret[0..size];
        if !ret.is_empty() {
            for x in ret {
                iprint!(stim, "{}", *x as char);
            }
            iprintln!(stim, "\n-----");
        }
    }
}
