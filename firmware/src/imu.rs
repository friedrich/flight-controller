use crate::{led, pac, spi, Spi, LED_COUNTER_PERIOD};
use cortex_m::delay;
use cortex_m::{iprintln, peripheral::itm};

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

pub fn accel_init(spi: &mut Spi, delay: &mut delay::Delay) {
    while accel_read(spi, delay, 0x0f) != 0x6b {
        delay.delay_ms(1);
    }
}

pub fn accel<'a>(
    spi: &mut Spi,
    dp: &'a pac::Peripherals,
    delay: &'a mut delay::Delay,
    stim: &'a mut itm::Stim,
) {
    accel_write(spi, delay, 0x10, 0b10100000); // CTRL1_XL, 6.66 kHz
    accel_write(spi, delay, 0x11, 0b10100000); // CTRL2_G, 6.66 kHz

    let mut v = (0.0, 0.0, 0.0);
    let mut g = (0.0, 0.0, 0.0);

    loop {
        let data = read_accelerometer_data(spi, delay);
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
        iprintln!(stim, "{:10.3} {:10.3} {:10.3}", g.0, g.1, g.2);

        let (a, b, c) = (g.0 * g.0, g.1 * g.1, g.2 * g.2);
        led(
            dp,
            (a * LED_COUNTER_PERIOD as f32) as u32,
            (b * LED_COUNTER_PERIOD as f32) as u32,
            (c * LED_COUNTER_PERIOD as f32) as u32,
        );
    }
}
