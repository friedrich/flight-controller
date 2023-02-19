use crate::pac::{self, gpioa, gpiob};
use crate::{pin_mode_alternate_l, HSI16_CLOCK_FREQUENCY};

pub fn init(dp: &pac::Peripherals) {
    use gpioa::afrl::AFRL0_A::Af9;
    use gpioa::ospeedr::OSPEEDR0_A::HighSpeed as HighSpeedA;
    use gpioa::otyper::OT0_A::PushPull as PushPullA;
    use gpioa::pupdr::PUPDR0_A::Floating as FloatingA;
    use gpiob::afrl::AFRL0_A::Af1;
    use gpiob::ospeedr::OSPEEDR0_A::HighSpeed as HighSpeedB;
    use gpiob::otyper::OT0_A::PushPull as PushPullB;
    use gpiob::pupdr::PUPDR0_A::Floating as FloatingB;

    // enable IO port A and B clock
    dp.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());
    dp.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

    // enable timer clocks
    dp.RCC.apb2enr.modify(|_, w| w.tim15en().enabled());
    dp.RCC.apb2enr.modify(|_, w| w.tim16en().enabled());

    pin_mode_alternate_l!(dp.GPIOA, 1, PushPullA, FloatingA, HighSpeedA, Af9); // AF9: TIM15_CH1N, AF1: TIM2_CH2,
    pin_mode_alternate_l!(dp.GPIOB, 6, PushPullB, FloatingB, HighSpeedB, Af1); // AF1: TIM16_CH1N, AF2: TIM4_CH1, AF5: TIM8_CH1

    const COUNTER_PERIOD: u32 = 100;
    const FREQUENCY: u32 = 3_000;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16; // TODO: reverse calculation, since the rounding error can be big

    // set prescaler period
    dp.TIM15.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    dp.TIM16.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    dp.TIM15.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    dp.TIM16.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));

    // set duty cycle
    dp.TIM15.ccr1().modify(|_, w| w.ccr().variant(0));
    dp.TIM16.ccr1().modify(|_, w| w.ccr().variant(0));

    // set PWM mode 1
    dp.TIM15.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    dp.TIM16.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));

    // select active low polarity
    dp.TIM15.ccer.modify(|_, w| w.cc1p().set_bit());
    dp.TIM16.ccer.modify(|_, w| w.cc1p().set_bit());

    // enable output
    dp.TIM15.ccer.modify(|_, w| w.cc1e().set_bit());
    dp.TIM16.ccer.modify(|_, w| w.cc1e().set_bit());

    // enable counter
    dp.TIM15.cr1.modify(|_, w| w.cen().set_bit());
    dp.TIM16.cr1.modify(|_, w| w.cen().set_bit());
}

pub fn init_servos(p: &pac::Peripherals, left_percent: u8, right_percent: u8, back_percent: u8) {
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
    p.GPIOA.afrl.modify(|_, w| w.afrl2().af1()); // AF1:  TIM2_CH3, AF9: TIM15_CH1
    p.GPIOA.afrh.modify(|_, w| w.afrh11().af10()); // AF10: TIM4_CH1, AF11: TIM1_CH4, AF6: TIM1_CH1N
    p.GPIOA.afrh.modify(|_, w| w.afrh12().af10()); // AF10: TIM4_CH2, AF6: TIM1_CH2N, AF1: TIM16_CH1

    // set alternate mode
    p.GPIOA.moder.modify(|_, w| w.moder2().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder11().alternate());
    p.GPIOA.moder.modify(|_, w| w.moder12().alternate());

    const COUNTER_PERIOD: u32 = 100;
    const FREQUENCY: u32 = 450;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16;

    // set prescaler period
    p.TIM2.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    p.TIM4.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));

    // set counter period
    p.TIM2.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    p.TIM4.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));

    const MIN_DUTY_PERCENT: u32 = 50; // measured: 49%
    const MAX_DUTY_PERCENT: u32 = 90; // measured: 87%

    let left_duty_percent =
        u32::from(left_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let right_duty_percent =
        u32::from(right_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;
    let back_duty_percent =
        u32::from(back_percent) * (MAX_DUTY_PERCENT - MIN_DUTY_PERCENT) / 100 + MIN_DUTY_PERCENT;

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
