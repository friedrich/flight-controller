use cortex_m::peripheral::itm;
use cortex_m::{delay, iprint, iprintln};

use crate::pac;
use crate::{pin_mode_alternate_l, pin_mode_analog, HSI16_CLOCK_FREQUENCY};

pub fn init(dp: &pac::Peripherals, stim: &mut itm::Stim, delay: &mut delay::Delay) {
    use pac::opamp::opamp1_csr::PGA_GAIN_A::Gain64;
    use pac::opamp::opamp1_csr::VM_SEL_A::Pga;
    use pac::opamp::opamp1_csr::VP_SEL_A::Vinp1;

    // PWM output

    // enable IO port A and B clock
    dp.RCC.ahb2enr.modify(|_, w| w.gpioaen().enabled());
    dp.RCC.ahb2enr.modify(|_, w| w.gpioben().enabled());

    // enable timer clocks
    dp.RCC.apb2enr.modify(|_, w| w.tim15en().enabled());
    dp.RCC.apb2enr.modify(|_, w| w.tim16en().enabled());
    dp.RCC.apb1enr1.modify(|_, w| w.tim2en().enabled()); // TODO: temporary
    dp.RCC.apb1enr1.modify(|_, w| w.tim4en().enabled()); // TODO: temporary

    // TODO: change to low speed?
    pin_mode_alternate_l!(dp, A, 1, PushPull, Floating, HighSpeed, Af9); // AF9: TIM15_CH1N, AF1: TIM2_CH2
    pin_mode_alternate_l!(dp, B, 6, PushPull, Floating, HighSpeed, Af1); // AF1: TIM16_CH1N, AF2: TIM4_CH1, AF5: TIM8_CH1

    pin_mode_alternate_l!(dp, A, 1, PushPull, Floating, HighSpeed, Af1); // TODO: temporary, AF1: TIM2_CH2
    pin_mode_alternate_l!(dp, B, 6, PushPull, Floating, HighSpeed, Af2); // TODO: temporary, AF2: TIM4_CH1

    const COUNTER_PERIOD: u32 = 1000;
    const FREQUENCY: u32 = 3_000;
    const COUNTER_FREQUENCY: u32 = FREQUENCY * COUNTER_PERIOD;
    const PRESCALER_PERIOD: u16 = (HSI16_CLOCK_FREQUENCY / COUNTER_FREQUENCY) as u16; // TODO: reverse calculation, since the rounding error can be big

    // set prescaler period
    dp.TIM15.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    dp.TIM16.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1));
    dp.TIM2.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1)); // TODO: temporary
    dp.TIM4.psc.modify(|_, w| w.psc().variant(PRESCALER_PERIOD - 1)); // TODO: temporary

    // set counter period
    dp.TIM15.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    dp.TIM16.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1));
    dp.TIM2.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1)); // TODO: temporary
    dp.TIM4.arr.modify(|_, w| w.arr().variant(COUNTER_PERIOD - 1)); // TODO: temporary

    // set duty cycle
    dp.TIM15.ccr1().modify(|_, w| w.ccr().variant(COUNTER_PERIOD / 10));
    dp.TIM16.ccr1().modify(|_, w| w.ccr().variant(COUNTER_PERIOD / 10));
    dp.TIM2.ccr2().modify(|_, w| w.ccr().variant(COUNTER_PERIOD / 10)); // TODO: temporary
    dp.TIM4.ccr1().modify(|_, w| w.ccr().variant(COUNTER_PERIOD / 10)); // TODO: temporary

    // set PWM mode 1
    dp.TIM15.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    dp.TIM16.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110));
    dp.TIM2.ccmr1_output().modify(|_, w| w.oc2m().bits(0b110)); // TODO: temporary
    dp.TIM4.ccmr1_output().modify(|_, w| w.oc1m().bits(0b110)); // TODO: temporary

    // select polarity
    dp.TIM15.ccer.modify(|_, w| w.cc1p().set_bit());
    dp.TIM16.ccer.modify(|_, w| w.cc1p().set_bit());
    dp.TIM2.ccer.modify(|_, w| w.cc2p().clear_bit()); // TODO: temporary
    dp.TIM4.ccer.modify(|_, w| w.cc1p().clear_bit()); // TODO: temporary

    // enable output
    dp.TIM15.ccer.modify(|_, w| w.cc1e().set_bit());
    dp.TIM16.ccer.modify(|_, w| w.cc1e().set_bit());
    dp.TIM2.ccer.modify(|_, w| w.cc2e().set_bit()); // TODO: temporary
    dp.TIM4.ccer.modify(|_, w| w.cc1e().set_bit()); // TODO: temporary

    // enable counter
    dp.TIM15.cr1.modify(|_, w| w.cen().set_bit());
    dp.TIM16.cr1.modify(|_, w| w.cen().set_bit());
    dp.TIM2.cr1.modify(|_, w| w.cen().set_bit()); // TODO: temporary
    dp.TIM4.cr1.modify(|_, w| w.cen().set_bit()); // TODO: temporary

    // op amp

    pin_mode_analog!(dp, A, 3, Floating); // OPAMP1_VINP1

    // enable clock
    dp.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    // TODO: calibrate op amp?
    // dp.OPAMP.opamp1_csr.modify(|_, w| w.opaen().set_bit());
    // dp.OPAMP.opamp1_csr.modify(|_, w| w.usertrim().set_bit());

    // configure op amp
    dp.OPAMP.opamp1_csr.modify(|_, w| w.pga_gain().variant(Gain64));
    dp.OPAMP.opamp1_csr.modify(|_, w| w.vp_sel().variant(Vinp1));
    dp.OPAMP.opamp1_csr.modify(|_, w| w.vm_sel().variant(Pga));
    dp.OPAMP.opamp1_csr.modify(|_, w| w.opaintoen().set_bit()); // ADC1_IN13

    // enable op amp
    dp.OPAMP.opamp1_csr.modify(|_, w| w.opaen().set_bit());

    // ADC

    // enable IO port F clock
    dp.RCC.ahb2enr.modify(|_, w| w.gpiofen().enabled());

    pin_mode_analog!(dp, F, 0, Floating); // ADC1_IN10
    pin_mode_analog!(dp, F, 1, Floating); // ADC2_IN10

    // enable ADC clocks
    dp.RCC.ahb2enr.modify(|_, w| w.adc12en().set_bit());

    // select clock source
    dp.RCC.ccipr.modify(|_, w| w.adc12sel().variant(0b10));

    // start ADC
    dp.ADC1.cr.modify(|_, w| w.deeppwd().clear_bit());
    dp.ADC1.cr.modify(|_, w| w.advregen().set_bit());
    dp.ADC2.cr.modify(|_, w| w.deeppwd().clear_bit());
    dp.ADC2.cr.modify(|_, w| w.advregen().set_bit());
    delay.delay_us(20); // TODO: value?

    // calibrate ADC
    dp.ADC1.cr.modify(|_, w| w.adcaldif().clear_bit()); // single-ended input
    dp.ADC1.cr.modify(|_, w| w.adcal().set_bit());
    dp.ADC2.cr.modify(|_, w| w.adcaldif().clear_bit()); // single-ended input
    dp.ADC2.cr.modify(|_, w| w.adcal().set_bit());
    while dp.ADC1.cr.read().adcal().bit_is_set() {}
    while dp.ADC2.cr.read().adcal().bit_is_set() {}

    // TODO: wait four cycles?

    // To convert a single channel, program a sequence with a length of 1.
    dp.ADC1.sqr1.modify(|_, w| w.sq1().variant(10));
    dp.ADC1.sqr1.modify(|_, w| w.sq1().variant(13)); // TODO: temporary
    dp.ADC2.sqr1.modify(|_, w| w.sq1().variant(10));

    // enable ADC
    dp.ADC1.isr.modify(|_, w| w.adrdy().clear());
    dp.ADC1.cr.modify(|_, w| w.aden().set_bit());
    while dp.ADC1.isr.read().adrdy().bit_is_clear() {}
    dp.ADC2.isr.modify(|_, w| w.adrdy().clear());
    dp.ADC2.cr.modify(|_, w| w.aden().set_bit());
    while dp.ADC2.isr.read().adrdy().bit_is_clear() {}

    loop {
        let mut sum1: u64 = 0;
        let mut sum2: u64 = 0;
        let count = 1000;

        iprint!(stim, "measurement... ");
        for _ in 0..count {
            dp.ADC1.cr.modify(|_, w| w.adstart().set_bit());
            dp.ADC2.cr.modify(|_, w| w.adstart().set_bit());
            while dp.ADC1.cr.read().adstart().bit_is_set() {}
            while dp.ADC2.cr.read().adstart().bit_is_set() {}

            let value1 = dp.ADC1.dr.read().bits();
            let value2 = dp.ADC2.dr.read().bits();

            sum1 += value1 as u64;
            sum2 += value2 as u64;
        }
        iprintln!(
            stim,
            "done: {:6} {:6}",
            100 * sum1 / count,
            100 * sum2 / count
        );
    }
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
