#[macro_export]
macro_rules! pin_mode_input {
    ($gpio:expr, $num:literal, $pull:expr) => {
        paste::paste! {
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().input());
        }
    };
}

#[macro_export]
macro_rules! pin_mode_output {
    ($gpio:expr, $num:literal, $type:expr, $speed:expr, $state:expr) => {
        paste::paste! {
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.odr.modify(|_, w| w.[<odr $num>]().bit($state));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().output());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().floating());
        }
    };
}

#[macro_export]
macro_rules! pin_mode_alternate_l {
    ($gpio:expr, $num:literal, $type:expr, $pull:expr, $speed:expr, $alt:expr) => {
        paste::paste! {
            $gpio.afrl.modify(|_, w| w.[<afrl $num>]().variant($alt));
            $gpio.ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $gpio.otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().alternate());
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
        }
    };
}

#[macro_export]
macro_rules! pin_mode_analog {
    ($gpio:expr, $num:literal, $pull:expr) => {
        paste::paste! {
            $gpio.pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $gpio.moder.modify(|_, w| w.[<moder $num>]().analog());
        }
    };
}
