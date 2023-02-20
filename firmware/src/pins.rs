#[macro_export]
macro_rules! pin_mode_input {
    ($dp:expr, $port:expr, $num:literal, $pull:expr) => {{
        paste::paste! {
            use $crate::pac::[<gpio $port:lower>]::pupdr::PUPDR0_A::*;

            $dp.[<GPIO $port>].pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $dp.[<GPIO $port>].moder.modify(|_, w| w.[<moder $num>]().input());
        }
    }};
}

#[macro_export]
macro_rules! pin_mode_output {
    ($dp:expr, $port:expr, $num:literal, $type:expr, $speed:expr, $state:expr) => {{
        paste::paste! {
            use $crate::pac::[<gpio $port:lower>]::otyper::OT0_A::*;
            use $crate::pac::[<gpio $port:lower>]::ospeedr::OSPEEDR0_A::*;

            $dp.[<GPIO $port>].ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $dp.[<GPIO $port>].otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $dp.[<GPIO $port>].odr.modify(|_, w| w.[<odr $num>]().bit($state));
            $dp.[<GPIO $port>].moder.modify(|_, w| w.[<moder $num>]().output());
            $dp.[<GPIO $port>].pupdr.modify(|_, w| w.[<pupdr $num>]().floating());
        }
    }};
}

#[macro_export]
macro_rules! pin_mode_alternate_l {
    ($dp:expr, $port:expr, $num:literal, $type:expr, $pull:expr, $speed:expr, $alt:expr) => {{
        paste::paste! {
            use $crate::pac::[<gpio $port:lower>]::otyper::OT0_A::*;
            use $crate::pac::[<gpio $port:lower>]::pupdr::PUPDR0_A::*;
            use $crate::pac::[<gpio $port:lower>]::ospeedr::OSPEEDR0_A::*;
            use $crate::pac::[<gpio $port:lower>]::afrl::AFRL0_A::*;

            $dp.[<GPIO $port>].afrl.modify(|_, w| w.[<afrl $num>]().variant($alt));
            $dp.[<GPIO $port>].ospeedr.modify(|_, w| w.[<ospeedr $num>]().variant($speed));
            $dp.[<GPIO $port>].otyper.modify(|_, w| w.[<ot $num>]().variant($type));
            $dp.[<GPIO $port>].moder.modify(|_, w| w.[<moder $num>]().alternate());
            $dp.[<GPIO $port>].pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
        }
    }};
}

#[macro_export]
macro_rules! pin_mode_analog {
    ($dp:expr, $port:expr, $num:literal, $pull:expr) => {{
        paste::paste! {
            use $crate::pac::[<gpio $port:lower>]::pupdr::PUPDR0_A::*;

            $dp.[<GPIO $port>].pupdr.modify(|_, w| w.[<pupdr $num>]().variant($pull));
            $dp.[<GPIO $port>].moder.modify(|_, w| w.[<moder $num>]().analog());
        }
    }};
}
