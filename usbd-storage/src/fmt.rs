#![allow(unused_macros)]
#![allow(unused_imports)]

macro_rules! trace {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt-log")]
            ::defmt::trace!($s $(, $x)*);
            #[cfg(not(feature="defmt-log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! info {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt-log")]
            ::defmt::info!($s $(, $x)*);
            #[cfg(not(feature="defmt-log"))]
            let _ = ($( & $x ),*);
        }
    };
}

macro_rules! debug {
    ($s:literal $(, $x:expr)* $(,)?) => {
        {
            #[cfg(feature = "defmt-log")]
            ::defmt::debug!($s $(, $x)*);
            #[cfg(not(feature="defmt-log"))]
            let _ = ($( & $x ),*);
        }
    };
}

pub(crate) use debug;
pub(crate) use info;
pub(crate) use trace;
