//! USB Mass Storage implementation for [usb-device]
//!
//! # Subclasses:
//! * [SCSI] - SCSI device
//! * [UFI] - USB Floppy Interface
//! * [Vendor Specific subclass] - implement [Transport] trait
//!
//! # Transports:
//! * [Bulk Only]
//! * [Vendor Specific Transport]
//!
//! # Features
//! | Feature | Description                           |
//! | ------- |---------------------------------------|
//! | `bbb` | Include Bulk Only Transport           |
//! | `scsi` | Include SCSI subclass                 |
//! | `ufi` | Include USB Floppy Interface sublcass |
//! | `defmt` | Enable logging via [defmt](https://crates.io/crates/defmt) crate |
//!
//! [usb-device]: https://crates.io/crates/usb-device
//! [SCSI]: crate::subclass::scsi
//! [UFI]: crate::subclass::ufi
//! [Bulk Only]: crate::transport::bbb
//! [Vendor Specific subclass]: crate::subclass
//! [Vendor Specific Transport]: crate::transport
//! [Transport]: crate::transport::Transport

#![no_std]

#[cfg(feature = "bbb")]
pub(crate) mod buffer;
pub(crate) mod fmt;
pub mod subclass;
pub mod transport;

/// USB Mass Storage Class code
pub const CLASS_MASS_STORAGE: u8 = 0x08;
