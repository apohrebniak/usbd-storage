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
//! | `ufi` | Include USB Floppy Interface subclass |
//! | `defmt` | Enable logging via [defmt](https://crates.io/crates/defmt) crate |
//!
//! [usb-device]: https://crates.io/crates/usb-device
//! [Vendor Specific subclass]: crate::subclass
//! [Vendor Specific Transport]: crate::transport
//! [Transport]: crate::transport::Transport

// Link the feature-gated modules when present, falling back to the parent module
// otherwise so the crate docs resolve regardless of enabled features.
#![cfg_attr(feature = "scsi", doc = "[SCSI]: crate::subclass::scsi")]
#![cfg_attr(not(feature = "scsi"), doc = "[SCSI]: crate::subclass")]
#![cfg_attr(feature = "ufi", doc = "[UFI]: crate::subclass::ufi")]
#![cfg_attr(not(feature = "ufi"), doc = "[UFI]: crate::subclass")]
#![cfg_attr(feature = "bbb", doc = "[Bulk Only]: crate::transport::bbb")]
#![cfg_attr(not(feature = "bbb"), doc = "[Bulk Only]: crate::transport")]
#![cfg_attr(not(test), no_std)]

#[cfg(feature = "bbb")]
pub(crate) mod buffer;
pub(crate) mod fmt;
pub mod subclass;
pub mod transport;

/// USB Mass Storage Class code
pub const CLASS_MASS_STORAGE: u8 = 0x08;
