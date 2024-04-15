//! USB Mass Storage transports

use core::fmt::Debug;
use usb_device::bus::UsbBus;
use usb_device::class::ControlIn;
use usb_device::descriptor::DescriptorWriter;
use usb_device::UsbError;

#[cfg(feature = "bbb")]
pub mod bbb;

/// Interface protocol for specific transports
pub const TRANSPORT_VENDOR_SPECIFIC: u8 = 0xFF;

/// USB Mass Storage transport.
///
/// An implementation of this trait can be used as underlying transport for subclasses
/// defined in [subclass] module .
///
/// [subclass]: crate::subclass
pub trait Transport {
    /// Interface protocol code
    const PROTO: u8;
    type Bus: UsbBus;

    /// Registers all required USB **endpoints** using a provided `writer`.
    fn get_endpoint_descriptors(&self, writer: &mut DescriptorWriter) -> Result<(), UsbError>;

    /// Called after a USB reset after the bus reset sequence is complete.
    fn reset(&mut self);

    /// Called when a control request is received with direction DeviceToHost.
    fn control_in(&mut self, xfer: ControlIn<Self::Bus>);
}

/// Generic error type that could be used by [Transport] impls.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransportError<E: Debug> {
    /// USB stack error
    Usb(UsbError),
    /// Transport-specific error
    Error(E),
}

/// The status of a Mass Storage command.
///
/// Refer to the USB-MS doc.
#[repr(u8)]
#[derive(Default, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CommandStatus {
    #[default]
    Passed = 0x00,
    Failed = 0x01,
    PhaseError = 0x02,
}
