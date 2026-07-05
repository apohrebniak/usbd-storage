//! USB SCSI

use crate::CLASS_MASS_STORAGE;
use crate::fmt::{debug, trace};
use crate::transport::Transport;
use crate::transport::TransportError;
use core::fmt::Debug;
use num_enum::TryFromPrimitive;
use usb_device::bus::InterfaceNumber;
use usb_device::bus::UsbBus;
use usb_device::class::{ControlIn, UsbClass};
use usb_device::descriptor::DescriptorWriter;

#[cfg(feature = "bbb")]
use {
    crate::subclass::Command,
    crate::transport::bbb::{BulkOnly, BulkOnlyError},
    core::borrow::BorrowMut,
    usb_device::bus::UsbBusAllocator,
};

/// SCSI device subclass code
pub const SUBCLASS_SCSI: u8 = 0x06; // SCSI Transparent command set

/* SCSI codes */

/* SPC */
const TEST_UNIT_READY: u8 = 0x00;
const REQUEST_SENSE: u8 = 0x03;
const INQUIRY: u8 = 0x12;
const MODE_SENSE_6: u8 = 0x1A;
const MODE_SENSE_10: u8 = 0x5A;

/* SBC */
const READ_10: u8 = 0x28;
#[cfg(feature = "extended_addressing")]
const READ_16: u8 = 0x88;
const READ_CAPACITY_10: u8 = 0x25;
const READ_CAPACITY_16: u8 = 0x9E;
const WRITE_10: u8 = 0x2A;
#[cfg(feature = "extended_addressing")]
const WRITE_16: u8 = 0x8A;

/* MMC */
const READ_FORMAT_CAPACITIES: u8 = 0x23;

/// SCSI command
///
/// Refer to specifications (SPC,SAM,SBC,MMC,etc.)
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum ScsiCommand {
    Unknown {
        cmd: u8,
    },

    /* SPC */
    Inquiry {
        evpd: bool,
        page_code: u8,
        alloc_len: u16,
    },
    TestUnitReady,
    RequestSense {
        desc: bool,
        alloc_len: u8,
    },
    ModeSense6 {
        dbd: bool,
        page_control: PageControl,
        page_code: u8,
        subpage_code: u8,
        alloc_len: u8,
    },
    ModeSense10 {
        dbd: bool,
        page_control: PageControl,
        page_code: u8,
        subpage_code: u8,
        alloc_len: u16,
    },

    /* SBC */
    ReadCapacity10,
    ReadCapacity16 {
        alloc_len: u32,
    },
    Read {
        #[cfg(feature = "extended_addressing")]
        lba: u64,
        #[cfg(feature = "extended_addressing")]
        len: u32,

        #[cfg(not(feature = "extended_addressing"))]
        lba: u32,
        #[cfg(not(feature = "extended_addressing"))]
        len: u16,
    },
    Write {
        #[cfg(feature = "extended_addressing")]
        lba: u64,
        #[cfg(feature = "extended_addressing")]
        len: u32,

        #[cfg(not(feature = "extended_addressing"))]
        lba: u32,
        #[cfg(not(feature = "extended_addressing"))]
        len: u16,
    },

    /* MMC */
    ReadFormatCapacities {
        alloc_len: u16,
    },
}

#[repr(u8)]
#[derive(Copy, Clone, Debug, TryFromPrimitive)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PageControl {
    CurrentValues = 0b00,
    ChangeableValues = 0b01,
    DefaultValues = 0b10,
    SavedValues = 0b11,
}

#[allow(dead_code)]
fn parse_cb(cb: &[u8]) -> ScsiCommand {
    debug_assert!(!cb.is_empty());

    match (cb[0], cb.len()) {
        (TEST_UNIT_READY, _) => ScsiCommand::TestUnitReady,
        (INQUIRY, 5..) => ScsiCommand::Inquiry {
            evpd: (cb[1] & 0b00000001) != 0,
            page_code: cb[2],
            alloc_len: u16::from_be_bytes([cb[3], cb[4]]),
        },
        (REQUEST_SENSE, 5..) => ScsiCommand::RequestSense {
            desc: (cb[1] & 0b00000001) != 0,
            alloc_len: cb[4],
        },
        (READ_CAPACITY_10, _) => ScsiCommand::ReadCapacity10,
        (READ_CAPACITY_16, 14..) => ScsiCommand::ReadCapacity16 {
            alloc_len: u32::from_be_bytes([cb[10], cb[11], cb[12], cb[13]]),
        },
        (READ_10, 9..) => ScsiCommand::Read {
            #[cfg(not(feature = "extended_addressing"))]
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            #[cfg(not(feature = "extended_addressing"))]
            len: u16::from_be_bytes([cb[7], cb[8]]),

            #[cfg(feature = "extended_addressing")]
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]) as u64,
            #[cfg(feature = "extended_addressing")]
            len: u16::from_be_bytes([cb[7], cb[8]]) as u32,
        },
        #[cfg(feature = "extended_addressing")]
        (READ_16, 14..) => ScsiCommand::Read {
            lba: u64::from_be_bytes([cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]]),
            len: u32::from_be_bytes([cb[10], cb[11], cb[12], cb[13]]),
        },
        (WRITE_10, 9..) => ScsiCommand::Write {
            #[cfg(not(feature = "extended_addressing"))]
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            #[cfg(not(feature = "extended_addressing"))]
            len: u16::from_be_bytes([cb[7], cb[8]]),

            #[cfg(feature = "extended_addressing")]
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]) as u64,
            #[cfg(feature = "extended_addressing")]
            len: u16::from_be_bytes([cb[7], cb[8]]) as u32,
        },
        #[cfg(feature = "extended_addressing")]
        (WRITE_16, 14..) => ScsiCommand::Write {
            lba: u64::from_be_bytes([cb[2], cb[3], cb[4], cb[5], cb[6], cb[7], cb[8], cb[9]]),
            len: u32::from_be_bytes([cb[10], cb[11], cb[12], cb[13]]),
        },
        (MODE_SENSE_6, 5..) => ScsiCommand::ModeSense6 {
            dbd: (cb[1] & 0b00001000) != 0,
            page_control: PageControl::try_from_primitive(cb[2] >> 6)
                .unwrap_or(PageControl::CurrentValues),
            page_code: cb[2] & 0b00111111,
            subpage_code: cb[3],
            alloc_len: cb[4],
        },
        (MODE_SENSE_10, 9..) => ScsiCommand::ModeSense10 {
            dbd: (cb[1] & 0b00001000) != 0,
            page_control: PageControl::try_from_primitive(cb[2] >> 6)
                .unwrap_or(PageControl::CurrentValues),
            page_code: cb[2] & 0b00111111,
            subpage_code: cb[3],
            alloc_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (READ_FORMAT_CAPACITIES, 9..) => ScsiCommand::ReadFormatCapacities {
            alloc_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (cmd, _) => ScsiCommand::Unknown { cmd },
    }
}

/// SCSI USB Mass Storage subclass
pub struct Scsi<T: Transport> {
    interface: InterfaceNumber,
    pub(crate) transport: T,
}

/// SCSI subclass implementation with [Bulk Only Transport]
///
/// [Bulk Only Transport]: crate::transport::bbb::BulkOnly
#[cfg(feature = "bbb")]
impl<'alloc, Bus: UsbBus + 'alloc, Buf: BorrowMut<[u8]>> Scsi<BulkOnly<'alloc, Bus, Buf>> {
    /// Creates an SCSI over Bulk Only Transport instance
    ///
    /// # Arguments
    /// * `alloc` - [UsbBusAllocator]
    /// * `packet_size` - Maximum USB packet size. Allowed values: 8,16,32,64
    /// * `max_lun` - The max index of the Logical Unit
    /// * `buf` - The underlying IO buffer. It is **required** to fit at least a `CBW` and/or a single
    ///   packet. It is **recommended** that buffer fits at least one sector
    ///
    /// # Errors
    /// * [InvalidMaxLun]
    /// * [BufferTooSmall]
    ///
    /// # Panics
    /// Panics if endpoint allocation fails.
    ///
    /// [InvalidMaxLun]: crate::transport::bbb::BulkOnlyError::InvalidMaxLun
    /// [BufferTooSmall]: crate::transport::bbb::BulkOnlyError::BufferTooSmall
    /// [UsbBusAllocator]: usb_device::bus::UsbBusAllocator
    pub fn new(
        alloc: &'alloc UsbBusAllocator<Bus>,
        packet_size: u16,
        max_lun: u8,
        buf: Buf,
    ) -> Result<Self, BulkOnlyError> {
        BulkOnly::new(alloc, packet_size, max_lun, buf).map(|transport| Self {
            interface: alloc.interface(),
            transport,
        })
    }

    /// Poll current SCSI command
    ///
    /// # Arguments
    /// * `callback` - closure, in which the SCSI command is processed. If there
    ///   is no current command available or it doesn't require any input, this
    ///   callback is *not* called.
    pub fn poll_command<F>(&mut self, mut callback: F) -> Result<(), TransportError<BulkOnlyError>>
    where
        F: FnMut(
            Command<ScsiCommand, Scsi<BulkOnly<'alloc, Bus, Buf>>>,
        ) -> Result<(), TransportError<BulkOnlyError>>,
    {
        trace!("usb: scsi: poll_command");

        if let Some(raw_cb) = self.transport.get_command() {
            // exec callback only if user action required
            if !self.transport.has_status() {
                let lun = raw_cb.lun;
                let kind = parse_cb(raw_cb.bytes);

                debug!("usb: scsi: poll_command: Command: {}", kind);

                callback(Command {
                    class: self,
                    kind,
                    lun,
                })?;

                self.transport.poll()?;
            }
        }

        Ok(())
    }
}

impl<Bus, T> UsbClass<Bus> for Scsi<T>
where
    Bus: UsbBus,
    T: Transport<Bus = Bus>,
{
    fn get_configuration_descriptors(
        &self,
        writer: &mut DescriptorWriter,
    ) -> usb_device::Result<()> {
        writer.iad(
            self.interface,
            1,
            CLASS_MASS_STORAGE,
            SUBCLASS_SCSI,
            T::PROTO,
            None,
        )?;
        writer.interface(self.interface, CLASS_MASS_STORAGE, SUBCLASS_SCSI, T::PROTO)?;

        self.transport.get_endpoint_descriptors(writer)?;

        Ok(())
    }

    fn reset(&mut self) {
        self.transport.reset()
    }

    fn control_in(&mut self, xfer: ControlIn<Bus>) {
        self.transport.control_in(xfer)
    }

    fn poll(&mut self) {
        if let Err(err) = self.transport.poll() {
            trace!("usb: scsi: poll: {}", err);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_cb_short_inquiry_does_not_panic() {
        // INQUIRY opcode but no operands present.
        assert!(matches!(
            parse_cb(&[INQUIRY]),
            ScsiCommand::Unknown { cmd } if cmd == INQUIRY
        ));
    }

    #[test]
    fn parse_cb_short_read10_does_not_panic() {
        // READ_10 needs bytes up to index 8; give it only the opcode.
        assert!(matches!(
            parse_cb(&[READ_10]),
            ScsiCommand::Unknown { cmd } if cmd == READ_10
        ));
    }

    #[test]
    fn parse_cb_short_read_capacity_16_does_not_panic() {
        // READ_CAPACITY_16 reads cb[10..14]; a 10-byte CDB is too short.
        let cb = [READ_CAPACITY_16, 0, 0, 0, 0, 0, 0, 0, 0, 0];
        assert!(matches!(
            parse_cb(&cb),
            ScsiCommand::Unknown { cmd } if cmd == READ_CAPACITY_16
        ));
    }

    #[cfg(feature = "extended_addressing")]
    #[test]
    fn parse_cb_short_read16_yields_unknown_not_panic() {
        // READ_16 (0x88) with a CDB shorter than 14 bytes used to index past
        // the slice and panic. It must yield the Unknown path instead.
        let cb = [READ_16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // 12 bytes < 14
        assert!(matches!(
            parse_cb(&cb),
            ScsiCommand::Unknown { cmd } if cmd == READ_16
        ));
    }

    #[cfg(feature = "extended_addressing")]
    #[test]
    fn parse_cb_short_write16_yields_unknown_not_panic() {
        let cb = [WRITE_16, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]; // 13 bytes < 14
        assert!(matches!(
            parse_cb(&cb),
            ScsiCommand::Unknown { cmd } if cmd == WRITE_16
        ));
    }

    #[cfg(feature = "extended_addressing")]
    #[test]
    fn parse_cb_full_read16_parses() {
        // A full 16-byte READ_16: opcode + 8-byte LBA + 4-byte length.
        let mut cb = [0u8; 16];
        cb[0] = READ_16;
        cb[9] = 0x01; // LBA low byte
        cb[13] = 0x08; // transfer length
        match parse_cb(&cb) {
            ScsiCommand::Read { lba, len } => {
                assert_eq!(lba, 1);
                assert_eq!(len, 8);
            }
            other => panic!("expected Read, got {other:?}"),
        }
    }

    #[test]
    fn parse_cb_mode_sense_6_page_control_does_not_panic() {
        // page_control = cb[2] >> 6; all four 2-bit values are valid, and a
        // too-short CDB yields Unknown rather than panicking on an unwrap.
        let cb = [MODE_SENSE_6, 0, 0b1100_0000, 0, 0];
        match parse_cb(&cb) {
            ScsiCommand::ModeSense6 { page_control, .. } => {
                assert!(matches!(page_control, PageControl::SavedValues));
            }
            other => panic!("expected ModeSense6, got {other:?}"),
        }
        assert!(matches!(
            parse_cb(&[MODE_SENSE_6]),
            ScsiCommand::Unknown { cmd } if cmd == MODE_SENSE_6
        ));
    }
}
