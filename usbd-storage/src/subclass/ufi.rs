//! USB Floppy Interface

use crate::CLASS_MASS_STORAGE;
use crate::fmt::trace;
use crate::transport::Transport;
use usb_device::bus::InterfaceNumber;
use usb_device::bus::UsbBus;
use usb_device::class::{ControlIn, UsbClass};
use usb_device::descriptor::DescriptorWriter;

#[cfg(feature = "bbb")]
use {
    crate::fmt::debug,
    crate::subclass::Command,
    crate::transport::TransportError,
    crate::transport::bbb::{BulkOnly, BulkOnlyError},
    core::borrow::BorrowMut,
    usb_device::bus::UsbBusAllocator,
};

/// UFI device subclass code
pub const SUBCLASS_UFI: u8 = 0x04; // UFI command set

/* UFI codes */
const FORMAT_UNIT: u8 = 0x04;
const INQUIRY: u8 = 0x12;
const MODE_SELECT: u8 = 0x55;
const MODE_SENSE: u8 = 0x5A;
const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1E;
const READ_10: u8 = 0x28;
const READ_12: u8 = 0xA8;
const READ_CAPACITY: u8 = 0x25;
const READ_FORMAT_CAPACITIES: u8 = 0x23;
const REQUEST_SENSE: u8 = 0x03;
const REZERO_UNIT: u8 = 0x01;
const SEEK_10: u8 = 0x2B;
const SEND_DIAGNOSTIC: u8 = 0x1D;
const START_STOP: u8 = 0x1B;
const TEST_UNIT_READY: u8 = 0x00;
const VERIFY: u8 = 0x2F;
const WRITE_10: u8 = 0x2A;
const WRITE_12: u8 = 0xAA;
const WRITE_AND_VERIFY: u8 = 0x2E;

pub fn lba_to_sector(lba: u32, sec_trk: u8) -> u32 {
    lba % sec_trk as u32 + 1
}

pub fn lba_to_head(lba: u32, sec_trk: u8, head_trk: u8) -> u32 {
    (lba / sec_trk as u32) % head_trk as u32
}

pub fn lba_to_track(lba: u32, sec_trk: u8, head_trk: u8) -> u32 {
    (lba / sec_trk as u32) / head_trk as u32
}

/// UFI command
///
/// Refer to specification
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum UfiCommand {
    Unknown,

    FormatUnit {
        track: u8,
        parameter_list_len: u16,
    },

    Inquiry {
        alloc_len: u8,
    },

    TestUnitReady,

    PreventAllowMediumRemoval {
        prevent: bool,
    },

    ReadCapacity,

    RequestSense {
        alloc_len: u8,
    },

    ModeSense {
        page_control: u8,
        page_code: u8,
        param_list_len: u16,
    },

    ModeSelect {
        parameter_list_len: u16,
    },

    StartStop {
        start: bool,
        eject: bool,
    },

    Read {
        lba: u32,
        len: u32,
    },

    Write {
        lba: u32,
        len: u32,
        verify: bool,
    },

    ReadFormatCapacities {
        alloc_len: u16,
    },

    RezeroUnit,

    Seek {
        lba: u32,
    },

    SendDiagnostic {
        default: bool,
    },

    Verify {
        lba: u32,
        len: u16,
    },
}

#[allow(dead_code)]
fn parse_cb(cb: &[u8]) -> UfiCommand {
    debug_assert!(!cb.is_empty());

    // Every arm states the minimum CB length it needs, so a short command
    // block from the host yields Unknown instead of indexing out of bounds.
    match (cb[0], cb.len()) {
        (REQUEST_SENSE, 5..) => UfiCommand::RequestSense { alloc_len: cb[4] },
        (INQUIRY, 5..) => UfiCommand::Inquiry { alloc_len: cb[4] },
        (TEST_UNIT_READY, _) => UfiCommand::TestUnitReady,
        (READ_CAPACITY, _) => UfiCommand::ReadCapacity,
        (PREVENT_ALLOW_MEDIUM_REMOVAL, 5..) => UfiCommand::PreventAllowMediumRemoval {
            prevent: cb[4] != 0,
        },
        (MODE_SENSE, 9..) => UfiCommand::ModeSense {
            page_control: cb[2] >> 6,
            page_code: cb[2] & 0b00111111,
            param_list_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (READ_10, 9..) => UfiCommand::Read {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u16::from_be_bytes([cb[7], cb[8]]) as u32,
        },
        (READ_12, 10..) => UfiCommand::Read {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u32::from_be_bytes([cb[6], cb[7], cb[8], cb[9]]),
        },
        (WRITE_10, 9..) => UfiCommand::Write {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u16::from_be_bytes([cb[7], cb[8]]) as u32,
            verify: false,
        },
        (WRITE_12, 10..) => UfiCommand::Write {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u32::from_be_bytes([cb[6], cb[7], cb[8], cb[9]]),
            verify: false,
        },
        (WRITE_AND_VERIFY, 9..) => UfiCommand::Write {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u16::from_be_bytes([cb[7], cb[8]]) as u32,
            verify: true,
        },
        (FORMAT_UNIT, 9..) => UfiCommand::FormatUnit {
            track: cb[2],
            parameter_list_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (MODE_SELECT, 9..) => UfiCommand::ModeSelect {
            parameter_list_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (READ_FORMAT_CAPACITIES, 9..) => UfiCommand::ReadFormatCapacities {
            alloc_len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        (REZERO_UNIT, _) => UfiCommand::RezeroUnit,
        (SEEK_10, 6..) => UfiCommand::Seek {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
        },
        (SEND_DIAGNOSTIC, 2..) => UfiCommand::SendDiagnostic {
            default: cb[1] & 1 << 2 > 0,
        },
        (START_STOP, 5..) => UfiCommand::StartStop {
            start: cb[4] & 1 > 0,
            eject: cb[4] == 2,
        },
        (VERIFY, 9..) => UfiCommand::Verify {
            lba: u32::from_be_bytes([cb[2], cb[3], cb[4], cb[5]]),
            len: u16::from_be_bytes([cb[7], cb[8]]),
        },
        _ => UfiCommand::Unknown,
    }
}

/// UFI subclass
pub struct Ufi<T: Transport> {
    interface: InterfaceNumber,
    pub(crate) transport: T,
}

/// UFI subclass implementation with [Bulk Only Transport]
///
/// [Bulk Only Transport]: crate::transport::bbb::BulkOnly
#[cfg(feature = "bbb")]
impl<'alloc, Bus: UsbBus + 'alloc, Buf: BorrowMut<[u8]>> Ufi<BulkOnly<'alloc, Bus, Buf>> {
    /// Creates a UFI over Bulk Only Transport instance
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
        buf: Buf,
    ) -> Result<Self, BulkOnlyError> {
        BulkOnly::new(alloc, packet_size, 0, buf).map(|transport| Self {
            interface: alloc.interface(),
            transport,
        })
    }

    /// Poll current UFI command
    ///
    /// # Arguments
    /// * `callback` - closure, in which the UFI command is processed. If there
    ///   is no current command available or it doesn't require any input, this
    ///   callback is *not* called.
    pub fn poll_command<F>(&mut self, mut callback: F) -> Result<(), TransportError<BulkOnlyError>>
    where
        F: FnMut(
            Command<UfiCommand, Ufi<BulkOnly<'alloc, Bus, Buf>>>,
        ) -> Result<(), TransportError<BulkOnlyError>>,
    {
        trace!("usb: ufi: poll_command");

        if let Some(raw_cb) = self.transport.get_command() {
            // exec callback only if user action required
            if !self.transport.has_status() {
                let lun = raw_cb.lun;
                let kind = parse_cb(raw_cb.bytes);

                debug!("usb: ufi: poll_command: Command: {}", kind);

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

impl<Bus, T> UsbClass<Bus> for Ufi<T>
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
            SUBCLASS_UFI,
            T::PROTO,
            None,
        )?;
        writer.interface(self.interface, CLASS_MASS_STORAGE, SUBCLASS_UFI, T::PROTO)?;

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
            trace!("usb: ufi: poll: {}", err);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every opcode the parser knows about, paired with the minimum CB length
    /// its fields require.
    const OPCODES: [(u8, usize); 20] = [
        (FORMAT_UNIT, 9),
        (INQUIRY, 5),
        (MODE_SELECT, 9),
        (MODE_SENSE, 9),
        (PREVENT_ALLOW_MEDIUM_REMOVAL, 5),
        (READ_10, 9),
        (READ_12, 10),
        (READ_CAPACITY, 1),
        (READ_FORMAT_CAPACITIES, 9),
        (REQUEST_SENSE, 5),
        (REZERO_UNIT, 1),
        (SEEK_10, 6),
        (SEND_DIAGNOSTIC, 2),
        (START_STOP, 5),
        (TEST_UNIT_READY, 1),
        (VERIFY, 9),
        (WRITE_10, 9),
        (WRITE_12, 10),
        (WRITE_AND_VERIFY, 9),
        (0xFF, 1), // not a UFI opcode
    ];

    #[test]
    fn parse_cb_short_command_blocks_do_not_panic() {
        // bCBWCBLength is host-controlled (1..=16). Every length shorter than
        // what an opcode needs must yield Unknown instead of panicking.
        for (opcode, min_len) in OPCODES {
            for len in 1..min_len {
                let mut cb = [0u8; 16];
                cb[0] = opcode;
                assert!(
                    matches!(parse_cb(&cb[..len]), UfiCommand::Unknown),
                    "opcode {opcode:#04x} with CB length {len}"
                );
            }
        }
    }

    #[test]
    fn parse_cb_any_length_does_not_panic() {
        for (opcode, _) in OPCODES {
            for len in 1..=16 {
                let mut cb = [0u8; 16];
                cb[0] = opcode;
                let _ = parse_cb(&cb[..len]);
            }
        }
    }

    #[test]
    fn parse_cb_full_read10_parses() {
        let cb = [READ_10, 0, 0, 0, 0x01, 0x02, 0, 0x00, 0x08, 0, 0, 0];
        match parse_cb(&cb) {
            UfiCommand::Read { lba, len } => {
                assert_eq!(lba, 0x0102);
                assert_eq!(len, 8);
            }
            other => panic!("expected Read, got {other:?}"),
        }
    }

    #[test]
    fn parse_cb_full_write12_parses() {
        let cb = [WRITE_12, 0, 0, 0, 0, 0x10, 0, 0, 0x01, 0x00, 0, 0];
        match parse_cb(&cb) {
            UfiCommand::Write { lba, len, verify } => {
                assert_eq!(lba, 0x10);
                assert_eq!(len, 0x100);
                assert!(!verify);
            }
            other => panic!("expected Write, got {other:?}"),
        }
    }
}
