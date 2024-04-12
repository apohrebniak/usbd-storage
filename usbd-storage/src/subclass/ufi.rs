//! USB Floppy Interface

use crate::transport::Transport;
use crate::CLASS_MASS_STORAGE;
use usb_device::bus::InterfaceNumber;
use usb_device::bus::UsbBus;
use usb_device::class::{ControlIn, UsbClass};
use usb_device::descriptor::DescriptorWriter;
#[cfg(feature = "bbb")]
use {
    crate::fmt::debug,
    crate::subclass::Command,
    crate::transport::bbb::{BulkOnly, BulkOnlyError},
    crate::transport::TransportError,
    core::borrow::BorrowMut,
    usb_device::bus::UsbBusAllocator,
    usb_device::UsbError,
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
    match cb[0] {
        REQUEST_SENSE => UfiCommand::RequestSense { alloc_len: cb[4] },
        INQUIRY => UfiCommand::Inquiry { alloc_len: cb[4] },
        TEST_UNIT_READY => UfiCommand::TestUnitReady,
        READ_CAPACITY => UfiCommand::ReadCapacity,
        PREVENT_ALLOW_MEDIUM_REMOVAL => UfiCommand::PreventAllowMediumRemoval {
            prevent: cb[4] != 0,
        },
        MODE_SENSE => UfiCommand::ModeSense {
            page_control: cb[2] >> 6,
            page_code: cb[2] & 0b00111111,
            param_list_len: u16::from_be_bytes(cb[7..9].try_into().unwrap()),
        },
        READ_10 => UfiCommand::Read {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()) as u32,
        },
        READ_12 => UfiCommand::Read {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u32::from_be_bytes(cb[6..=9].try_into().unwrap()),
        },
        WRITE_10 => UfiCommand::Write {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()) as u32,
            verify: false,
        },
        WRITE_12 => UfiCommand::Write {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u32::from_be_bytes(cb[6..=9].try_into().unwrap()),
            verify: false,
        },
        WRITE_AND_VERIFY => UfiCommand::Write {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()) as u32,
            verify: true,
        },
        FORMAT_UNIT => UfiCommand::FormatUnit {
            track: cb[2],
            parameter_list_len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()),
        },
        MODE_SELECT => UfiCommand::ModeSelect {
            parameter_list_len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()),
        },
        READ_FORMAT_CAPACITIES => UfiCommand::ReadFormatCapacities {
            alloc_len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()),
        },
        REZERO_UNIT => UfiCommand::RezeroUnit,
        SEEK_10 => UfiCommand::Seek {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
        },
        SEND_DIAGNOSTIC => UfiCommand::SendDiagnostic {
            default: cb[1] & 1 << 2 > 0,
        },
        START_STOP => UfiCommand::StartStop {
            start: cb[4] & 1 > 0,
            eject: cb[4] == 2,
        },
        VERIFY => UfiCommand::Verify {
            lba: u32::from_be_bytes(cb[2..=5].try_into().unwrap()),
            len: u16::from_be_bytes(cb[7..=8].try_into().unwrap()),
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
    /// packet. It is **recommended** that buffer fits at least one sector
    ///
    /// # Errors
    /// * [InvalidMaxLun]
    /// * [BufferTooSmall]
    ///
    /// # Panics
    /// Panics if endpoint allocations fails.
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

    /// Drive subclass in both directions
    ///
    /// The passed closure may or may not be called after each time this function is called.
    /// Moreover, it may me called multiple times, if subclass is unable to proceed further.
    ///
    /// # Arguments
    /// * `callback` - closure, in which the SCSI command is processed
    pub fn poll<F>(&mut self, mut callback: F) -> Result<(), UsbError>
    where
        F: FnMut(Command<UfiCommand, Ufi<BulkOnly<'alloc, Bus, Buf>>>),
    {
        fn map_ignore<T>(res: Result<T, TransportError<BulkOnlyError>>) -> Result<(), UsbError> {
            match res {
                Ok(_)
                | Err(TransportError::Usb(UsbError::WouldBlock))
                | Err(TransportError::Error(_)) => Ok(()),
                Err(TransportError::Usb(err)) => Err(err),
            }
        }
        // drive transport in both directions before user action
        map_ignore(self.transport.read())?;
        map_ignore(self.transport.write())?;

        if let Some(raw_cb) = self.transport.get_command() {
            // exec callback only if user action required
            if !self.transport.has_status() {
                let lun = raw_cb.lun;
                let kind = parse_cb(raw_cb.bytes);

                debug!("usb: scsi: Command: {}", kind);

                loop {
                    let command = Command {
                        class: self,
                        kind,
                        lun,
                    };
                    callback(command);

                    // drive transport in both directions after user action.
                    // call callback if not enough data
                    match self.transport.write() {
                        Err(TransportError::Error(BulkOnlyError::FullPacketExpected)) => {
                            continue;
                        }
                        Ok(_)
                        | Err(TransportError::Error(_))
                        | Err(TransportError::Usb(UsbError::WouldBlock)) => { /* ignore */ }
                        Err(TransportError::Usb(err)) => {
                            return Err(err);
                        }
                    };
                    map_ignore(self.transport.read())?;

                    break;
                }
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
}
