//! Bulk Only Transport (BBB/BOT)

use crate::buffer::Buffer;
use crate::fmt::{trace, warning, debug};
use crate::transport::{CommandStatus, Transport, TransportError};
use core::borrow::BorrowMut;
use core::cmp::min;
use usb_device::UsbError;
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::class::ControlIn;
use usb_device::class_prelude::DescriptorWriter;
use usb_device::control::{Recipient, RequestType};
use usb_device::endpoint::{Endpoint, In, Out};
use core::assert_matches;

/// Bulk Only Transport interface protocol
pub(crate) const TRANSPORT_BBB: u8 = 0x50;

const CLASS_SPECIFIC_BULK_ONLY_MASS_STORAGE_RESET: u8 = 0xFF;
const CLASS_SPECIFIC_GET_MAX_LUN: u8 = 0xFE;

const CBW_SIGNATURE_LE: [u8; 4] = 0x43425355u32.to_le_bytes();
const CSW_SIGNATURE_LE: [u8; 4] = 0x53425355u32.to_le_bytes();

const CBW_LEN: usize = 31;
const CSW_LEN: usize = 13;

const FILL_BYTE: u8 = 0xFF;

struct InvalidCbwError; // Inner transport-specific error

/// Bulk Only Transport error
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BulkOnlyError {
    /// Not enough space to fit additional data
    IoBufferOverflow,
    /// Invalid MAX_LUN value. Refer to USB BBB doc
    InvalidMaxLun,
    /// Transport is not in Data Transfer state
    InvalidState,
    /// Data Transfer expects a full packet to be sent next but not enough data available
    FullPacketExpected,
    /// The IO buffer cannot fit a CBW or a single full packet
    BufferTooSmall,
}

/// Raw Command Block bytes
pub struct CommandBlock<'a> {
    /// CBWCB slice of CBW
    pub bytes: &'a [u8],
    pub lun: u8,
}

#[derive(Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum State {
    /// Device is expecting CBW transfer on OUT EP
    CommandTransfer,
    /// CBW is invalid. This state is terminal
    CommandTransferInvalid,
    /// CBW is valid. No data transfer expected by the host
    DataTransferNoData,
    /// Device actively writes CSW.
    /// Invariant: buffer contains CSW bytes letf to send
    StatusTransfer,
    /// Device actively sends data to host
    DataTransferToHost,
    /// Device has sent all the data and now waits for status from the user
    DataTransferToHostStatusAwait,
    /// User has set the status before all data has been sent
    DataTransferToHostEnding,
    /// Actively reads data from host
    DataTransferFromHost,
    /// User has set the status before all data has been read
    DataTransferFromHostEnding,
}

#[repr(u8)]
#[derive(Default, Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum DataDirection {
    Out,
    In,
    #[default]
    NotExpected,
}

type BulkOnlyTransportResult<T> = Result<T, TransportError<BulkOnlyError>>;

/// Bulk Only Transport
///
/// All data goes through an underlying IO buffer in both directions.
/// During a Data Transfer, data could be read or written via [read_data], [write_data]
/// and [try_write_data_all] methods.
///
/// [read_data]: crate::transport::bbb::BulkOnly::read_data
/// [write_data]: crate::transport::bbb::BulkOnly::write_data
/// [try_write_data_all]: crate::transport::bbb::BulkOnly::try_write_data_all
pub struct BulkOnly<'alloc, Bus: UsbBus, Buf: BorrowMut<[u8]>> {
    in_ep: Endpoint<'alloc, Bus, In>,
    out_ep: Endpoint<'alloc, Bus, Out>,
    buf: Buffer<Buf>,
    state: State,
    cbw: CommandBlockWrapper,
    cs: Option<CommandStatus>,
    max_lun: u8,
    /// A number of "fill" bytes sent during the data transfer.
    /// Used together with `data_residue`
    filled_bytes: u32,
}

impl<'alloc, Bus, Buf> BulkOnly<'alloc, Bus, Buf>
where
    Bus: UsbBus,
    Buf: BorrowMut<[u8]>,
{
    /// Creates Bulk Only Transport instance
    ///
    /// # Arguments
    /// * `alloc` - [UsbBusAllocator]
    /// * `packet_size` - Maximum USB packet size. Allowed values: 8,16,32,64
    /// * `max_lun` - The max index of the Logical Unit
    /// * `buf` - The underlying IO buffer. It is **required** to fit at least a `CBW` and/or a single
    ///   packet. It is **recommended** that buffer fits at least one `LBA` size
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
        max_lun: u8,
        buf: Buf,
    ) -> Result<BulkOnly<'alloc, Bus, Buf>, BulkOnlyError> {
        if max_lun > 0x0F {
            return Err(BulkOnlyError::InvalidMaxLun);
        }

        let buf_len = buf.borrow().len();
        if buf_len < CBW_LEN || buf_len < packet_size as usize {
            return Err(BulkOnlyError::BufferTooSmall);
        }

        Ok(BulkOnly {
            in_ep: alloc.bulk(packet_size),
            out_ep: alloc.bulk(packet_size),
            buf: Buffer::new(buf),
            state: State::CommandTransfer,
            cbw: Default::default(),
            cs: Default::default(),
            max_lun,
            filled_bytes: 0,
        })
    }

    /// Sets a `status` of the current command
    ///
    /// This method doesn't try to send a status immediately. However, all further
    /// writes to the IO buffer won't succeed. The transport will try to send all
    /// contents of the buffer before `CSW`.
    ///
    /// # Panics
    /// Panics if called during any by Data Transfer state. Usually, this means an error in
    /// subclass implementation.
    pub fn set_status(&mut self, status: CommandStatus) {
        assert_matches!(
            self.state,
            State::DataTransferToHost | State::DataTransferFromHost | State::DataTransferNoData | State::DataTransferToHostStatusAwait);
        trace!("usb: bbb: Set status: {}", status);
        self.cs = Some(status);
    }

    /// Returns a Command Block if present
    pub fn get_command(&self) -> Option<CommandBlock<'_>> {
        if matches!(self.state, State::CommandTransfer) {
            return None;
        }

        Some(CommandBlock {
            bytes: &self.cbw.block[..self.cbw.block_len],
            lun: self.cbw.lun,
        })
    }

    /// Reads data from the IO buffer returning the number of bytes actually read
    ///
    /// # Arguments
    /// * `dst` - buffer, to read bytes into
    ///
    /// # Errors
    /// Returns [BulkOnlyError::InvalidState] if called
    /// during any but OUT Data Transfer state.
    ///
    /// [BulkOnlyError::InvalidState]: crate::transport::bbb::BulkOnlyError::InvalidState
    pub fn read_data(&mut self, dst: &mut [u8]) -> BulkOnlyTransportResult<usize> {
        if !matches!(self.state, State::DataTransferFromHost) {
            return Err(TransportError::Error(BulkOnlyError::InvalidState));
        }

        Ok(self
            .buf
            .read(|buf| {
                // fill 'dst' with however much is in 'buf'
                let size = min(dst.len(), buf.len());
                dst[..size].copy_from_slice(&buf[..size]);
                Ok::<usize, core::convert::Infallible>(size)
            })
            .unwrap_or(0))
    }

    /// Writes data from the IO buffer returning the number of bytes actually written
    ///
    /// # Arguments
    /// * `src` - bytes to write
    ///
    /// # Errors
    /// Returns [BulkOnlyError::InvalidState] if called
    /// during any but IN Data Transfer state.
    ///
    /// [BulkOnlyError::InvalidState]: crate::transport::bbb::BulkOnlyError::InvalidState
    pub fn write_data(&mut self, src: &[u8]) -> BulkOnlyTransportResult<usize> {
        if !matches!(self.state, State::DataTransferToHost) {
            return Err(TransportError::Error(BulkOnlyError::InvalidState));
        }

        Ok(self
            .buf
            .write(&src[..min(src.len(), self.data_residue() as usize)]))
    }

    /// Tries to write all data from `src` into the IO buffer returning the number of bytes actually written
    ///
    /// # Errors
    /// * [BulkOnlyError::IoBufferOverflow] - if not enough space is available
    /// * [BulkOnlyError::InvalidState] - if called during any but IN Data Transfer state
    ///
    /// [BulkOnlyError::IoBufferOverflow]: crate::transport::bbb::BulkOnlyError::IoBufferOverflow
    /// [BulkOnlyError::InvalidState]: crate::transport::bbb::BulkOnlyError::InvalidState
    pub fn try_write_data_all(&mut self, src: &[u8]) -> BulkOnlyTransportResult<()> {
        if !matches!(self.state, State::DataTransferToHost) {
            return Err(TransportError::Error(BulkOnlyError::InvalidState));
        }

        self.buf
            .write_all(
                src.len(),
                TransportError::Error(BulkOnlyError::IoBufferOverflow),
                |dst| {
                    dst[..src.len()].copy_from_slice(src);
                    Ok(src.len())
                },
            )
            .map(|bytes_written| debug!("usb: bbb: buf written: {}", bytes_written))
    }

    /// Whether a Command Status has been set
    pub fn has_status(&self) -> bool {
        self.cs.is_some()
    }

    fn enter_state_command_transfer(&mut self) {
        self.cs = None;
        self.filled_bytes = 0;
        self.in_ep.unstall();
        self.out_ep.unstall();

        self.state = State::CommandTransfer;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_cbw_invalid(&mut self) {
        self.state = State::CommandTransferInvalid;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_data_transfer_no_data(&mut self) {
        self.state = State::DataTransferNoData;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_data_transfer_to_host(&mut self) {
        self.state = State::DataTransferToHost;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_data_transfer_to_host_status_await(&mut self) {
        self.state = State::DataTransferToHostStatusAwait;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_data_transfer_to_host_ending(&mut self) {
        self.state = State::DataTransferToHostEnding;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    #[inline]
    fn enter_state_data_transfer_from_host_ending(&mut self) {
        self.state = State::DataTransferFromHostEnding;
        trace!("usb: bbb: enter {:?}", self.state);
    }
    
    #[inline]
    fn enter_state_data_transfer_from_host(&mut self) {
        self.state = State::DataTransferFromHost;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    fn enter_state_status_transfer(&mut self) {
        let Some(csw_bytes) = self.build_csw() else {
            debug!("usb: bbb: UNEXPECTED no CSW");
            panic!("usb: bbb: UNEXPECTED no CSW");
        };


        // this state's invariant
        self.buf.clean();
        self.buf.write(csw_bytes.as_slice());

        self.state = State::StatusTransfer;
        trace!("usb: bbb: enter {:?}", self.state);
    }

    /// The device expects to read CBW bytes
    fn handle_read_cbw(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::CommandTransfer);

        self.read_packet()?;

        if self.buf.available_read() >= CBW_LEN {
            // try parse CBW if enough data available
            match self.try_parse_cbw() {
                Ok(cbw) => {
                    trace!("usb: bbb: Recv CBW: {}", cbw);
                    return self.start_data_transfer(cbw);
                }
                Err(_) => {
                    self.enter_state_cbw_invalid();
                    return self.handle_cbw_invalid();
                }
            }
        }

        Ok(())
    }

    /// The device has read an invalid CBW - wait for reset
    fn handle_cbw_invalid(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::CommandTransferInvalid);

        // Spec. 6.6.1
        self.in_ep.stall();
        self.out_ep.stall();
        
        Ok(())
    }

    fn handle_data_transfer_no_data(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferNoData);

        self.enter_state_data_transfer_to_host_status_await();
        self.handle_data_transfer_to_host_status_await()?;

        Ok(())
    }

    /// Actively try to send more bytes if possible
    fn handle_data_transfer_to_host(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferToHost);

        // data is all sent. wait for the status
        if self.data_residue() == 0 {
            self.enter_state_data_transfer_to_host_status_await();
            return self.handle_data_transfer_to_host_status_await();
        }

        // is this a short transfer?
        if self.has_status() {
            self.enter_state_data_transfer_to_host_ending();
            return self.handle_data_transfer_to_host_ending();
        }

        // allow a short packet if needed
        let bytes_sent = if self.data_residue() < self.packet_size() as u32 {
            self.write_packet()?
        } else {
            self.try_write_full_packet()?
        };

        self.decrease_data_residue_by(bytes_sent as u32);

        Ok(())
    }

    /// The device attempts to send the CSW
    fn handle_status_transfer(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::StatusTransfer);

        // has CSW been sent off?
        if self.buf.available_read() == 0 {
            self.enter_state_command_transfer();
            return self.handle_read_cbw();
        }

        self.write_packet()?;

        Ok(())
    }

    /// Just wait for the user to set the status
    fn handle_data_transfer_to_host_status_await(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferToHostStatusAwait);
        
        if self.has_status() {
            self.enter_state_status_transfer();
            self.handle_status_transfer()?;
        }

        Ok(())
    }

    /// Send off what's left
    fn handle_data_transfer_to_host_ending(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferToHostEnding);

        let left_to_send = self.data_residue().saturating_sub(self.filled_bytes) as usize;
        let has_to_send = min(left_to_send, self.packet_size());

        // data has been completely sent off. send status now
        if has_to_send == 0 {
            self.enter_state_status_transfer();
            return self.handle_status_transfer();
        }

        let available = self.buf.available_read();

        if available >= has_to_send {
            // enough data to send a full packet
            let bytes_written = self.write_packet()?;
            self.decrease_data_residue_by(bytes_written as u32);
        } else {
            // fill up to has_to_send
            let to_fill = has_to_send.saturating_sub(available);
            self.buf.fill_up_to(FILL_BYTE, to_fill);

            self.write_packet()?;
            self.decrease_data_residue_by(available as u32); // actual data

            self.filled_bytes += to_fill as u32;
        }

        Ok(())
    }

    /// Actively keep reading bytes if there is no status set
    fn handle_data_transfer_from_host(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferFromHost);

        // data is all read. wait for the status
        if self.data_residue() == 0 {
            self.enter_state_data_transfer_to_host_status_await();
            return self.handle_data_transfer_to_host_status_await();
        }

        // status is set. keep reading anyway
        if self.has_status() {
            self.enter_state_data_transfer_from_host_ending();
            return self.handle_data_transfer_from_host_ending();
        }

        let bytes_read = self.read_packet()?;
        self.decrease_data_residue_by(bytes_read as u32);

        Ok(())
    }

    fn handle_data_transfer_from_host_ending(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferFromHostEnding);

        // keep reading up to the data_residue
        if self.data_residue() > self.filled_bytes {
            self.buf.clean(); // no one is going to read this data anyway

            let bytes_read = self.read_packet()?;
            self.filled_bytes = self.filled_bytes.saturating_sub(bytes_read as u32);
        } else {
            // all data has been read
            self.enter_state_status_transfer();
            return self.handle_status_transfer();
        }

        Ok(())
    }

    fn build_csw(&mut self) -> Option<[u8; CSW_LEN]> {
        self.cs.map(|status| {
            let mut csw = [0u8; CSW_LEN];
            csw[..4].copy_from_slice(CSW_SIGNATURE_LE.as_slice());
            csw[4..8].copy_from_slice(self.cbw.tag.to_le_bytes().as_slice());
            csw[8..12].copy_from_slice(self.cbw.data_residue.to_le_bytes().as_slice());
            csw[12..].copy_from_slice(&[status as u8]);
            csw
        })
    }

    /// The caller must ensure that there is enough data available
    fn try_parse_cbw(&mut self) -> Result<CommandBlockWrapper, InvalidCbwError> {
        assert!(self.buf.available_read() >= CBW_LEN);

        // read CBW from buf
        let mut raw_cbw = [0u8; CBW_LEN];
        // The closure always returns Ok; unwrap_or(0) is unreachable but avoids unwrap().
        self.buf
            .read::<core::convert::Infallible>(|buf| {
                raw_cbw.copy_from_slice(&buf[..CBW_LEN]); // buf.len() checked in the beginning
                Ok(CBW_LEN)
            })
            .unwrap_or(0);

        // check if CBW is valid. Spec. 6.2.1
        if !raw_cbw.starts_with(&CBW_SIGNATURE_LE) {
            return Err(InvalidCbwError);
        }

        CommandBlockWrapper::from_le_bytes(&raw_cbw[4..]) // parse CBW (skipping signature)
    }

    fn start_data_transfer(&mut self, mut cbw: CommandBlockWrapper) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::CommandTransfer);

        self.cbw = cbw;

        match cbw.direction {
            DataDirection::Out => {
                self.enter_state_data_transfer_from_host();
                self.handle_data_transfer_from_host()
            }
            DataDirection::In => {
                self.enter_state_data_transfer_to_host();
                self.handle_data_transfer_to_host()
            }
            DataDirection::NotExpected => {
                self.enter_state_data_transfer_no_data();
                self.handle_data_transfer_no_data()
            }
        }
    }

    #[inline]
    fn packet_size(&self) -> usize {
        self.in_ep.max_packet_size() as usize // same for both In and Out EPs
    }

    #[inline]
    fn data_residue(&self) -> u32 {
        self.cbw.data_residue
    }

    #[inline]
    fn decrease_data_residue_by(&mut self, by: u32) {
        self.cbw.data_residue = self.cbw.data_residue.saturating_sub(by);
    }

    // Tries to read a single packet from OUT EP
    fn read_packet(&mut self) -> BulkOnlyTransportResult<usize> {
        let bytes_read = self.buf.write_all(
            self.packet_size(),
            TransportError::Error(BulkOnlyError::IoBufferOverflow),
            |buf| {
                match self.out_ep.read(buf) {
                    Ok(bytes_read) =>  {
                        trace!("usb: bbb: Read bytes: {}", bytes_read);
                        Ok(bytes_read)
                    }
                    Err(UsbError::WouldBlock) => {
                        trace!("usb: bbb: Read bytes: WOULD_BLOCK");
                        Err(TransportError::Usb(UsbError::WouldBlock))
                    },
                    Err(err) => Err(TransportError::Usb(err)),
                
                }
            }
        )?;

        Ok(bytes_read)
    }

    /// Tries to write a single packet into IN EP returning the number of bytes actually written
    ///
    /// Might write a short packet if not enough data was in the buffer
    fn write_packet(&mut self) -> BulkOnlyTransportResult<usize> {
        let packet_size = self.packet_size();

        let bytes_written = self.buf.read(|buf| {
            if !buf.is_empty() {
                match self.in_ep.write(&buf[..min(packet_size, buf.len())]) {
                    Ok(bytes_written) => {
                        trace!("usb: bbb: Wrote bytes: {}", bytes_written);
                        Ok(bytes_written)
                    }
                    Err(UsbError::WouldBlock) => {
                        trace!("usb: bbb: Wrote bytes: WOULD_BLOCK");
                        Err(TransportError::Usb(UsbError::WouldBlock))
                    },
                    Err(err) => Err(TransportError::Usb(err)),
                }
            } else {
                trace!(
                    "usb: bbb: Wrote bytes: 0, buf write available: 0",
                );
                Ok(0) // not enough data in buf, though it's not an error
            }
        })?;

        Ok(bytes_written)
    }

    /// Tries to write a FULL packet into IN EP returning the number of bytes actually written
    fn try_write_full_packet(&mut self) -> BulkOnlyTransportResult<usize> {
        if self.buf.available_read() < self.packet_size() {
            return Err(TransportError::Error(BulkOnlyError::FullPacketExpected));
        }

        self.write_packet()
    }

}

impl<Bus, Buf> Transport for BulkOnly<'_, Bus, Buf>
where
    Bus: UsbBus,
    Buf: BorrowMut<[u8]>,
{
    const PROTO: u8 = TRANSPORT_BBB;

    type Bus = Bus;
    type Error = BulkOnlyError;

    fn get_endpoint_descriptors(&self, writer: &mut DescriptorWriter) -> Result<(), UsbError> {
        writer.endpoint(&self.in_ep)?;
        writer.endpoint(&self.out_ep)?;
        Ok(())
    }

    fn reset(&mut self) {
        trace!("usb: bbb: Recv reset");
        self.enter_state_command_transfer();
    }

    fn control_in(&mut self, xfer: ControlIn<Self::Bus>) {
        let req = xfer.request();

        // not interested in this request
        if !(req.request_type == RequestType::Class && req.recipient == Recipient::Interface) {
            return;
        }

        trace!("usb: bbb: Recv ctrl_in: {}", req);

        match req.request {
            // Spec. section 3.1
            CLASS_SPECIFIC_BULK_ONLY_MASS_STORAGE_RESET => {
                self.enter_state_command_transfer();
            }
            // Spec. section 3.2
            CLASS_SPECIFIC_GET_MAX_LUN => {
                // always respond with LUN. A failure here means the host
                // misbehaved or the bus glitched mid control-transfer; log and
                // return rather than panicking the device.
                if let Err(err) = xfer.accept_with(&[self.max_lun]) {
                    warning!("usb: bbb: Get Max Lun accept failed: {}", err);
                }
            }
            _ => {}
        }
    }

    /// Something has happened either on the peripheral or the internal buffer
    fn poll(&mut self) -> BulkOnlyTransportResult<()> {
        trace!("usb: bbb: poll");

        // Spec. 3.3
        // "The host may request Data-In or CSW before sending the associated CBW."
        // Ignore that
        
        match self.state {
            State::CommandTransfer => self.handle_read_cbw(),
            State::CommandTransferInvalid => self.handle_cbw_invalid(),
            State::DataTransferNoData => self.handle_data_transfer_no_data(),
            State::DataTransferToHost => self.handle_data_transfer_to_host(),
            State::DataTransferToHostStatusAwait => self.handle_data_transfer_to_host_status_await(),
            State::DataTransferToHostEnding => self.handle_data_transfer_to_host_ending(),
            State::DataTransferFromHost => self.handle_data_transfer_from_host(),
            State::DataTransferFromHostEnding => self.handle_data_transfer_from_host_ending(),
            State::StatusTransfer => self.handle_status_transfer(),
        }
    }
}

#[derive(Default, Debug, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct CommandBlockWrapper {
    tag: u32,
    // data_transfer_len
    data_residue: u32,
    direction: DataDirection,
    lun: u8,
    block_len: usize,
    block: [u8; 16],
}

impl CommandBlockWrapper {
    fn from_le_bytes(value: &[u8]) -> Result<Self, InvalidCbwError> {
        const MIN_CB_LEN: u8 = 1;
        const MAX_CB_LEN: u8 = 16;

        let block_len = value[10];

        if !(MIN_CB_LEN..=MAX_CB_LEN).contains(&block_len) {
            return Err(InvalidCbwError);
        }

        // These slices are always exactly 4 / 4 / 16 bytes given the CBW layout;
        // map the (unreachable) TryInto failure to InvalidCbwError rather than
        // unwrapping.
        let tag = u32::from_le_bytes(value[..4].try_into().map_err(|_| InvalidCbwError)?);
        let data_transfer_len =
            u32::from_le_bytes(value[4..8].try_into().map_err(|_| InvalidCbwError)?);
        let direction = if data_transfer_len != 0 {
            if (value[8] & (1 << 7)) > 0 {
                DataDirection::In
            } else {
                DataDirection::Out
            }
        } else {
            DataDirection::NotExpected
        };
        let block: [u8; 16] = value[11..].try_into().map_err(|_| InvalidCbwError)?;

        Ok(CommandBlockWrapper {
            tag,
            data_residue: data_transfer_len,
            direction,
            lun: value[9] & 0b00001111,
            block_len: block_len as usize,
            block,
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::transport::bbb::BulkOnly;
    use crate::transport::bbb::State::DataTransferFromHost;
    use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
    use usb_device::class_prelude::{EndpointAddress, EndpointType};
    use usb_device::{UsbDirection, UsbError};

    struct DummyBus;

    impl UsbBus for DummyBus {
        fn alloc_ep(
            &mut self,
            _ep_dir: UsbDirection,
            _ep_addr: Option<EndpointAddress>,
            _ep_type: EndpointType,
            _max_packet_size: u16,
            _interval: u8,
        ) -> usb_device::Result<EndpointAddress> {
            Ok(EndpointAddress::from(0))
        }

        fn enable(&mut self) {}

        fn reset(&self) {}
        fn set_device_address(&self, _addr: u8) {}

        fn write(&self, _ep_addr: EndpointAddress, _buf: &[u8]) -> usb_device::Result<usize> {
            Err(UsbError::InvalidEndpoint)
        }

        fn read(&self, _ep_addr: EndpointAddress, _buf: &mut [u8]) -> usb_device::Result<usize> {
            Err(UsbError::InvalidEndpoint)
        }

        fn set_stalled(&self, _ep_addr: EndpointAddress, _stalled: bool) {}
        fn is_stalled(&self, _ep_addr: EndpointAddress) -> bool {
            false
        }
        fn suspend(&self) {}
        fn resume(&self) {}
        fn poll(&self) -> PollResult {
            PollResult::None
        }
    }

    #[test]
    fn should_read_data_into_small_buffer() {
        const BUF_SIZE: usize = 512;
        const N: usize = 123;

        let alloc = UsbBusAllocator::new(DummyBus);
        let mut bbb = BulkOnly::new(&alloc, 8, 0, vec![0u8; BUF_SIZE]).unwrap();
        bbb.state = DataTransferFromHost;
        bbb.buf.write([0xFFu8; BUF_SIZE].as_slice()); // fill the buffer

        assert_eq!(N, bbb.read_data([0u8; N].as_mut_slice()).unwrap());
    }
}

