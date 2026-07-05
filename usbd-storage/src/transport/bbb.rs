//! Bulk Only Transport (BBB/BOT)

use crate::buffer::Buffer;
use crate::fmt::{debug, trace, warning};
use crate::transport::{CommandStatus, Transport, TransportError};
use core::assert_matches;
use core::borrow::BorrowMut;
use core::cmp::min;
use usb_device::UsbError;
use usb_device::bus::{UsbBus, UsbBusAllocator};
use usb_device::class::ControlIn;
use usb_device::class_prelude::DescriptorWriter;
use usb_device::control::{Recipient, RequestType};
use usb_device::endpoint::{Endpoint, In, Out};

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

struct StatusAndResidue {
    status: CommandStatus,
    data_residue: u32,
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
    cs: Option<StatusAndResidue>,
    max_lun: u8,
    left_to_transfer: u32,
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
            left_to_transfer: 0,
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
    pub fn set_status(&mut self, status: CommandStatus, data_processed: u32) {
        assert_matches!(
            self.state,
            State::DataTransferToHost
                | State::DataTransferFromHost
                | State::DataTransferNoData
                | State::DataTransferToHostStatusAwait
        );
        let data_residue = self.cbw.data_transfer_len.saturating_sub(data_processed);
        trace!("usb: bbb: Set status: {} residue: {}", status, data_residue);
        self.cs = Some(StatusAndResidue {
            status,
            data_residue,
        });
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
            .write(&src[..min(src.len(), self.left_to_transfer as usize)]))
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
        self.left_to_transfer = 0;
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
        if self.left_to_transfer == 0 {
            self.enter_state_data_transfer_to_host_status_await();
            return self.handle_data_transfer_to_host_status_await();
        }

        // is this a short transfer?
        if self.has_status() {
            self.enter_state_data_transfer_to_host_ending();
            return self.handle_data_transfer_to_host_ending();
        }

        // allow a short packet if needed
        let bytes_sent = if self.left_to_transfer < self.packet_size() as u32 {
            self.write_packet()?
        } else {
            self.try_write_full_packet()?
        };

        self.mark_as_transfered(bytes_sent as u32);

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

        let has_to_send = min(self.left_to_transfer as usize, self.packet_size());

        // data has been completely sent off. send status now
        if has_to_send == 0 {
            self.enter_state_status_transfer();
            return self.handle_status_transfer();
        }

        let available = self.buf.available_read();

        if available >= has_to_send {
            // enough data to send a full packet
            let bytes_written = self.write_packet()?;
            self.mark_as_transfered(bytes_written as u32);
        } else {
            // fill up to has_to_send
            let to_fill = has_to_send.saturating_sub(available);
            self.buf.fill_up_to(FILL_BYTE, to_fill);

            let bytes_written = self.write_packet()?;
            self.mark_as_transfered(bytes_written as u32);
        }

        Ok(())
    }

    /// Actively keep reading bytes if there is no status set
    fn handle_data_transfer_from_host(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferFromHost);

        // data is all read. wait for the status
        if self.left_to_transfer == 0 {
            self.enter_state_data_transfer_to_host_status_await();
            return self.handle_data_transfer_to_host_status_await();
        }

        // status is set. keep reading anyway
        if self.has_status() {
            self.enter_state_data_transfer_from_host_ending();
            return self.handle_data_transfer_from_host_ending();
        }

        let bytes_read = self.read_packet()?;
        self.mark_as_transfered(bytes_read as u32);

        Ok(())
    }

    fn handle_data_transfer_from_host_ending(&mut self) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::DataTransferFromHostEnding);

        // keep reading up to the data_residue
        if self.left_to_transfer > 0 {
            self.buf.clean(); // no one is going to read this data anyway

            let bytes_read = self.read_packet()?;
            self.mark_as_transfered(bytes_read as u32);
        } else {
            // all data has been read
            self.enter_state_status_transfer();
            return self.handle_status_transfer();
        }

        Ok(())
    }

    fn build_csw(&mut self) -> Option<[u8; CSW_LEN]> {
        self.cs.as_ref().map(|status| {
            let mut csw = [0u8; CSW_LEN];
            csw[..4].copy_from_slice(CSW_SIGNATURE_LE.as_slice());
            csw[4..8].copy_from_slice(self.cbw.tag.to_le_bytes().as_slice());
            csw[8..12].copy_from_slice(status.data_residue.to_le_bytes().as_slice());
            csw[12..].copy_from_slice(&[status.status as u8]);
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

    fn start_data_transfer(&mut self, cbw: CommandBlockWrapper) -> BulkOnlyTransportResult<()> {
        assert_matches!(self.state, State::CommandTransfer);

        self.cbw = cbw;
        self.left_to_transfer = cbw.data_transfer_len;

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
    fn mark_as_transfered(&mut self, by: u32) {
        self.left_to_transfer = self.left_to_transfer.saturating_sub(by);
    }

    // Tries to read a single packet from OUT EP
    fn read_packet(&mut self) -> BulkOnlyTransportResult<usize> {
        let bytes_read = self.buf.write_all(
            self.packet_size(),
            TransportError::Error(BulkOnlyError::IoBufferOverflow),
            |buf| match self.out_ep.read(buf) {
                Ok(bytes_read) => {
                    trace!("usb: bbb: Read bytes: {}", bytes_read);
                    Ok(bytes_read)
                }
                Err(UsbError::WouldBlock) => {
                    trace!("usb: bbb: Read bytes: WOULD_BLOCK");
                    Err(TransportError::Usb(UsbError::WouldBlock))
                }
                Err(err) => Err(TransportError::Usb(err)),
            },
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
                    }
                    Err(err) => Err(TransportError::Usb(err)),
                }
            } else {
                trace!("usb: bbb: Wrote bytes: 0, buf write available: 0",);
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
            State::DataTransferToHostStatusAwait => {
                self.handle_data_transfer_to_host_status_await()
            }
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
    data_transfer_len: u32,
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
            data_transfer_len,
            direction,
            lun: value[9] & 0b00001111,
            block_len: block_len as usize,
            block,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    use std::sync::{Arc, Mutex};
    use usb_device::UsbDirection;
    use usb_device::bus::{PollResult, UsbBusAllocator};
    use usb_device::class_prelude::{EndpointAddress, EndpointType};
    use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

    // The Thirteen Cases (USB Mass Storage Bulk-Only Transport, section 6.7).

    // 6.7.1 Hn - host expects no data

    #[test]
    fn host_expects_no_data_device_sends_no_data() {
        // Case 1: Hn = Dn
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 0, Host::ExpectsNoData, |b| {
                b.set_status(CommandStatus::Passed, 0)
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 0, "ps={ps}");
        }
    }

    #[test]
    fn host_expects_no_data_device_wants_to_send() {
        // Case 2: Hn < Di -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 0, Host::ExpectsNoData, |b| {
                assert!(is_invalid_state(b.write_data(&[0xAA; 8])));
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
            assert!(data.is_empty(), "ps={ps}");
        }
    }

    #[test]
    fn host_expects_no_data_device_wants_to_receive() {
        // Case 3: Hn < Do -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 0, Host::ExpectsNoData, |b| {
                assert!(is_invalid_state(b.read_data(&mut [0u8; 8])));
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
        }
    }

    // 6.7.2 Hi - host expects to receive data from the device

    #[test]
    fn host_expects_data_in_device_sends_none() {
        // Case 4: Hi > Dn
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 32, Host::ExpectsDataIn, |b| {
                b.set_status(CommandStatus::Passed, 0)
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 32, "ps={ps}");
            assert_eq!(data.len(), 32, "ps={ps}"); // padded with fill bytes
        }
    }

    #[test]
    fn host_expects_more_data_in_than_device_sends() {
        // Case 5: Hi > Di
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 32, Host::ExpectsDataIn, |b| {
                let n = b.write_data(&[0xAA; 16]).unwrap();
                b.set_status(CommandStatus::Passed, n as u32);
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 16, "ps={ps}");
            assert_eq!(data.len(), 32, "ps={ps}");
        }
    }

    #[test]
    fn host_expects_exactly_what_device_sends() {
        // Case 6: Hi = Di
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 32, Host::ExpectsDataIn, |b| {
                let n = b.write_data(&[0xAA; 32]).unwrap();
                b.set_status(CommandStatus::Passed, n as u32);
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 0, "ps={ps}");
            assert_eq!(data, vec![0xAA; 32], "ps={ps}");
        }
    }

    #[test]
    fn host_expects_less_data_in_than_device_sends() {
        // Case 7: Hi < Di -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 16, Host::ExpectsDataIn, |b| {
                let n = b.write_data(&[0xAA; 32]).unwrap(); // capped to dCBWDataTransferLength
                assert_eq!(n, 16); // device wanted to send more than the host allows
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
            assert_eq!(data.len(), 16, "ps={ps}");
        }
    }

    #[test]
    fn host_expects_data_in_device_wants_to_receive() {
        // Case 8: Hi <> Do -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, data) = run(ps, 32, Host::ExpectsDataIn, |b| {
                assert!(is_invalid_state(b.read_data(&mut [0u8; 32])));
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
            assert_eq!(data.len(), 32, "ps={ps}");
        }
    }

    // 6.7.3 Ho - host expects to send data to the device

    #[test]
    fn host_sends_data_device_receives_none() {
        // Case 9: Ho > Dn
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 32, Host::ExpectsDataOut, |b| {
                b.set_status(CommandStatus::Passed, 0)
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 32, "ps={ps}");
        }
    }

    #[test]
    fn host_sends_data_device_wants_to_send() {
        // Case 10: Ho <> Di -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 32, Host::ExpectsDataOut, |b| {
                assert!(is_invalid_state(b.write_data(&[0xAA; 8])));
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
        }
    }

    #[test]
    fn host_sends_more_data_out_than_device_receives() {
        // Case 11: Ho > Do
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 32, Host::ExpectsDataOut, |b| {
                let n = b.read_data(&mut [0u8; 16]).unwrap();
                b.set_status(CommandStatus::Passed, n as u32);
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 16, "ps={ps}");
        }
    }

    #[test]
    fn host_sends_exactly_what_device_receives() {
        // Case 12: Ho = Do
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 32, Host::ExpectsDataOut, |b| {
                let n = b.read_data(&mut [0u8; 32]).unwrap();
                b.set_status(CommandStatus::Passed, n as u32);
            });
            assert_eq!(csw.status, 0, "ps={ps}");
            assert_eq!(csw.residue, 0, "ps={ps}");
        }
    }

    #[test]
    fn host_sends_less_data_out_than_device_receives() {
        // Case 13: Ho < Do -> Phase Error
        for ps in PACKET_SIZES {
            let (csw, _) = run(ps, 16, Host::ExpectsDataOut, |b| {
                let n = b.read_data(&mut [0u8; 32]).unwrap(); // only what the host sent
                assert_eq!(n, 16); // device wanted to receive more than the host sends
                b.set_status(CommandStatus::PhaseError, 0);
            });
            assert_eq!(csw.status, 0x02, "ps={ps}");
        }
    }

    #[test]
    fn should_read_data_into_small_buffer() {
        const BUF_SIZE: usize = 512;
        const N: usize = 123;

        let (_shared, mut bbb) = new_bbb(64);
        bbb.state = State::DataTransferFromHost;
        bbb.buf.write([0xFFu8; BUF_SIZE].as_slice()); // fill the buffer

        assert_eq!(N, bbb.read_data([0u8; N].as_mut_slice()).unwrap());
    }

    // ---- test harness ----

    const PACKET_SIZES: [u16; 4] = [8, 16, 32, 64];

    #[derive(Copy, Clone)]
    enum Host {
        ExpectsNoData,
        ExpectsDataIn,  // device -> host
        ExpectsDataOut, // host -> device
    }

    /// Queues shared between the test and the bus moved into the allocator.
    #[derive(Clone, Default)]
    struct Shared {
        out: Arc<Mutex<VecDeque<Vec<u8>>>>, // host -> device packets
        input: Arc<Mutex<Vec<Vec<u8>>>>,    // device -> host packets
    }

    struct MockBus {
        shared: Shared,
    }

    impl UsbBus for MockBus {
        fn alloc_ep(
            &mut self,
            ep_dir: UsbDirection,
            _ep_addr: Option<EndpointAddress>,
            _ep_type: EndpointType,
            _max_packet_size: u16,
            _interval: u8,
        ) -> usb_device::Result<EndpointAddress> {
            Ok(EndpointAddress::from_parts(1, ep_dir))
        }

        fn enable(&mut self) {}
        fn reset(&self) {}
        fn set_device_address(&self, _addr: u8) {}

        fn write(&self, _ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
            self.shared.input.lock().unwrap().push(buf.to_vec());
            Ok(buf.len())
        }

        fn read(&self, _ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
            match self.shared.out.lock().unwrap().pop_front() {
                Some(pkt) => {
                    buf[..pkt.len()].copy_from_slice(&pkt);
                    Ok(pkt.len())
                }
                None => Err(UsbError::WouldBlock),
            }
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

    type MockBbb = BulkOnly<'static, MockBus, Vec<u8>>;

    fn new_bbb(packet_size: u16) -> (Shared, MockBbb) {
        let shared = Shared::default();
        let bus = MockBus {
            shared: shared.clone(),
        };
        let alloc = Box::leak(Box::new(UsbBusAllocator::new(bus)));
        let bbb = BulkOnly::new(alloc, packet_size, 0, vec![0u8; 512]).unwrap();
        // Endpoint read/write reach the bus through a pointer set by the allocator's
        // (crate-private) freeze(); building a device is the only public way to trigger it.
        core::mem::forget(UsbDeviceBuilder::new(alloc, UsbVidPid(0xabcd, 0xabcd)).build());
        (shared, bbb)
    }

    /// Builds a CBW. `data_len` of 0 yields a no-data command.
    fn cbw(data_len: u32, host: Host) -> Vec<u8> {
        let mut c = vec![0u8; CBW_LEN];
        c[..4].copy_from_slice(&0x43425355u32.to_le_bytes());
        c[4..8].copy_from_slice(&1u32.to_le_bytes()); // tag
        c[8..12].copy_from_slice(&data_len.to_le_bytes());
        c[12] = if matches!(host, Host::ExpectsDataIn) {
            0x80
        } else {
            0x00
        };
        c[14] = 6; // CBWCB length
        c[15] = 0x12; // opcode
        c
    }

    /// Splits `bytes` into `packet_size` OUT packets the device will read.
    fn enqueue(shared: &Shared, bytes: &[u8], packet_size: u16) {
        let mut out = shared.out.lock().unwrap();
        for chunk in bytes.chunks(packet_size as usize) {
            out.push_back(chunk.to_vec());
        }
    }

    struct Csw {
        status: u8,
        residue: u32,
    }

    /// Drives a whole transaction and returns the CSW and the data bytes seen on the wire.
    fn run(
        packet_size: u16,
        data_len: u32,
        host: Host,
        device: impl FnOnce(&mut MockBbb),
    ) -> (Csw, Vec<u8>) {
        let (shared, mut bbb) = new_bbb(packet_size);

        enqueue(&shared, &cbw(data_len, host), packet_size);
        if matches!(host, Host::ExpectsDataOut) {
            enqueue(&shared, &vec![0xAA; data_len as usize], packet_size);
        }

        // Read the CBW (and, for Data-Out, all the OUT data) into the transport. A CBW
        // spans several packets at small MTUs, so this may take multiple polls.
        for _ in 0..64 {
            let _ = bbb.poll();
            let out_drained = !matches!(host, Host::ExpectsDataOut) || bbb.left_to_transfer == 0;
            if bbb.get_command().is_some() && out_drained {
                break;
            }
        }

        device(&mut bbb);

        for _ in 0..64 {
            let _ = bbb.poll();
            if matches!(bbb.state, State::CommandTransfer) {
                break;
            }
        }

        // The stream is data (if any) followed by the 13-byte CSW. Both fragment
        // across packets at small MTUs, so reassemble and split off the trailing CSW.
        let stream = std::mem::take(&mut *shared.input.lock().unwrap()).concat();
        assert!(stream.len() >= CSW_LEN, "no CSW sent");
        let (data, raw) = stream.split_at(stream.len() - CSW_LEN);
        assert!(raw.starts_with(&CSW_SIGNATURE_LE));
        (
            Csw {
                status: raw[12],
                residue: u32::from_le_bytes(raw[8..12].try_into().unwrap()),
            },
            data.to_vec(),
        )
    }

    fn is_invalid_state<T>(r: Result<T, TransportError<BulkOnlyError>>) -> bool {
        matches!(r, Err(TransportError::Error(BulkOnlyError::InvalidState)))
    }
}
