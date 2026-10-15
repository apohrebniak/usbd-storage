use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use usb_device::bus::{PollResult, UsbBus};
use usb_device::class::UsbClass;
use usb_device::class_prelude::{EndpointAddress, EndpointType};
use usb_device::device::UsbDevice;
use usb_device::{UsbDirection, UsbError};

const MAX_CB_LEN: u8 = 16;
const CSW_LEN: u8 = 13;

#[derive(Debug, Eq, PartialEq)]
pub enum CommandStatus {
    Passed = 0x00,
    Failed = 0x01,
    PhaseError = 0x02,
}

#[allow(dead_code)]
pub enum DataDirection {
    Out,
    In,
    NotExpected,
}
pub struct Cbw {
    pub(crate) data_transfer_len: u32,
    pub(crate) direction: DataDirection,
    pub(crate) block: Vec<u8>,
}

impl Cbw {
    pub fn into_bytes(self) -> Vec<u8> {
        const CBW_SIGNATURE_LE: [u8; 4] = 0x43425355u32.to_le_bytes();

        assert!((1..=16).contains(&self.block.len()));

        let mut bytes = vec![];
        bytes.extend_from_slice(CBW_SIGNATURE_LE.as_slice()); // signature
        bytes.extend_from_slice([0u8; 4].as_slice()); //tag
        bytes.extend_from_slice(self.data_transfer_len.to_le_bytes().as_slice()); // data transfer len

        let direction = match self.direction {
            DataDirection::In => 1_u8 << 7,
            DataDirection::Out | DataDirection::NotExpected => 0u8,
        };
        bytes.push(direction); // direction
        bytes.push(0); // lun
        bytes.push(self.block.len() as u8); // block size

        let mut block = vec![0u8; MAX_CB_LEN as usize];
        block.as_mut_slice()[..self.block.len()].copy_from_slice(self.block.as_slice());
        bytes.extend_from_slice(block.as_slice()); // block

        bytes
    }
}

#[derive(Debug, Eq, PartialEq)]
pub struct Csw {
    pub(crate) data_transfer_len: u32,
    pub(crate) status: CommandStatus,
}

impl Csw {
    pub fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(CSW_LEN as usize, bytes.len());

        let data_transfer_len = u32::from_le_bytes(bytes[8..12].try_into().unwrap());
        let status = match bytes[12] {
            0x00 => CommandStatus::Passed,
            0x01 => CommandStatus::Failed,
            0x02 => CommandStatus::PhaseError,
            _ => panic!("invalid status code"),
        };

        Self {
            data_transfer_len,
            status,
        }
    }
}

pub struct DummyEp {
    addr: EndpointAddress,
    max_packet_size: u16,
    stalled: bool,
    bytes_written: usize,
    bytes_read: usize,
    packets: VecDeque<Vec<u8>>,
}

impl DummyEp {
    pub fn new(addr: EndpointAddress, max_packet_size: u16) -> Self {
        Self {
            addr,
            max_packet_size,
            stalled: false,
            bytes_written: 0,
            bytes_read: 0,
            packets: VecDeque::new(),
        }
    }

    pub fn write_bytes(&mut self, bytes: &[u8]) {
        for chunk in bytes.chunks(self.max_packet_size as usize) {
            self.packets.push_back(chunk.to_vec());
        }
        self.bytes_written += bytes.len();
    }

    pub fn read_packet(&mut self) -> Option<Vec<u8>> {
        let packet = self.packets.pop_front();
        if let Some(len) = packet.as_ref().map(|p| p.len()) {
            self.bytes_read += len;
        }
        packet
    }
}

#[derive(Eq, PartialEq)]
pub struct BytesProcessed {
    /// (written, read)
    ep_in: (usize, usize),
    /// (written, read)
    ep_out: (usize, usize),
}

#[derive(Clone)]
pub struct DummyUsbBus {
    inner: Arc<Mutex<Inner>>,
}

impl DummyUsbBus {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(Inner::new())),
        }
    }

    /// Write Command Block Wrapper as if it was written by a USB host
    pub fn write_cbw(&self, cbw: Cbw) {
        self.write_data(&cbw.into_bytes());
    }

    /// Read Command Status as if it was read by a USB host
    pub fn read_cs(&self) -> Option<Csw> {
        let mut bytes = vec![];
        while bytes.len() < CSW_LEN as usize {
            let mut packet = self.read_packet()?;
            bytes.append(&mut packet);
        }
        Some(Csw::from_bytes(bytes.as_slice()))
    }

    /// Write some data as if it was written by a USB host during Host to Device data transfer
    pub fn write_data(&self, data: &[u8]) {
        let mut lock = self.inner.lock().unwrap();
        let ep = lock.ep_out.as_mut().unwrap();
        ep.write_bytes(data);
    }

    /// Read a single packet as if it was read by a USB host during Device to Host data transfer
    pub fn read_packet(&self) -> Option<Vec<u8>> {
        let mut lock = self.inner.lock().unwrap();
        let ep = lock.ep_in.as_mut().unwrap();
        ep.read_packet()
    }

    pub fn read_n_bytes(&self, n: usize) -> Vec<u8> {
        let mut lock = self.inner.lock().unwrap();
        let ep = lock.ep_in.as_mut().unwrap();

        assert_eq!(0, n % ep.max_packet_size as usize);

        let mut bytes = vec![];
        while bytes.len() < n {
            match ep.read_packet() {
                None => {
                    break;
                }
                Some(mut packet) => {
                    bytes.append(&mut packet);
                }
            }
        }

        bytes
    }

    /// Run one control transfer the way a USB host would, and return what the device
    /// answered on endpoint 0, or `None` when it refused the request
    pub fn control(
        &self,
        device: &mut UsbDevice<'_, DummyUsbBus>,
        class: &mut dyn UsbClass<DummyUsbBus>,
        setup: [u8; 8],
    ) -> Option<Vec<u8>> {
        {
            let mut lock = self.inner.lock().unwrap();
            lock.ep0 = Ep0::default();
            lock.ep0.setup.push_back(setup);
        }
        for _ in 0..64 {
            device.poll(&mut [&mut *class]);
            let lock = self.inner.lock().unwrap();
            if lock.ep0.setup.is_empty() && !lock.ep0.in_pending {
                break;
            }
        }
        let lock = self.inner.lock().unwrap();
        (!lock.ep0.stalled).then(|| lock.ep0.data_in.clone())
    }

    pub fn bytes_processed(&self) -> BytesProcessed {
        let lock = self.inner.lock().unwrap();
        BytesProcessed {
            ep_in: (lock
                .ep_in
                .as_ref()
                .map(|ep| (ep.bytes_written, ep.bytes_read))
                .unwrap()),
            ep_out: (lock
                .ep_out
                .as_ref()
                .map(|ep| (ep.bytes_written, ep.bytes_read))
                .unwrap()),
        }
    }
}

/// A SETUP packet: bmRequestType, bRequest, wValue, wIndex, wLength
pub fn setup(request_type: u8, request: u8, value: u16, index: u16, length: u16) -> [u8; 8] {
    let mut packet = [0u8; 8];
    packet[0] = request_type;
    packet[1] = request;
    packet[2..4].copy_from_slice(&value.to_le_bytes());
    packet[4..6].copy_from_slice(&index.to_le_bytes());
    packet[6..8].copy_from_slice(&length.to_le_bytes());
    packet
}

/// Endpoint 0, as the host sees it
#[derive(Default)]
struct Ep0 {
    /// SETUP packets the host has queued
    setup: VecDeque<[u8; 8]>,
    /// what the device wrote on endpoint 0 IN
    data_in: Vec<u8>,
    /// a write on endpoint 0 IN waits for its completion to be reported
    in_pending: bool,
    /// the device stalled endpoint 0, i.e. refused the request; sticky until the next
    /// transfer, so that an unstall cannot hide the refusal
    stalled: bool,
}

struct Inner {
    enabled: bool,
    ep0: Ep0,
    ep_in: Option<DummyEp>,
    ep_out: Option<DummyEp>,
}

impl Inner {
    fn new() -> Self {
        Self {
            enabled: false,
            ep0: Ep0::default(),
            ep_in: None,
            ep_out: None,
        }
    }
}

impl UsbBus for DummyUsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        _ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> usb_device::Result<EndpointAddress> {
        assert!(!self.inner.lock().unwrap().enabled);

        const EP_OUT_ADDR: usize = 0xFF;
        const EP_IN_ADDR: usize = 0xEE;
        const EP_CTRL: usize = 0;

        if matches!(ep_type, EndpointType::Control) {
            return Ok(EndpointAddress::from(EP_CTRL as u8));
        }

        let mut lock = self.inner.lock().unwrap();
        let addr = match ep_dir {
            UsbDirection::Out => {
                let addr = EndpointAddress::from(EP_OUT_ADDR as u8);
                lock.ep_out.replace(DummyEp::new(addr, max_packet_size));
                addr
            }
            UsbDirection::In => {
                let addr = EndpointAddress::from(EP_IN_ADDR as u8);
                lock.ep_in.replace(DummyEp::new(addr, max_packet_size));
                addr
            }
        };

        Ok(addr)
    }

    fn enable(&mut self) {
        self.inner.lock().unwrap().enabled = true;
    }

    fn reset(&self) {}

    fn set_device_address(&self, _addr: u8) {}

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        let mut lock = self.inner.lock().unwrap();

        if ep_addr.index() == 0 {
            lock.ep0.data_in.extend_from_slice(buf);
            lock.ep0.in_pending = true;
            return Ok(buf.len());
        }

        let ep = lock.ep_in.as_mut().unwrap();

        if ep.addr != ep_addr {
            return Err(UsbError::InvalidEndpoint);
        }

        if buf.len() > ep.max_packet_size as usize {
            return Err(UsbError::BufferOverflow);
        }

        ep.write_bytes(buf);

        Ok(buf.len())
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        let mut lock = self.inner.lock().unwrap();

        if ep_addr.index() == 0 {
            return match lock.ep0.setup.pop_front() {
                Some(setup) => {
                    buf[..setup.len()].copy_from_slice(&setup);
                    Ok(setup.len())
                }
                None => Err(UsbError::WouldBlock),
            };
        }

        let ep = lock.ep_out.as_mut().unwrap();

        if ep.addr != ep_addr {
            return Err(UsbError::InvalidEndpoint);
        }

        if let Some(n) = ep.packets.front().map(|p| p.len())
            && n > buf.len()
        {
            return Err(UsbError::BufferOverflow);
        }

        match ep.read_packet() {
            Some(packet) => {
                let n = packet.len();
                buf[..n].copy_from_slice(packet.as_slice());
                Ok(n)
            }
            None => Err(UsbError::WouldBlock),
        }
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let mut lock = self.inner.lock().unwrap();

        if ep_addr.index() == 0 {
            return lock.ep0.stalled |= stalled;
        }

        if let Some(ep) = lock.ep_in.as_mut()
            && ep.addr == ep_addr
        {
            return ep.stalled = stalled;
        }

        if let Some(ep) = lock.ep_out.as_mut()
            && ep.addr == ep_addr
        {
            ep.stalled = stalled
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let mut lock = self.inner.lock().unwrap();

        if let Some(ep) = lock.ep_in.as_mut()
            && ep.addr == ep_addr
        {
            return ep.stalled;
        }

        if let Some(ep) = lock.ep_out.as_mut()
            && ep.addr == ep_addr
        {
            return ep.stalled;
        }

        false
    }

    fn suspend(&self) {}

    fn resume(&self) {}

    fn poll(&self) -> PollResult {
        let mut lock = self.inner.lock().unwrap();

        if !lock.ep0.setup.is_empty() {
            return PollResult::Data {
                ep_out: 0,
                ep_in_complete: 0,
                ep_setup: 1,
            };
        }

        if lock.ep0.in_pending {
            lock.ep0.in_pending = false;
            return PollResult::Data {
                ep_out: 0,
                ep_in_complete: 1,
                ep_setup: 0,
            };
        }

        PollResult::None
    }
}
