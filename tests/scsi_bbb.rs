use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
use usb_device::class_prelude::{EndpointAddress, EndpointType};
use usb_device::device::{StringDescriptors, UsbDeviceBuilder, UsbVidPid};
use usb_device::{LangID, UsbDirection};
use usbd_storage::subclass::Command;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::transport::bbb::{BulkOnly, BulkOnlyError};
use usbd_storage::transport::TransportError;

struct DummyUsbBus;

impl UsbBus for DummyUsbBus {
    fn alloc_ep(&mut self, ep_dir: UsbDirection, ep_addr: Option<EndpointAddress>, ep_type: EndpointType, max_packet_size: u16, interval: u8) -> usb_device::Result<EndpointAddress> {
        Ok(EndpointAddress::from(0))
    }

    fn enable(&mut self) {}

    fn reset(&self) {
        todo!()
    }

    fn set_device_address(&self, addr: u8) {
        todo!()
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        todo!()
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        &buf[..32].copy_from_slice(&[
            0x55, 0x53, 0x42, 0x43, // CBW signature
            0x01, 0x00, 0x00, 0x00, // tag
            0xFF, 0x00, 0x00, 0x00, // data transfer len
            0x00, 0x00, 0x09, 0xFF, // data direction out
            0x00,                   // lun 0
            0x00, 0x00, 0x00, 0x09, // block size
            0x28, 0x00,             // SCSI READ_10
            0x00, 0x00, 0x00, 0x00, // LBA = 0
            0x00, 0x00, 0x01,       // 1 block
            0x00,                   // CBW padding
            0xFF,                   // DATA
        ]);
        Ok(32)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        todo!()
    }

    fn suspend(&self) {
        todo!()
    }

    fn resume(&self) {
        todo!()
    }

    fn poll(&self) -> PollResult {
        todo!()
    }
}

#[test]
fn foo() {
    let mut io_buf = [0u8; 512];
    let dummy_bus = DummyUsbBus;
    let usb_bus = UsbBusAllocator::new(dummy_bus);
    let mut scsi = Scsi::new(&usb_bus, 512, 0, io_buf.as_mut_slice()).unwrap();
    let _ = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd)).build();

    loop {
        let _ = scsi.poll(|command| {
            process_command(command).unwrap()
        });
    }

}

fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    match command.kind {
        ref unknown_scsi_kind => {
            command.fail();
        }
    }

    Ok(())
}
