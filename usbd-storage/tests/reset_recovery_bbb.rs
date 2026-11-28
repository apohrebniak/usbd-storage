#[allow(dead_code)]
mod common;

use crate::common::PACKET_SIZE;
use crate::common::bbb::{
    BULK_IN_ADDR, BULK_OUT_ADDR, Cbw, DataDirection, DummyUsbBus, drive, setup,
};
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usbd_storage::subclass::scsi::Scsi;
use usbd_storage::subclass::ufi::Ufi;
use usbd_storage::transport::bbb::BulkOnly;

/// TEST UNIT READY: the one command whose operation code is the same in SCSI and in UFI
fn test_unit_ready() -> Cbw {
    Cbw {
        data_transfer_len: 0,
        direction: DataDirection::NotExpected,
        block: vec![0x00],
    }
}

/// Whether a command waits for the subclass, which leaves it in flight
fn scsi_has_command(scsi: &mut Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>) -> bool {
    let mut seen = false;
    scsi.poll_command(|_| {
        seen = true;
        Ok(())
    })
    .unwrap();
    seen
}

/// Whether a command waits for the subclass, which leaves it in flight
fn ufi_has_command(ufi: &mut Ufi<BulkOnly<DummyUsbBus, &mut [u8]>>) -> bool {
    let mut seen = false;
    ufi.poll_command(|_| {
        seen = true;
        Ok(())
    })
    .unwrap();
    seen
}

/// The Reset Recovery a host performs after an invalid CBW (Spec. 5.3.4): a Bulk-Only
/// Mass Storage Reset, then `CLEAR_FEATURE(ENDPOINT_HALT)` on each bulk endpoint.
///
/// Spec. 3.1 has the device keep its endpoints stalled across the reset, so the two
/// `CLEAR_FEATURE` requests are what releases them. Unstalling in the reset would
/// pass a test that only looks at the next CBW, and would hide that the host has
/// nothing left to clear.
fn assert_reset_recovery_works<C: UsbClass<DummyUsbBus>>(
    bus: &DummyUsbBus,
    device: &mut UsbDevice<'_, DummyUsbBus>,
    class: &mut C,
    has_command: impl Fn(&mut C) -> bool,
) {
    let reset = setup(0x21, 0xFF, 0, 0, 0);
    let clear_halt = |addr: u8| setup(0x02, 0x01, 0, u16::from(addr), 0);
    let stalled = |addr: u8| bus.is_bulk_stalled(addr);

    // Given an invalid CBW, which is terminal until a Reset Recovery (Spec. 6.6.1)
    let mut invalid = test_unit_ready().into_bytes();
    invalid[0] ^= 0xFF; // corrupt dCBWSignature
    bus.write_data(&invalid);
    drive(bus, class);
    assert!(
        !has_command(class),
        "an invalid CBW should not reach the subclass"
    );
    assert!(
        stalled(BULK_IN_ADDR) && stalled(BULK_OUT_ADDR),
        "an invalid CBW should stall both bulk endpoints"
    );

    // When the host resets the Mass Storage interface
    assert_eq!(
        bus.control(device, class, reset),
        Some(vec![]),
        "the Mass Storage Reset should be accepted"
    );
    drive(bus, class);

    // Then both endpoints are still stalled (Spec. 3.1)
    assert!(
        stalled(BULK_IN_ADDR) && stalled(BULK_OUT_ADDR),
        "the Mass Storage Reset should preserve the STALL conditions"
    );

    // And each CLEAR_FEATURE(ENDPOINT_HALT) releases its own endpoint only
    assert_eq!(
        bus.control(device, class, clear_halt(BULK_IN_ADDR)),
        Some(vec![]),
        "CLEAR_FEATURE(ENDPOINT_HALT) on Bulk-In should be accepted"
    );
    assert!(
        !stalled(BULK_IN_ADDR) && stalled(BULK_OUT_ADDR),
        "clearing the halt on Bulk-In should leave Bulk-Out stalled"
    );
    assert_eq!(
        bus.control(device, class, clear_halt(BULK_OUT_ADDR)),
        Some(vec![]),
        "CLEAR_FEATURE(ENDPOINT_HALT) on Bulk-Out should be accepted"
    );
    assert!(
        !stalled(BULK_IN_ADDR) && !stalled(BULK_OUT_ADDR),
        "both bulk endpoints should be released"
    );

    // And the device accepts the next CBW, without stalling again
    bus.write_cbw(test_unit_ready());
    drive(bus, class);
    assert!(
        has_command(class),
        "the next CBW should reach the subclass after the Reset Recovery"
    );
    assert!(
        !stalled(BULK_IN_ADDR) && !stalled(BULK_OUT_ADDR),
        "a valid CBW after the Reset Recovery should not stall the bulk endpoints"
    );
}

#[test]
fn scsi_recovers_from_an_invalid_cbw_through_the_reset_recovery() {
    for packet_size in PACKET_SIZE {
        let mut io_buf = [0u8; 1024];
        let bus = DummyUsbBus::new();
        let alloc = UsbBusAllocator::new(bus.clone());
        let mut scsi = Scsi::new(&alloc, packet_size, 0, io_buf.as_mut_slice()).unwrap();
        let mut device = UsbDeviceBuilder::new(&alloc, UsbVidPid(0xabcd, 0xabcd)).build();

        assert_reset_recovery_works(&bus, &mut device, &mut scsi, scsi_has_command);
    }
}

#[test]
fn ufi_recovers_from_an_invalid_cbw_through_the_reset_recovery() {
    for packet_size in PACKET_SIZE {
        let mut io_buf = [0u8; 1024];
        let bus = DummyUsbBus::new();
        let alloc = UsbBusAllocator::new(bus.clone());
        let mut ufi = Ufi::new(&alloc, packet_size, io_buf.as_mut_slice()).unwrap();
        let mut device = UsbDeviceBuilder::new(&alloc, UsbVidPid(0xabcd, 0xabcd)).build();

        assert_reset_recovery_works(&bus, &mut device, &mut ufi, ufi_has_command);
    }
}
