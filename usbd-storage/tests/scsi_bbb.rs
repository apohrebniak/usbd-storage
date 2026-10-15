mod common;

use crate::common::Step;
use crate::common::bbb::{Cbw, CommandStatus, Csw, DataDirection, DummyUsbBus, setup};
use crate::common::scsi::cmd_into_bytes;
use std::time::Duration;
use usb_device::bus::UsbBusAllocator;
use usb_device::class::UsbClass;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_storage::subclass::Command;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::transport::bbb::BulkOnly;

const TIMEOUT: Duration = Duration::from_secs(1);

/// A malformed CBW must not reach the subclass.
///
/// Spec. 6.6.1 makes an invalid CBW terminal until a Reset Recovery, so there is no
/// command to dispatch. When `get_command` still reported one, the block came from an
/// unpopulated `CommandBlockWrapper`; on the first CBW after power-up that is an empty
/// CDB, and `scsi::parse_cb` indexes `cb[0]` -- an index-out-of-bounds panic in release
/// builds, reachable from a single 31-byte packet on the wire.
#[test]
fn invalid_first_cbw_does_not_reach_the_subclass() {
    /*
    let mut io_buf = [0u8; 1024];
    let dummy_bus = DummyUsbBus::new();
    let usb_bus = UsbBusAllocator::new(dummy_bus.clone());
    let mut scsi = Scsi::new(&usb_bus, 64, 0, io_buf.as_mut_slice()).unwrap();
    let _ = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd)).build();

    let mut bad = Cbw {
        data_transfer_len: 0,
        direction: DataDirection::NotExpected,
        block: vec![0x12],
    }
    .into_bytes();
    bad[0] ^= 0xFF; // corrupt dCBWSignature
    dummy_bus.write_data(bad.as_slice());

    scsi.poll(); // reads the CBW, enters the terminal invalid state

    let mut dispatched = false;
    scsi.poll_command(|_cmd| {
        dispatched = true;
        Ok(())
    })
    .unwrap();

    assert!(!dispatched, "an invalid CBW was dispatched to the subclass");
    */

    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let bad_cbw = Cbw {
                data_transfer_len: 0,
                direction: DataDirection::NotExpected,
                block: vec![0x12],
            };

            let mut bad_cbw_bytes = bad_cbw.into_bytes();
            bad_cbw_bytes[0] ^= 0xFF; // corrupt dCBWSignature

            bus.write_data(&bad_cbw_bytes);
        }),
        Step::DevIo,
        Step::DevAssertNoCommand,
    ]}
}

#[test]
fn should_fail_reading_data_from_host_with_bytes_processed() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::Out,
                block: cmd_into_bytes(ScsiCommand::Write { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
            bus.write_data([0u8; 512].as_slice()); // host has written a block
        }),
        Step::DevIo,
        Step::DevCmdHandle(
            |cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                cmd.fail(512); // processed all data
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 0, // processed all data
                status: CommandStatus::Failed,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

#[test]
fn should_fail_reading_data_from_host_without_bytes_processed() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::Out,
                block: cmd_into_bytes(ScsiCommand::Write { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
            bus.write_data([0u8; 512].as_slice()); // host has written a block
        }),
        Step::DevIo,
        Step::DevCmdHandle(
            |cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                cmd.fail_phase();
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 512, // no data has been processed
                status: CommandStatus::PhaseError,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

#[test]
fn should_pass_reading_data_from_host_with_bytes_processed() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::Out,
                block: cmd_into_bytes(ScsiCommand::Write { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
            bus.write_data([0u8; 512].as_slice()); // host has written a block
        }),
        Step::DevIo,
        Step::DevCmdHandle(
            |cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                cmd.pass(512); // processed all data
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 0, // processed all data
                status: CommandStatus::Passed,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

#[test]
fn should_phase_fail_reading_data_from_host_trying_to_pass_without_bytes_read() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::Out,
                block: cmd_into_bytes(ScsiCommand::Write { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
            bus.write_data([0u8; 512].as_slice()); // host has written a block
        }),
        Step::DevIo,
        Step::DevCmdHandle(
            |cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                cmd.fail_phase();
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 512,
                status: CommandStatus::PhaseError,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

#[test]
fn should_fail_in_the_middle_writing_data_to_host() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::In,
                block: cmd_into_bytes(ScsiCommand::Read { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
        }),
        Step::DevIo,
        Step::DevCmdHandle(
            |mut cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                assert_eq!(256, cmd.write_data([0xFFu8; 256].as_slice()).unwrap());
                cmd.fail(256); // processed some data
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            assert_eq!(512, bus.read_n_bytes(512).len()); // skip all data bytes
            let expected_csw = Csw {
                data_transfer_len: 256, // processed part
                status: CommandStatus::Failed,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

/// Spec. 3.1 and 3.2 address both Bulk-Only class requests to the Mass Storage
/// interface, in `wIndex`. usb-device offers every control request to every class, so
/// on a composite device the requests meant for a sibling interface reach this class
/// too. Answering one steals it from the class it was meant for, and a Mass Storage
/// Reset honoured that way also drops the command in flight.
#[test]
fn class_requests_for_another_interface_are_left_alone() {
    let mut io_buf = [0u8; 1024];
    let bus = DummyUsbBus::new();
    let alloc = UsbBusAllocator::new(bus.clone());
    let mut scsi = Scsi::new(&alloc, 64, 0, io_buf.as_mut_slice()).unwrap();
    let theirs = u16::from(u8::from(alloc.interface()));
    let ours = 0;
    assert_eq!(theirs, 1, "Scsi should hold the first interface");
    let mut device = UsbDeviceBuilder::new(&alloc, UsbVidPid(0xabcd, 0xabcd)).build();

    let get_max_lun = |index| setup(0xA1, 0xFE, 0, index, 1);
    let reset = |index| setup(0x21, 0xFF, 0, index, 0);

    assert_eq!(
        bus.control(&mut device, &mut scsi, get_max_lun(theirs)),
        None,
        "GET MAX LUN for another interface should be refused"
    );
    assert_eq!(
        bus.control(&mut device, &mut scsi, get_max_lun(ours)),
        Some(vec![0]),
        "GET MAX LUN for this interface should be answered"
    );

    bus.write_cbw(Cbw {
        data_transfer_len: 0,
        direction: DataDirection::NotExpected,
        block: cmd_into_bytes(ScsiCommand::TestUnitReady),
    });
    drive(&bus, &mut scsi);
    assert!(has_command(&mut scsi), "the CBW should be in flight");

    assert_eq!(
        bus.control(&mut device, &mut scsi, reset(theirs)),
        None,
        "a Mass Storage Reset for another interface should be refused"
    );
    assert!(
        has_command(&mut scsi),
        "a Mass Storage Reset for another interface should leave the command in flight"
    );

    assert_eq!(
        bus.control(&mut device, &mut scsi, reset(ours)),
        Some(vec![]),
        "a Mass Storage Reset for this interface should be accepted"
    );
    assert!(
        !has_command(&mut scsi),
        "a Mass Storage Reset for this interface should drop the command in flight"
    );
}

/// Drive the device until it has no IO left to do
fn drive(bus: &DummyUsbBus, scsi: &mut Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>) {
    let mut bytes_processed = bus.bytes_processed();
    loop {
        scsi.poll();
        let new = bus.bytes_processed();
        if new == bytes_processed {
            break;
        }
        bytes_processed = new;
    }
}

/// Whether a command waits for the subclass, which leaves it in flight
fn has_command(scsi: &mut Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>) -> bool {
    let mut seen = false;
    scsi.poll_command(|_| {
        seen = true;
        Ok(())
    })
    .unwrap();
    seen
}
