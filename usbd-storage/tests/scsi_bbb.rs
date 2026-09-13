mod common;

use crate::common::Step;
use crate::common::bbb::{Cbw, CommandStatus, Csw, DataDirection, DummyUsbBus};
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
