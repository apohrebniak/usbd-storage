mod common;

use crate::common::bbb::{Cbw, CommandStatus, Csw, DataDirection, DummyUsbBus};
use crate::common::scsi::cmd_into_bytes;
use crate::common::Step;
use std::time::Duration;
use usb_device::bus::UsbBusAllocator;
use usb_device::device::{UsbDeviceBuilder, UsbVidPid};
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::subclass::Command;
use usbd_storage::transport::bbb::BulkOnly;

const TIMEOUT: Duration = Duration::from_secs(1);

#[test]
fn should_fail_reading_data_from_host_with_bytes_read() {
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
                cmd.fail();
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 0, // read all
                status: CommandStatus::Failed,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}

#[test]
fn should_fail_reading_data_from_host_without_bytes_read() {
    run_on_scsi_bbb_bus_timed! { TIMEOUT, [
        Step::HostIo(|bus: &DummyUsbBus| {
            let cbw = Cbw {
                data_transfer_len: 512,
                direction: DataDirection::Out,
                block: cmd_into_bytes(ScsiCommand::Write { lba: 0, len: 1 }),
            };
            bus.write_cbw(cbw);
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
fn should_pass_reading_data_from_host_with_bytes_read() {
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
                cmd.pass();
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            let expected_csw = Csw {
                data_transfer_len: 0, // read all
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
        Step::DevCmdHandle(
            |mut cmd: Command<ScsiCommand, Scsi<BulkOnly<DummyUsbBus, &mut [u8]>>>| {
                assert_eq!(256, cmd.write_data([0xFFu8; 256].as_slice()).unwrap());
                cmd.fail();
            },
        ),
        Step::DevIo,
        Step::HostIo(|bus: &DummyUsbBus| {
            assert_eq!(256, bus.read_n_bytes(256).len()); // skip data bytes
            let expected_csw = Csw {
                data_transfer_len: 256,
                status: CommandStatus::Failed,
            };
            assert_eq!(expected_csw, bus.read_cs().unwrap());
        }),
    ] }
}
