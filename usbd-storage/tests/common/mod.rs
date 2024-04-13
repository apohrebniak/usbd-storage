use std::sync::mpsc::sync_channel;
use std::thread;
use std::time::Duration;
use usbd_storage::subclass::Command;

pub mod bbb;
pub mod scsi;

pub const PACKET_SIZE: [u16; 4] = [8, 16, 32, 64];

pub enum Step<BUS, CMD, CLASS> {
    /// Read/Write data on the Host side
    HostIo(fn(&BUS) -> ()),
    /// Drive Device until no pending IO operations left
    DevIo,
    /// Handle a command on the Device side
    DevCmdHandle(fn(Command<CMD, CLASS>) -> ()),
}

// perhaps not the best way, but it's easier that battling against escaped borrows in closures
#[macro_export]
macro_rules! run_on_scsi_bbb_bus_timed {
    { $timeout:expr, $steps:expr } => {
            use common;

            common::timeout($timeout, || {
            for packet_size in common::PACKET_SIZE {
                let steps = $steps;

                let mut io_buf = [0u8; 1024];
                let dummy_bus = DummyUsbBus::new();
                let usb_bus = UsbBusAllocator::new(dummy_bus.clone());
                let mut scsi = Scsi::new(&usb_bus, packet_size, 0, io_buf.as_mut_slice()).unwrap();
                let _ = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd)).build();

                for step in &steps {
                    match step {
                        Step::DevIo => {
                            let mut bytes_processed = dummy_bus.bytes_processed();
                            loop {
                                scsi.poll(|_| {}).unwrap();
                                let new = dummy_bus.bytes_processed();
                                if new == bytes_processed {
                                    break;
                                } else {
                                    bytes_processed = new;
                                }
                            }
                        }
                        Step::HostIo(func) => {
                            func(&dummy_bus);
                        }
                        Step::DevCmdHandle(func) => {
                            let mut command_processed = false;
                            loop {
                                scsi.poll(|command| {
                                    func(command);
                                    command_processed = true;
                                })
                                    .unwrap();

                                if command_processed {
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        });
    };
}

pub fn timeout<F, T>(timeout: Duration, f: F)
where
    F: FnOnce() -> T,
    F: Send + 'static,
    T: Send + 'static,
{
    let (tx, rx) = sync_channel(0);
    thread::spawn(move || {
        f();
        tx.send(()).unwrap();
    });
    rx.recv_timeout(timeout).expect("timeout");
}
