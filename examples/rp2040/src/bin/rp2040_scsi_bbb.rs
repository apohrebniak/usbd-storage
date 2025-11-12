#![no_std]
#![no_main]

use core::cell::RefCell;
use core::mem::MaybeUninit;
use critical_section::Mutex;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
use rp2040_hal::pac;
use rp2040_hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usb_device::prelude::*;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::subclass::Command;
use usbd_storage::transport::bbb::{BulkOnly, BulkOnlyError};
use usbd_storage::transport::TransportError;

/// Not necessarily `'static`. May reside in some special memory location
static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; BLOCK_SIZE as usize]> = MaybeUninit::uninit();

#[link_section = ".filesystem"]
#[used]
pub static FILESYSTEM: [u8; (BLOCK_SIZE * BLOCKS) as usize] = [0u8; (BLOCK_SIZE * BLOCKS) as usize];

static WRITE_BUFFER: Mutex<RefCell<[u8; BLOCK_SIZE as usize]>> =
    Mutex::new(RefCell::new([0u8; BLOCK_SIZE as usize]));

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
}));

const BLOCK_SIZE: u32 = 4096;
const BLOCKS: u32 = 256;
const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64
const MAX_LUN: u8 = 0; // max 0x0F

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
}
/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;

#[derive(Default)]
struct State {
    storage_offset: usize,
    sense_key: Option<u8>,
    sense_key_code: Option<u8>,
    sense_qualifier: Option<u8>,
}

impl State {
    fn reset(&mut self) {
        self.storage_offset = 0;
        self.sense_key = None;
        self.sense_key_code = None;
        self.sense_qualifier = None;
    }
}

#[rp2040_hal::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = rp2040_hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = rp2040_hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = rp2040_hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Configure GPIO25 as an output
    let mut led = pins.gpio25.into_push_pull_output();

    defmt::info!("Started...");

    let usb_bus = UsbBusAllocator::new(UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    ));
    let mut scsi = usbd_storage::subclass::scsi::Scsi::new(
        &usb_bus,
        USB_PACKET_SIZE,
        MAX_LUN,
        unsafe {
            #[allow(static_mut_refs)]
            USB_TRANSPORT_BUF.assume_init_mut()
        }
        .as_mut_slice(),
    )
    .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("Foo Bar")
            .product("STM32 USB Flash")
            .serial_number("FOOBAR1234567890ABCDEF")])
        .unwrap()
        .self_powered(false)
        .build();

    loop {
        led.set_high().unwrap();

        if !usb_device.poll(&mut [&mut scsi]) {
            continue;
        }

        // clear state if just configured or reset
        if matches!(usb_device.state(), UsbDeviceState::Default) {
            critical_section::with(|cs| {
                STATE.borrow_ref_mut(cs).reset();
            })
        }

        let _ = scsi.poll(|command| {
            led.set_low().unwrap();
            if let Err(err) = process_command(command) {
                defmt::error!("{}", err);
            }
        });
    }
}

fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<UsbBus, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    defmt::info!("Handling: {}", command.kind);

    match command.kind {
        ScsiCommand::TestUnitReady { .. } => {
            command.pass();
        }
        ScsiCommand::Inquiry { .. } => {
            command.try_write_data_all(&[
                0x00, // periph qualifier, periph device type
                0x80, // Removable
                0x04, // SPC-2 compliance
                0x02, // NormACA, HiSu, Response data format
                0x20, // 36 bytes in total
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                b'U', b'N', b'K', b'N', b'O', b'W', b'N', b' ', // 8-byte T-10 vendor id
                b'R', b'P', b'2', b'0', b'4', b'0', b' ', b'U', b'S', b'B', b' ', b'F', b'l', b'a',
                b's', b'h', // 16-byte product identification
                b'1', b'.', b'2', b'3', // 4-byte product revision
            ])?;
            command.pass();
        }
        ScsiCommand::RequestSense { .. } => critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);

            command.try_write_data_all(&[
                0x70,                         // RESPONSE CODE. Set to 70h for information on current errors
                0x00,                         // obsolete
                state.sense_key.unwrap_or(0), // Bits 3..0: SENSE KEY. Contains information describing the error.
                0x00,
                0x00,
                0x00,
                0x00, // INFORMATION. Device-specific or command-specific information.
                0x00, // ADDITIONAL SENSE LENGTH.
                0x00,
                0x00,
                0x00,
                0x00,                               // COMMAND-SPECIFIC INFORMATION
                state.sense_key_code.unwrap_or(0),  // ASC
                state.sense_qualifier.unwrap_or(0), // ASCQ
                0x00,
                0x00,
                0x00,
                0x00,
            ])?;
            state.reset();
            command.pass();
            Ok(())
        })?,
        ScsiCommand::ReadCapacity10 { .. } => {
            let mut data = [0u8; 8];
            let _ = &mut data[0..4].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadCapacity16 { .. } => {
            let mut data = [0u8; 16];
            let _ = &mut data[0..8].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[8..12].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::ReadFormatCapacities { .. } => {
            let mut data = [0u8; 12];
            let _ = &mut data[0..4].copy_from_slice(&[
                0x00, 0x00, 0x00, 0x08, // capacity list length
            ]);
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCKS)); // number of blocks
            data[8] = 0x01; //unformatted media
            let block_length_be = u32::to_be_bytes(BLOCK_SIZE);
            data[9] = block_length_be[1];
            data[10] = block_length_be[2];
            data[11] = block_length_be[3];

            command.try_write_data_all(&data)?;
            command.pass();
        }
        ScsiCommand::Read { lba, len } => critical_section::with(|cs| {
            let lba = lba as u32;
            let len = len as u32;
            let mut state = STATE.borrow_ref_mut(cs);

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;

                // Uncomment this in order to push data in chunks smaller than a USB packet.
                // let end = min(start + USB_PACKET_SIZE as usize - 1, end);

                defmt::info!("Data transfer >>>>>>>> [{}..{}]", start, end);
                let count = command.write_data(&FILESYSTEM[start..end])?;
                state.storage_offset += count;
            } else {
                command.pass();
                state.storage_offset = 0;
            }

            Ok(())
        })?,
        ScsiCommand::Write { lba, len } => critical_section::with(|cs| {
            let lba = lba as u32;
            let len = len as u32;

            let mut state = STATE.borrow_ref_mut(cs);
            let mut write_buffer = WRITE_BUFFER.borrow_ref_mut(cs);

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                loop {
                    let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                    let block_offset = start % (BLOCK_SIZE as usize);
                    let end = start + ((BLOCK_SIZE as usize) - block_offset);
                    defmt::info!("Data transfer <<<<<<<< [{}..{}]", start, end);
                    let count = command.read_data(&mut write_buffer[block_offset..])?;
                    state.storage_offset += count;

                    if count > 0 && (state.storage_offset % (BLOCK_SIZE as usize)) == 0 {
                        // received a full block
                        defmt::warn!("Writing block {}", start / (BLOCK_SIZE as usize));
                        unsafe {
                            rp2040_flash::flash::flash_range_erase_and_program(
                                ((FILESYSTEM.as_ptr() as u32) & 0xffffff)
                                    + ((start as u32) & !0xfff),
                                write_buffer.as_mut(),
                                false,
                            )
                        };
                    } else {
                        break;
                    }
                }

                if state.storage_offset == (len * BLOCK_SIZE) as usize {
                    command.pass();
                    state.storage_offset = 0;
                }
            } else {
                command.pass();
                state.storage_offset = 0;
            }

            Ok(())
        })?,
        ScsiCommand::ModeSense6 { .. } => {
            command.try_write_data_all(&[
                0x03, // number of bytes that follow
                0x00, // the media type is SBC
                0x00, // not write-protected, no cache-control bytes support
                0x00, // no mode-parameter block descriptors
            ])?;
            command.pass();
        }
        ScsiCommand::ModeSense10 { .. } => {
            command.try_write_data_all(&[0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;
            command.pass();
        }
        ref unknown_scsi_kind => {
            defmt::error!("Unknown SCSI command: {}", unknown_scsi_kind);
            critical_section::with(|cs| {
                let mut state = STATE.borrow_ref_mut(cs);
                state.sense_key.replace(0x05); // illegal request Sense Key
                state.sense_key_code.replace(0x20); // Invalid command operation ASC
                state.sense_qualifier.replace(0x00); // Invalid command operation ASCQ

                command.fail();

                Ok(())
            })?
        }
    }

    Ok(())
}
