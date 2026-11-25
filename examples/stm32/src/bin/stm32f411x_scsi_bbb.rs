#![no_std]
#![no_main]

use core::cell::RefCell;
use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;
use critical_section::Mutex;
use defmt_rtt as _;
use stm32f4xx_hal::gpio::alt::otg_fs::{Dm, Dp};

use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::otg_fs::{USB, UsbBus};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::RccExt;
use usb_device::prelude::*;
use usbd_storage::subclass::Command;
use usbd_storage::subclass::scsi::{Scsi, ScsiCommand};
use usbd_storage::transport::TransportError;
use usbd_storage::transport::bbb::{BulkOnly, BulkOnlyError};

static mut USB_EP_MEMORY: [u32; 1024] = [0u32; 1024];
/// Not necessarily `'static`. May reside in some special memory location
static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; BLOCK_SIZE as usize]> = MaybeUninit::uninit();

static FAT: &[u8; (BLOCK_SIZE * BLOCKS) as usize] = include_bytes!("../../../empty_fat12.img");

static STORAGE: Mutex<RefCell<[u8; (BLOCK_SIZE * BLOCKS) as usize]>> =
    Mutex::new(RefCell::new([0u8; (BLOCK_SIZE * BLOCKS) as usize]));

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
}));

const BLOCK_SIZE: u32 = 512;
const BLOCKS: u32 = 200;
const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64
const MAX_LUN: u8 = 0; // max 0x0F

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    defmt::error!("{}", defmt::Display2Format(info));
    loop {}
}

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

#[cortex_m_rt::entry]
fn main() -> ! {
    defmt::info!("Started...");

    // take core peripherals
    let cp = cortex_m::Peripherals::take().unwrap();
    // take device-specific peripherals
    let dp = pac::Peripherals::take().unwrap();

    // setup clocks
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(25.MHz()) // 25Mhz HSE is present on the board
        .sysclk(48.MHz())
        .require_pll48clk()
        .freeze();

    // setup GPIO
    let gpioa = dp.GPIOA.split();
    let gpioc = dp.GPIOC.split();
    // USB
    let mut pin_usb_dm = gpioa.pa11.into_push_pull_output();
    let mut pin_usb_dp = gpioa.pa12.into_push_pull_output();
    // indicator LED
    let mut led = gpioc.pc13.into_push_pull_output();

    // force D+ for 100ms
    // this forces the host to enumerate devices
    pin_usb_dm.set_low();
    pin_usb_dp.set_low();
    cp.SYST.delay(&clocks).delay_ms(100u32);

    let usb_peripheral = USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: Dm::from(pin_usb_dm.into_alternate()),
        pin_dp: Dp::from(pin_usb_dp.into_alternate()),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb_peripheral, unsafe { &mut *addr_of_mut!(USB_EP_MEMORY) });
    let mut scsi =
        usbd_storage::subclass::scsi::Scsi::new(&usb_bus, USB_PACKET_SIZE, MAX_LUN, unsafe {
            #[allow(static_mut_refs)]
            USB_TRANSPORT_BUF.assume_init_mut().as_mut_slice()
        })
        .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("Foo Bar")
            .product("STM32 USB Flash")
            .serial_number("FOOBAR1234567890ABCDEF")])
        .unwrap()
        .self_powered(false)
        .build();

    critical_section::with(|cs| {
        STORAGE.borrow_ref_mut(cs).copy_from_slice(FAT);
    });

    loop {
        led.set_high();

        if !usb_device.poll(&mut [&mut scsi]) {
            continue;
        }

        // clear state if just configured or reset
        if matches!(usb_device.state(), UsbDeviceState::Default) {
            critical_section::with(|cs| {
                STATE.borrow_ref_mut(cs).reset();
            })
        }

        if let Err(err) = scsi.poll_command(|command| {
            led.set_low();
            process_command(command)
        }) {
            defmt::warn!("{}", err);
        }
    }
}

/// Truncates a response to the allocation length requested by the host.
fn truncate_to_alloc_len(data: &[u8], alloc_len: usize) -> &[u8] {
    &data[..core::cmp::min(data.len(), alloc_len)]
}

/// Whether the blocks `lba..lba + len` lie within the disk.
fn lba_range_valid(lba: u32, len: u32) -> bool {
    lba as u64 + len as u64 <= BLOCKS as u64
}

#[allow(clippy::type_complexity)]
fn process_command(
    mut command: Command<ScsiCommand, Scsi<BulkOnly<UsbBus<USB>, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    defmt::info!("Handling: {:#X}", command.kind);

    match command.kind {
        ScsiCommand::TestUnitReady => {
            command.pass(0);
        }
        ScsiCommand::Inquiry { alloc_len, .. } => {
            let data = [
                0x00, // periph qualifier, periph device type
                0x80, // Removable
                0x04, // SPC-2 compliance
                0x02, // NormACA, HiSu, Response data format
                0x20, // 36 bytes in total
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                0x00, // additional fields, none set
                b'U', b'N', b'K', b'N', b'O', b'W', b'N', b' ', // 8-byte T-10 vendor id
                b'S', b'T', b'M', b'3', b'2', b' ', b'U', b'S', b'B', b' ', b'F', b'l', b'a', b's',
                b'h', b' ', // 16-byte product identification
                b'1', b'.', b'2', b'3', // 4-byte product revision
            ];
            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            command.pass(data.len() as u32);
        }
        ScsiCommand::RequestSense { alloc_len, .. } => critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);

            let data = [
                0x70,                         // RESPONSE CODE. Set to 70h for information on current errors
                0x00,                         // obsolete
                state.sense_key.unwrap_or(0), // Bits 3..0: SENSE KEY. Contains information describing the error.
                0x00,
                0x00,
                0x00,
                0x00, // INFORMATION. Device-specific or command-specific information.
                0x0A, // ADDITIONAL SENSE LENGTH. 18 bytes total - 8 byte header
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
            ];
            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            state.reset();
            command.pass(data.len() as u32);
            Ok(())
        })?,
        ScsiCommand::ReadCapacity10 => {
            let mut data = [0u8; 8];
            let _ = &mut data[0..4].copy_from_slice(&u32::to_be_bytes(BLOCKS - 1));
            let _ = &mut data[4..8].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            command.try_write_data_all(&data)?;
            command.pass(data.len() as u32);
        }
        ScsiCommand::ReadCapacity16 { alloc_len, .. } => {
            let mut data = [0u8; 16];
            let _ = &mut data[0..8].copy_from_slice(&u64::to_be_bytes((BLOCKS - 1) as u64));
            let _ = &mut data[8..12].copy_from_slice(&u32::to_be_bytes(BLOCK_SIZE));
            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            command.pass(data.len() as u32);
        }
        ScsiCommand::ReadFormatCapacities { alloc_len, .. } => {
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

            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            command.pass(data.len() as u32);
        }
        ScsiCommand::Read { lba, len } => critical_section::with(|cs| {
            let len = len as u32;

            let mut state = STATE.borrow_ref_mut(cs);

            if !lba_range_valid(lba, len) {
                defmt::error!("LBA out of range: lba={} len={}", lba, len);
                state.sense_key.replace(0x05); // illegal request Sense Key
                state.sense_key_code.replace(0x21); // Logical block address out of range ASC
                state.sense_qualifier.replace(0x00); // Logical block address out of range ASCQ
                command.fail(0);
                return Ok(());
            }

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;

                // Uncomment this in order to push data in chunks smaller than a USB packet.
                // let end = min(start + USB_PACKET_SIZE as usize - 1, end);

                defmt::info!("Data transfer >>>>>>>> [{}..{}]", start, end);
                let count = command.write_data(&STORAGE.borrow_ref_mut(cs)[start..end])?;
                state.storage_offset += count;
            } else {
                command.pass(state.storage_offset as u32);
                state.storage_offset = 0;
            }

            Ok(())
        })?,
        ScsiCommand::Write { lba, len } => critical_section::with(|cs| {
            let len = len as u32;

            let mut state = STATE.borrow_ref_mut(cs);

            if !lba_range_valid(lba, len) {
                defmt::error!("LBA out of range: lba={} len={}", lba, len);
                state.sense_key.replace(0x05); // illegal request Sense Key
                state.sense_key_code.replace(0x21); // Logical block address out of range ASC
                state.sense_qualifier.replace(0x00); // Logical block address out of range ASCQ
                command.fail(0);
                return Ok(());
            }

            if state.storage_offset != (len * BLOCK_SIZE) as usize {
                let start = (BLOCK_SIZE * lba) as usize + state.storage_offset;
                let end = (BLOCK_SIZE * lba) as usize + (BLOCK_SIZE * len) as usize;
                defmt::info!("Data transfer <<<<<<<< [{}..{}]", start, end);
                let count = command.read_data(&mut STORAGE.borrow_ref_mut(cs)[start..end])?;
                state.storage_offset += count;

                if state.storage_offset == (len * BLOCK_SIZE) as usize {
                    command.pass(state.storage_offset as u32);
                    state.storage_offset = 0;
                }
            } else {
                command.pass(state.storage_offset as u32);
                state.storage_offset = 0;
            }

            Ok(())
        })?,
        ScsiCommand::ModeSense6 { alloc_len, .. } => {
            let data = [
                0x03, // number of bytes that follow
                0x00, // the media type is SBC
                0x00, // not write-protected, no cache-control bytes support
                0x00, // no mode-parameter block descriptors
            ];
            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            command.pass(data.len() as u32);
        }
        ScsiCommand::ModeSense10 { alloc_len, .. } => {
            let data = [0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
            let data = truncate_to_alloc_len(&data, alloc_len as usize);
            command.try_write_data_all(data)?;
            command.pass(data.len() as u32);
        }
        ref unknown_scsi_kind => {
            defmt::error!("Unknown SCSI command: {:#X}", unknown_scsi_kind);
            critical_section::with(|cs| {
                let mut state = STATE.borrow_ref_mut(cs);
                state.sense_key.replace(0x05); // illegal request Sense Key
                state.sense_key_code.replace(0x20); // Invalid command operation ASC
                state.sense_qualifier.replace(0x00); // Invalid command operation ASCQ

                command.fail(0);

                Ok(())
            })?
        }
    }

    Ok(())
}
