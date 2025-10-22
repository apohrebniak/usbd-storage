#![no_std]
#![no_main]

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;
use defmt_rtt as _;
use stm32f4xx_hal::gpio::alt::otg_fs::{Dm, Dp};

use core::cell::RefCell;
use critical_section::Mutex;

use stm32f4xx_hal::gpio::GpioExt;
use stm32f4xx_hal::otg_fs::{UsbBus, USB};
use stm32f4xx_hal::pac;
use stm32f4xx_hal::prelude::*;
use stm32f4xx_hal::rcc::RccExt;

use usb_device::prelude::*;
use usbd_storage::subclass::ufi::{Ufi, UfiCommand};
use usbd_storage::subclass::Command;
use usbd_storage::transport::bbb::{BulkOnly, BulkOnlyError};
use usbd_storage::transport::TransportError;

static mut USB_EP_MEMORY: [u32; 1024] = [0u32; 1024];
/// Not necessarily `'static`. May reside in some special memory location
static mut USB_TRANSPORT_BUF: MaybeUninit<[u8; 512]> = MaybeUninit::uninit();

static FAT: &[u8] = include_bytes!("../../cat_fat12.img"); // part of fat12 fs with some data

static STATE: Mutex<RefCell<State>> = Mutex::new(RefCell::new(State {
    storage_offset: 0,
    sense_key: None,
    sense_key_code: None,
    sense_qualifier: None,
}));

const BLOCK_SIZE: usize = 512;
const USB_PACKET_SIZE: u16 = 64; // 8,16,32,64

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
    let mut pin_usb_dm = gpioa.pa11.into_push_pull_output();
    let mut pin_usb_dp = gpioa.pa12.into_push_pull_output();
    // indicator LED
    let mut led = gpioc.pc13.into_push_pull_output();

    // force D+ for 100ms
    // this will force the host to enumerate devices
    pin_usb_dm.set_low();
    pin_usb_dp.set_low();
    cp.SYST.delay(&clocks).delay_ms(100u32);

    let usb_peripheral = stm32f4xx_hal::otg_fs::USB {
        usb_global: dp.OTG_FS_GLOBAL,
        usb_device: dp.OTG_FS_DEVICE,
        usb_pwrclk: dp.OTG_FS_PWRCLK,
        pin_dm: Dm::from(pin_usb_dm.into_alternate()),
        pin_dp: Dp::from(pin_usb_dp.into_alternate()),
        hclk: clocks.hclk(),
    };

    let usb_bus = UsbBus::new(usb_peripheral, unsafe { &mut *addr_of_mut!(USB_EP_MEMORY) });
    let mut ufi = usbd_storage::subclass::ufi::Ufi::new(&usb_bus, USB_PACKET_SIZE, unsafe {
        #[allow(static_mut_refs)]
        USB_TRANSPORT_BUF.assume_init_mut().as_mut_slice()
    })
    .unwrap();

    let mut usb_device = UsbDeviceBuilder::new(&usb_bus, UsbVidPid(0xabcd, 0xabcd))
        .strings(&[StringDescriptors::new(LangID::EN)
            .manufacturer("Foo Bar")
            .product("STM32 USB Floppy")
            .serial_number("FOOBAR1234567890ABCDEF")])
        .unwrap()
        .self_powered(false)
        .build();

    loop {
        led.set_high();

        if !usb_device.poll(&mut [&mut ufi]) {
            continue;
        }

        // clear state if just configured or reset
        if matches!(usb_device.state(), UsbDeviceState::Default) {
            critical_section::with(|cs| {
                STATE.borrow_ref_mut(cs).reset();
            })
        }

        let _ = ufi.poll(|command| {
            led.set_low();
            if let Err(err) = process_command(command) {
                defmt::error!("{}", err);
            }
        });
    }
}

fn process_command(
    mut command: Command<UfiCommand, Ufi<BulkOnly<UsbBus<USB>, &mut [u8]>>>,
) -> Result<(), TransportError<BulkOnlyError>> {
    defmt::info!("Handling: {}", command.kind);

    match command.kind {
        UfiCommand::Inquiry { .. } => {
            command.try_write_data_all(&[
                0x00, 0b10000000, 0, 0x01, 0x1F, 0, 0, 0, b'F', b'o', b'o', b' ', b'B', b'a', b'r',
                b'0', b'F', b'o', b'o', b' ', b'B', b'a', b'r', b'0', b'F', b'o', b'o', b' ', b'B',
                b'a', b'r', b'0', b'1', b'.', b'2', b'3',
            ])?;
            command.pass();
        }
        UfiCommand::StartStop { .. }
        | UfiCommand::TestUnitReady
        | UfiCommand::PreventAllowMediumRemoval { .. } => {
            command.pass();
        }
        UfiCommand::ReadCapacity => {
            command.try_write_data_all(&[0x00, 0x00, 0x0b, 0x3f, 0x00, 0x00, 0x02, 0x00])?;
            command.pass();
        }
        UfiCommand::RequestSense { .. } => critical_section::with(|cs| {
            let mut state = STATE.borrow_ref_mut(cs);

            command.try_write_data_all(&[
                0x70, // error code
                0x00,
                state.sense_key.unwrap_or(0),
                0x00,
                0x00,
                0x00,
                0x00,
                0x0A, // additional length
                0x00,
                0x00,
                0x00,
                0x00,
                state.sense_key_code.unwrap_or(0),
                state.sense_qualifier.unwrap_or(0),
                0x00,
                0x00,
                0x00,
                0x00,
            ])?;

            state.reset();
            command.pass();
            Ok(())
        })?,
        UfiCommand::ModeSense { .. } => {
            /* Read Only */
            command.try_write_data_all(&[0x00, 0x46, 0x02, 0x80, 0x00, 0x00, 0x00, 0x00])?;

            /* Read Write */
            // command.try_write_data_all(&[0x00, 0x46, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00])?;

            command.pass();
        }
        UfiCommand::Write { .. } => {
            command.pass();
        }
        UfiCommand::Read { lba, len } => critical_section::with(|cs| {
            let lba = lba as u32;
            let len = len as u32;

            let mut state = STATE.borrow_ref_mut(cs);

            if state.storage_offset != len as usize * BLOCK_SIZE {
                const DUMP_MAX_LBA: u32 = 0xCE;
                if lba < DUMP_MAX_LBA {
                    /* requested data from dump */
                    let start = (BLOCK_SIZE * lba as usize) + state.storage_offset;
                    let end = (BLOCK_SIZE * lba as usize) + (BLOCK_SIZE as usize * len as usize);
                    defmt::info!("Data transfer >>>>>>>> [{}..{}]", start, end);
                    let count = command.write_data(&FAT[start..end])?;
                    state.storage_offset += count;
                } else {
                    /* fill with 0xF6 */
                    loop {
                        let count = command.write_data(&[0xF6; BLOCK_SIZE as usize])?;
                        state.storage_offset += count;
                        if count == 0 {
                            break;
                        }
                    }
                }
            } else {
                command.pass();
                state.storage_offset = 0;
            }

            Ok(())
        })?,
        ref unknown_ufi_kind => {
            defmt::error!("Unknown UFI command: {}", unknown_ufi_kind);

            critical_section::with(|cs| {
                let mut state = STATE.borrow_ref_mut(cs);
                state.sense_key.replace(0x05); // illegal request
                state.sense_key_code.replace(0x20); // Invalid command operation
                state.sense_qualifier.replace(0x00); // Invalid command operation
            });

            command.fail();
        }
    }

    Ok(())
}
