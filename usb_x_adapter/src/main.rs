#![no_std]
#![no_main]

use cortex_m::delay::Delay;
use defmt::info;
use defmt_rtt as _;
use panic_probe as _;

use rp_pico::{
    entry,
    hal::{
        self, pac,
        uart::{UartConfig, UartPeripheral},
        usb::UsbBus,
        Clock,
    },
};

mod uart_config;
use uart_config::ConfigDTO;
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

static mut USB_BUS_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut UART_CONFIG: Option<ConfigDTO> = None;

const PID_VID: UsbVidPid = UsbVidPid(0x5678, 0x1234);

type EnabledUart0 = UartPeripheral<
    hal::uart::Enabled,
    pac::UART0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    ),
>;
fn init_for_uart() -> (Delay, EnabledUart0) {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let usb = hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut pac.RESETS,
    );
    unsafe { USB_BUS_ALLOC = Some(UsbBusAllocator::new(usb)) };
    let alloc_ref = unsafe { USB_BUS_ALLOC.as_ref().unwrap() };

    //init this first to evade BorrowMutError
    let serial = SerialPort::new(alloc_ref);
    unsafe {
        USB_SERIAL = Some(serial);
    }

    let device = UsbDeviceBuilder::new(alloc_ref, PID_VID)
        .manufacturer("Dblm0")
        .product("USB UART ADAPTER")
        .serial_number("123")
        .device_class(2) // from: https://www.usb.org/defined-class-codes
        .build();
    unsafe {
        USB_DEV = Some(device);
    }

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let uart_pins = (
        pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
    );

    unsafe {
        UART_CONFIG = Some(ConfigDTO::default());
    }

    let conf = UartConfig::from(unsafe { UART_CONFIG.as_ref().unwrap() });
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(conf, clocks.peripheral_clock.freq())
        .unwrap();
    (delay, uart)
}

#[entry]
fn main() -> ! {
    let (_delay, uart) = init_for_uart();
    let (usb, usb_sp) = unsafe { (USB_DEV.as_mut().unwrap(), USB_SERIAL.as_mut().unwrap()) };

    info!("USB Initialized");

    let mut usb_send_buf = [0u8; 128];
    let mut usb_receive_buf = [0u8; 128];

    loop {
        //Read data from UART and send it to USB Serial
        if let Ok(readen) = uart.read_raw(&mut usb_send_buf) {
            if let Ok(written) = usb_sp.write(&usb_send_buf[0..readen]) {
                info!("{} of {} byte written to USB", written, readen);
            }
        }

        if usb.poll(&mut [usb_sp]) {
            //Read data from USB Serial and send it to UART
            if let Ok(readen) = usb_sp.read(&mut usb_receive_buf) {
                let mut data_to_send = &usb_receive_buf[0..readen];
                let mut written = 0;

                while !data_to_send.is_empty() {
                    if let Ok(rem) = uart.write_raw(data_to_send) {
                        written += data_to_send.len() - rem.len();
                        data_to_send = rem;
                    }
                }

                info!("{} of {} byte written to UART", written, readen);
            }
        }
    }
}
