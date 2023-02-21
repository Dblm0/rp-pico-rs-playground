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
type UartConfigWithRate = (ConfigDTO, fugit::HertzU32);
use usb_device::{
    class_prelude::UsbBusAllocator,
    prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
};
use usbd_serial::SerialPort;

static mut USB_BUS_ALLOC: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_DEV: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut UART_CONFIG: Option<UartConfigWithRate> = None;
static mut UART: Option<EnabledUart0> = None;

const PID_VID: UsbVidPid = UsbVidPid(0x5678, 0x1234);

type EnabledUart0 = UartPeripheral<
    hal::uart::Enabled,
    pac::UART0,
    (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    ),
>;
fn init_statics() {
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
        let rate = clocks.peripheral_clock.freq();
        UART_CONFIG = Some((ConfigDTO::default(), rate));
    }

    let conf = unsafe { UART_CONFIG.as_ref().unwrap() };
    let rate = conf.1;
    let conf = UartConfig::from(&conf.0);
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(conf, rate)
        .unwrap();
    unsafe { UART = Some(uart) };
}

fn try_reconfigure_uart(
    uart: EnabledUart0,
    new_config: ConfigDTO,
) -> Result<EnabledUart0, hal::uart::Error> {
    let uart_config = UartConfig::from(&new_config);
    let old_conf = unsafe { UART_CONFIG.as_ref().unwrap() };
    let uart = uart.disable();

    let result = uart.enable(uart_config, old_conf.1);
    if result.is_ok() {
        unsafe { UART_CONFIG = Some((new_config, old_conf.1)) }
    }
    result
}

#[entry]
fn main() -> ! {
    init_statics();

    let (usb, usb_sp) = unsafe { (USB_DEV.as_mut().unwrap(), USB_SERIAL.as_mut().unwrap()) };
    let mut new_config: Option<ConfigDTO> = None;
    info!("USB Initialized");

    let mut usb_send_buf = [0u8; 128];
    let mut usb_receive_buf = [0u8; 128];

    loop {
        if let Some(conf) = new_config {
            unsafe {
                if let Ok(reconfigured) = try_reconfigure_uart(UART.take().unwrap(), conf) {
                    UART.replace(reconfigured);
                    info!("UART reconfigured :{}", conf);
                }
            };
            let _ = new_config.take();
        }
        let uart = unsafe { UART.as_ref().unwrap() };
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

            if let Ok(conf) = ConfigDTO::try_from(usb_sp.line_coding()) {
                if unsafe { conf != UART_CONFIG.unwrap().0 } {
                    new_config = Some(conf);
                    break;
                }
            }
        }
    }
}
