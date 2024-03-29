#![no_std]
#![no_main]

mod uart_config;

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use defmt::info;
    use defmt_rtt as _;
    use panic_probe as _;

    use crate::uart_config::ConfigDTO;
    use embedded_hal::PwmPin;
    use rp_pico::hal::{
        self, pac,
        uart::{UartConfig, UartPeripheral},
        usb::UsbBus,
        Clock,
    };
    use usb_device::{
        class_prelude::UsbBusAllocator,
        prelude::{UsbDevice, UsbDeviceBuilder, UsbVidPid},
    };
    use usbd_serial::SerialPort;

    type UartConfigWithRate = (ConfigDTO, fugit::HertzU32);
    type UartPins = (
        hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::Function<hal::gpio::Uart>>,
    );

    type EnabledUart = hal::uart::UartPeripheral<hal::uart::Enabled, pac::UART0, UartPins>;
    type UartReader = rp_pico::hal::uart::Reader<pac::UART0, UartPins>;
    type UartWriter = rp_pico::hal::uart::Writer<pac::UART0, UartPins>;
    type PwmLed = hal::pwm::Channel<hal::pwm::Pwm4, hal::pwm::FreeRunning, hal::pwm::B>;

    #[shared]
    struct Shared {
        uart_reader: Option<UartReader>,
        uart_writer: Option<UartWriter>,
        usb_serial: SerialPort<'static, UsbBus>,
    }

    //make both buffers the same length to prevent data losses
    const BUF_SIZE: usize = 32;

    static mut GLOBAL_TIMER: Option<hal::Timer> = None;

    defmt::timestamp!("{=u64:us}", unsafe {
        if let Some(timer) = &GLOBAL_TIMER {
            timer.get_counter().ticks()
        } else {
            0
        }
    });
    #[local]
    struct Local {
        led: PwmLed,
        delay: cortex_m::delay::Delay,
        uart_config: UartConfigWithRate,
        usb_dev: UsbDevice<'static, UsbBus>,
    }
    const PID_VID: UsbVidPid = UsbVidPid(0x5678, 0x1234);

    #[init(local = [USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut pac = c.device;
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

        let usb_alloc = UsbBusAllocator::new(hal::usb::UsbBus::new(
            pac.USBCTRL_REGS,
            pac.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut pac.RESETS,
        ));
        c.local.USB_ALLOCATOR.replace(usb_alloc);
        let usb_alloc = c.local.USB_ALLOCATOR.as_ref().unwrap();

        //init this first to evade BorrowMutError
        let usb_serial = SerialPort::new(usb_alloc);

        let usb_dev = UsbDeviceBuilder::new(usb_alloc, PID_VID)
            .manufacturer("Dblm0")
            .product("USB UART ADAPTER")
            .device_class(2) // from: https://www.usb.org/defined-class-codes
            .build();

        info!("USB Initialized");
        let sio = hal::Sio::new(pac.SIO);

        let pins = rp_pico::Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );
        let uart_pins: UartPins = (
            pins.gpio0.into_mode::<hal::gpio::FunctionUart>(),
            pins.gpio1.into_mode::<hal::gpio::FunctionUart>(),
        );

        let rate = clocks.peripheral_clock.freq();
        let uart_config = (ConfigDTO::default(), rate);

        let mut uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
            .enable(UartConfig::from(&uart_config.0), uart_config.1)
            .unwrap();
        uart.enable_rx_interrupt();

        let (uart_reader, uart_writer) = uart.split();
        let (uart_reader, uart_writer) = (Some(uart_reader), Some(uart_writer));

        let delay = cortex_m::delay::Delay::new(c.core.SYST, clocks.system_clock.freq().to_Hz());

        let mut pwm = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS).pwm4;
        pwm.set_top(25000);
        pwm.channel_b.output_to(pins.led);
        pwm.set_ph_correct();
        pwm.enable();

        unsafe { GLOBAL_TIMER.replace(rp_pico::hal::Timer::new(pac.TIMER, &mut pac.RESETS)) };
        (
            Shared {
                usb_serial,
                uart_reader,
                uart_writer,
            },
            Local {
                led: pwm.channel_b,
                delay,
                usb_dev,
                uart_config,
            },
            init::Monotonics(),
        )
    }
    #[idle(local = [delay, led])]
    fn idle(c: idle::Context) -> ! {
        loop {
            for i in (0..=25000).skip(100) {
                c.local.delay.delay_us(32);
                c.local.led.set_duty(i);
            }

            for i in (0..=25000).rev().skip(100) {
                c.local.delay.delay_us(32);
                c.local.led.set_duty(i);
            }
            c.local.delay.delay_us(64);
        }
    }

    #[task(
        binds = UART0_IRQ,
        priority = 2,
        shared = [usb_serial, uart_reader],
        local = [usb_send_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]]
    )]
    fn uart_irq(c: uart_irq::Context) {
        (c.shared.usb_serial, c.shared.uart_reader).lock(|usb_serial, uart| {
            // Read data from UART and send it to USB Serial
            let uart = uart.as_ref().unwrap();
            let usb_send_buf = c.local.usb_send_buf;
            //TODO:Fix data loss due to the pauses caused by defmt
            if let Ok(readen) = uart.read_raw(usb_send_buf) {
                if let Ok(written) = usb_serial.write(&usb_send_buf[0..readen]) {
                    assert_eq!(written, readen);
                    info!("{} of {} byte written to USB", written, readen);
                }
            }
        });
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 1, //priority is less than in uart_irq task to prevent data losses
        shared = [usb_serial, uart_writer, uart_reader],
        local = [
            usb_dev,
            uart_config,
            usb_receive_buf: [u8; BUF_SIZE] = [0; BUF_SIZE]
        ],
    )]
    fn usbctrl_irq(mut c: usbctrl_irq::Context) {
        let poll_res = c
            .shared
            .usb_serial
            .lock(|usb_serial| c.local.usb_dev.poll(&mut [usb_serial]));

        if !poll_res {
            return;
        }

        //Read data from USB Serial and send it to UART
        (&mut c.shared.usb_serial, &mut c.shared.uart_writer).lock(|usb_serial, uart| {
            let uart = uart.as_ref().unwrap();
            let usb_receive_buf = c.local.usb_receive_buf;
            if let Ok(readen) = usb_serial.read(usb_receive_buf) {
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
        });

        let new_conf = c
            .shared
            .usb_serial
            .lock(|usb_serial| ConfigDTO::try_from(usb_serial.line_coding()));

        match new_conf {
            Ok(new) if new != c.local.uart_config.0 => {
                (c.shared.uart_reader, c.shared.uart_writer).lock(|r_container, w_container| {
                    let (r, w) = (r_container.take().unwrap(), w_container.take().unwrap());
                    let mut uart = EnabledUart::join(r, w);
                    uart.disable_rx_interrupt();
                    let uart = uart.disable();
                    let mut uart = uart
                        .enable(UartConfig::from(&new), c.local.uart_config.1)
                        .unwrap();
                    uart.enable_rx_interrupt();
                    let (r, w) = uart.split();
                    r_container.replace(r);
                    w_container.replace(w);
                });
                c.local.uart_config.0 = new;
                info!("UART reconfigured :{}", c.local.uart_config.0);
            }
            _ => (),
        }
    }
}
