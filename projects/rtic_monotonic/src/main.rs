#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;

defmt::timestamp!("{=u64:us}", {
    crate::app::monotonics::now().ticks()
});

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [SW0_IRQ])]
mod app {
    use defmt::info;
    use embedded_hal::digital::v2::OutputPin;
    use fugit::ExtU64;
    use hal::Clock;
    use rp_pico::{
        hal::{
            self,
            clocks::init_clocks_and_plls,
            timer::{monotonic::Monotonic, Alarm0},
            watchdog::Watchdog,
            Sio,
        },
        XOSC_CRYSTAL_FREQ,
    };

    type TestPin = hal::gpio::Pin<hal::gpio::pin::bank0::Gpio2, hal::gpio::PushPullOutput>;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        test_pin: TestPin,
    }

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type MyMono = Monotonic<Alarm0>;

    #[init]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        // Soft-reset does not release the hardware spinlocks
        // Release them now to avoid a deadlock after debug or watchdog reset
        unsafe {
            hal::sio::spinlock_reset();
        }
        let pac = c.device;
        let mut resets = pac.RESETS;
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let _clocks = init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            pac.XOSC,
            pac.CLOCKS,
            pac.PLL_SYS,
            pac.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        info!(
            "System clock freq :{} MHz",
            _clocks.system_clock.freq().to_Hz() as f32 / 1e6
        );

        let sio = Sio::new(pac.SIO);
        let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);
        let mut test_pin = pins.gpio2.into_push_pull_output();
        test_pin.set_low().unwrap();

        let mut timer = hal::Timer::new(pac.TIMER, &mut resets);
        let alarm = timer.alarm_0().unwrap();

        pin_toggle::spawn().unwrap();

        (
            Shared {},
            Local { test_pin },
            init::Monotonics(Monotonic::new(timer, alarm)),
        )
    }

    #[task(
        local = [test_pin, count:u32 = 0]
    )]
    fn pin_toggle(c: pin_toggle::Context) {
        let pin = c.local.test_pin;
        pin.set_high().unwrap();
        pin.set_low().unwrap();
        *c.local.count = (*c.local.count + 1) % 1000;
        if *c.local.count % 5 == 0 {
            pin_toggle::spawn_after(100.millis()).unwrap();
            info!("Pin toggled!");
        } else {
            pin_toggle::spawn_after(100.micros()).unwrap();
        }
    }
}
