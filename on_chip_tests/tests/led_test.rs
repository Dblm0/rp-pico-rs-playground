#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp_pico::hal as _;

#[defmt_test::tests]
mod tests {
    use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin, ToggleableOutputPin};
    use rp_pico::{
        hal::{
            clocks,
            gpio::{bank0::Gpio25, Output, Pin, PushPull},
            pac, Sio, Watchdog,
        },
        Pins,
    };
    #[init]
    fn init() -> Pin<Gpio25, Output<PushPull>> {
        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = Watchdog::new(pac.WATCHDOG);
        let _clocks = clocks::init_clocks_and_plls(
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

        let sio = Sio::new(pac.SIO);
        let pins = Pins::new(
            pac.IO_BANK0,
            pac.PADS_BANK0,
            sio.gpio_bank0,
            &mut pac.RESETS,
        );

        pins.led.into_push_pull_output()
    }

    // #[before_each]
    // fn before_each(state: &mut Pin<Gpio25, Output<PushPull>>) {
    //     state.set_low().unwrap();
    // }

    #[test]
    fn led_test(state: &mut Pin<Gpio25, Output<PushPull>>) {
        state.toggle().unwrap();
        debug_assert!(state.is_set_high().unwrap());
        state.toggle().unwrap();
        debug_assert!(state.is_set_low().unwrap())
    }

    #[test]
    #[should_panic]
    fn should_fail(state: &mut Pin<Gpio25, Output<PushPull>>) {
        state.toggle().unwrap();
        debug_assert!(state.is_set_low().unwrap());
        state.toggle().unwrap();
        debug_assert!(state.is_set_high().unwrap());
    }
}
