#![no_std]
#![no_main]

use embedded_hal::PwmPin;
use embedded_time::rate::*;
use rp_pico as bsp;
use rp_pico::hal;
use rp_pico::hal::pac;
use rp_pico::hal::prelude::*;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// The minimum PWM value (i.e. LED brightness) we want
const LOW: u16 = 0;
// The maximum PWM value (i.e. LED brightness) we want
const HIGH: u16 = 25000;
// Text data blob

const BLOB: &str = big_firmware::generate_blob!();

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let pwm = &mut pwm_slices.pwm4;
    pwm.set_ph_correct();
    pwm.enable();

    let channel = &mut pwm.channel_b;
    channel.output_to(pins.led);

    info!("blob read begin:");
    let mut d = 0;
    for _line in BLOB.lines() {
        d += 1;
    }
    info!("blob read end. total lines = {}", d);
    delay.delay_ms(1_000);

    // Infinite loop, fading LED up and down
    loop {
        // Ramp brightness up
        for i in (LOW..=HIGH).skip(100) {
            delay.delay_us(16);
            channel.set_duty(i);
        }

        // Ramp brightness down
        for i in (LOW..=HIGH).rev().skip(100) {
            delay.delay_us(4);
            channel.set_duty(i);
        }

        delay.delay_ms(250);
    }
}
