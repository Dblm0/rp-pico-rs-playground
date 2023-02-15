#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embedded_hal::PwmPin;
use fugit::HertzU32;
use panic_probe as _;

use rp_pico::{
    entry,
    hal::{self, pac, pwm::Slice, Clock},
};

const INT: u8 = 1;
const FRAQ: u8 = 0;
const PHASE_CORR: bool = true;

const CLK_DIV: f32 = INT as f32 + (FRAQ % 16) as f32 / 16.0;
const TOP: u16 = u16::MAX / 16;
const L: usize = 256;

#[entry]
fn main() -> ! {
    let (mut pwm, sys_freq) = init();

    let k = if PHASE_CORR { 0.5 } else { 1.0 };
    let carrier_f = k * sys_freq.to_Hz() as f32 / TOP as f32 / CLK_DIV;
    info!("PWM CLK_DIV {}", CLK_DIV);
    info!("PWM carrier frequency: {} kHz", carrier_f / 1e3);

    let s_freq = carrier_f / L as f32;
    info!("Signal Frequency: {} Hz", s_freq);

    let mut signal = [0u16; L];
    for (i, s) in signal.iter_mut().enumerate() {
        let value = num::Float::sin(i as f32 * core::f32::consts::TAU / L as f32);
        let value = (value + 1.0) / 2.0;
        let value = num::Float::round(value * TOP as f32) as u16;
        *s = value;
    }

    loop {
        for s in signal {
            while !pwm.has_overflown() {
                cortex_m::asm::nop();
            }
            pwm.clear_interrupt();
            pwm.channel_a.set_duty(s);
        }
    }
}

fn init() -> (Slice<hal::pwm::Pwm1, hal::pwm::FreeRunning>, HertzU32) {
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

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);
    let mut pwm = pwm_slices.pwm1;

    pwm.set_div_int(INT);
    pwm.set_div_frac(FRAQ);

    if PHASE_CORR {
        pwm.set_ph_correct();
    } else {
        pwm.clr_ph_correct();
    }
    pwm.enable_interrupt();
    pwm.set_top(TOP);
    pwm.enable();

    pwm.channel_a.output_to(pins.gpio2);

    (pwm, clocks.system_clock.freq())
}
