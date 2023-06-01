#![no_std]
#![no_main]

use rp_pico as bsp;
use rp_pico::hal;
use rp_pico::hal::pac;

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

const BLOB: &str = big_firmware::generate_blob!();

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let _clocks = hal::clocks::init_clocks_and_plls(
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

    info!("blob read begin:");
    let mut d = 0;
    for _line in BLOB.lines() {
        d += 1;
    }
    info!("blob read end. total lines = {}", d);

    loop {
        cortex_m::asm::wfi();
    }
}
