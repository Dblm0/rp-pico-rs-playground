#![no_std]
#![no_main]

use cortex_m::prelude::_embedded_hal_adc_OneShot;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use bsp::{
    entry,
    hal::{
        clocks::{init_clocks_and_plls, Clock},
        pac,
        watchdog::Watchdog,
        Adc,
    },
};
use rp_pico as bsp;

const IO_VDD: f32 = 3.3;
const ADC_FACTOR: f32 = 4096.0;

fn convert_temperature(digits: u16) -> f32 {
    let sensor_v = IO_VDD * (digits as f32 / ADC_FACTOR);
    27.0 - (sensor_v as f32 - 0.706) / 0.001721
}
#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);

    let clocks = init_clocks_and_plls(
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // Enable ADC
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    // Enable the temperature sense channel
    let mut temperature_sensor = adc.enable_temp_sensor();

    loop {
        // Read the raw ADC counts from the temperature sensor channel.
        let sensor_read: u16 = adc.read(&mut temperature_sensor).unwrap();
        debug!("int = {}", sensor_read);
        debug!("t = {}°C", convert_temperature(sensor_read));
        delay.delay_ms(1000);
    }
}
