#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp_pico::hal as _;

mod sens {
    use embedded_hal::adc::OneShot;
    use rp_pico::hal::{adc::TempSense, Adc};
    pub struct TemperatureSensor {
        adc: Adc,
        sensor: TempSense,
    }

    const IO_VDD: f32 = 3.3;
    const ADC_FACTOR: f32 = 4096.0;

    impl TemperatureSensor {
        pub fn new(adc: Adc, sensor: TempSense) -> Self {
            TemperatureSensor { adc, sensor }
        }
        pub fn read_temperature(&mut self) -> f32 {
            let digits: u16 = self.adc.read(&mut self.sensor).unwrap();
            let sensor_v = IO_VDD * (digits as f32 / ADC_FACTOR);
            27.0 - (sensor_v as f32 - 0.706) / 0.001721
        }
    }
}

#[defmt_test::tests]
mod tests {
    use num_traits::float::FloatCore;
    use rp_pico::hal::{pac, Adc};

    #[init]
    fn init() -> super::sens::TemperatureSensor {
        let mut pac = pac::Peripherals::take().unwrap();
        let mut watchdog = rp_pico::hal::Watchdog::new(pac.WATCHDOG);
        let _clocks = rp_pico::hal::clocks::init_clocks_and_plls(
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

        let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
        let sens = adc.enable_temp_sensor();
        super::sens::TemperatureSensor::new(adc, sens)
    }

    #[before_each]
    fn before_each(state: &mut super::sens::TemperatureSensor) {
        let meas = state.read_temperature();
        defmt::info!("Current temperature = {}°C", meas);
    }

    #[test]
    fn max_temp_test(state: &mut super::sens::TemperatureSensor) {
        let meas = state.read_temperature();
        let max_temp = 33.0;
        debug_assert!(meas < max_temp, "meas = {}°C max = {}°C", meas, max_temp);
    }
    #[test]
    fn min_temp_test(state: &mut super::sens::TemperatureSensor) {
        let meas = state.read_temperature();
        let min_temp = 25.0;
        debug_assert!(meas > min_temp, "meas = {}°C min = {}°C", meas, min_temp);
    }
    #[test]
    fn set_point_test(state: &mut super::sens::TemperatureSensor) {
        let meas = state.read_temperature();
        let target = 27.0;
        let err_bias = 2.0;
        debug_assert!(
            f32::abs(meas - target) < err_bias,
            "meas = {}°C target = {}°C",
            meas,
            target
        );
    }
}
