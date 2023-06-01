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
        27.0 - (sensor_v - 0.706) / 0.001721
    }
}
