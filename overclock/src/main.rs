#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

use bsp::{
    entry,
    hal::{
        gpio::{FunctionPio0, PinId},
        pac,
        pll::PLLConfig,
        prelude::_rphal_pio_PIOExt,
        sio::Sio,
        watchdog::Watchdog,
        Adc, Clock,
    },
};
use rp_pico as bsp;

use common_extensions::custom_clock::PllConfigurator;

pub const PLL_SYS_250MHZ: PLLConfig = PLLConfig {
    vco_freq: fugit::HertzU32::MHz(1500),
    refdiv: 1,
    post_div1: 3,
    post_div2: 2,
};

#[entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let core = pac::CorePeripherals::take().unwrap();
    let clocks = PllConfigurator::with(PLL_SYS_250MHZ)
        .init_clocks_and_plls(
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

    let sys_freq = clocks.system_clock.freq().to_Hz();
    info!("System clock freq :{} MHz", sys_freq as f32 / 1e6);

    let sio = Sio::new(pac.SIO);

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let test_pin = pins.gpio2.into_mode::<FunctionPio0>();
    let program = create_pio_prog();
    let (mut pio, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);
    let installed = pio.install(&program).unwrap();
    let (int, frac) = (0, 0); // as slow as possible (0 is interpreted as 65536)
    let (sm, _, _) = bsp::hal::pio::PIOBuilder::from_program(installed)
        .set_pins(get_pin_id(test_pin), 1)
        .clock_divisor_fixed_point(int, frac)
        .build(sm0);
    sm.start();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, sys_freq);
    let mut adc = Adc::new(pac.ADC, &mut pac.RESETS);
    let sens = adc.enable_temp_sensor();
    let mut temp_sensor = sens::TemperatureSensor::new(adc, sens);

    loop {
        let t = temp_sensor.read_temperature();
        defmt::info!("Current temperature = {}°C", t);
        delay.delay_ms(1000);
    }
}

fn get_pin_id<I, M>(_: bsp::hal::gpio::Pin<I, M>) -> u8
where
    I: PinId,
    M: bsp::hal::gpio::PinMode + bsp::hal::gpio::ValidPinMode<I>,
{
    I::DYN.num
}

fn create_pio_prog() -> pio::Program<32> {
    const MAX_DELAY: u8 = 0;
    let mut a = pio::Assembler::<32>::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    a.set(pio::SetDestination::PINDIRS, 1);
    a.bind(&mut wrap_target);
    a.set_with_delay(pio::SetDestination::PINS, 0, MAX_DELAY);
    a.set_with_delay(pio::SetDestination::PINS, 1, MAX_DELAY);
    a.bind(&mut wrap_source);
    a.assemble_with_wrap(wrap_source, wrap_target)
}

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
            27.0 - (sensor_v - 0.706) / 0.001721
        }
    }
}
