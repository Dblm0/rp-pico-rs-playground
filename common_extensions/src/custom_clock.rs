// inspired by https://github.com/rp-rs/rp-hal/issues/625
// copied and modified from https://github.com/Neotron-Compute/Neotron-Pico-BIOS/blob/develop/src/main.rs#L346-L404

use fugit::RateExtU32;
use rp_pico::{
    hal::{
        clocks::{self, ClocksManager, InitError},
        pll::{self, PLLConfig},
        xosc::setup_xosc_blocking,
        Watchdog,
    },
    pac::{CLOCKS, PLL_SYS, PLL_USB, RESETS, XOSC},
};

pub struct PllConfigurator {
    config: PLLConfig,
}
impl PllConfigurator {
    pub fn with(config: PLLConfig) -> Self {
        PllConfigurator { config }
    }
    #[allow(clippy::too_many_arguments)] //Just wanted to keep the original signature
    pub fn init_clocks_and_plls(
        self,
        xosc_crystal_freq: u32,
        xosc_dev: XOSC,
        clocks_dev: CLOCKS,
        pll_sys_dev: PLL_SYS,
        pll_usb_dev: PLL_USB,
        resets: &mut RESETS,
        watchdog: &mut Watchdog,
    ) -> Result<ClocksManager, InitError> {
        let xosc =
            setup_xosc_blocking(xosc_dev, xosc_crystal_freq.Hz()).map_err(InitError::XoscErr)?;

        watchdog.enable_tick_generation((xosc_crystal_freq / 1_000_000) as u8);
        let mut clocks = clocks::ClocksManager::new(clocks_dev);

        let pll_sys = pll::setup_pll_blocking(
            pll_sys_dev,
            xosc.operating_frequency(),
            self.config,
            &mut clocks,
            resets,
        )
        .map_err(|_x| false)
        .unwrap();

        let pll_usb = pll::setup_pll_blocking(
            pll_usb_dev,
            xosc.operating_frequency(),
            pll::common_configs::PLL_USB_48MHZ,
            &mut clocks,
            resets,
        )
        .map_err(|_x| false)
        .unwrap();

        clocks
            .init_default(&xosc, &pll_sys, &pll_usb)
            .map_err(|_x| false)
            .unwrap();
        Ok(clocks)
    }
}
