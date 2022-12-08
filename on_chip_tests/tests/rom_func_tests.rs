#![no_std]
#![no_main]

use defmt_rtt as _;
use panic_probe as _;
use rp_pico::hal as _;

#[defmt_test::tests]
mod tests {
    use rp_pico::hal::rom_data;
    #[init]
    fn init() -> () {
        defmt::info!("ROM Copyright: {}", rom_data::copyright_string());
        defmt::info!("ROM Version: {}", rom_data::rom_version_number());
        defmt::info!("ROM Git Revision: 0x{:x}", rom_data::git_revision());
    }

    #[test]
    fn popcount_test() {
        assert!(rom_data::popcount32(0b1111) == 4);
        assert!(rom_data::popcount32(0b1110) == 3);
        assert!(rom_data::popcount32(0b1100) == 2);
        assert!(rom_data::popcount32(0b1000) == 1);
        assert!(rom_data::popcount32(0b0000) == 0);
    }

    #[test]
    fn reverse_test() {
        assert!(rom_data::reverse32(0b1111) == 0b1111 << 28);
        assert!(rom_data::reverse32(0b1110) == 0b0111 << 28);
        assert!(rom_data::reverse32(0b1100) == 0b0011 << 28);
        assert!(rom_data::reverse32(0b1000) == 0b0001 << 28);
        assert!(rom_data::reverse32(0b0000) == 0b0000 << 28);

        assert!(rom_data::reverse32(0b1010) == 0b0101 << 28);
    }

    #[test]
    fn count_lead_zeroes_test() {
        assert!(rom_data::clz32(0) == 32);
        assert!(rom_data::clz32(0b1111) == 32 - 4);
        assert!(rom_data::clz32(0b1111_1111) == 32 - 8);
    }

    #[test]
    fn count_trail_zeroes_test() {
        assert!(rom_data::ctz32(0) == 32);
        assert!(rom_data::ctz32(0b1111 << 28) == 32 - 4);
        assert!(rom_data::ctz32(0b1111_1111 << 24) == 32 - 8);
    }

    #[test]
    fn float_v1_sqrt_test() {
        assert!(rom_data::float_funcs::fsqrt(-1.0) == f32::INFINITY);
        assert!(rom_data::float_funcs::fsqrt(2.0) - 1.4142135 < f32::EPSILON);
        assert!(rom_data::float_funcs::fsqrt(25.0) == 5.0);
    }
}
