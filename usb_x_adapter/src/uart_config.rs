use fugit::HertzU32;

mod uart_conversions;
mod usbd_conversions;

#[derive(Clone, Copy, PartialEq)]
pub enum StopBits {
    One,
    Two,
}

#[derive(Clone, Copy, PartialEq)]
pub enum ParityType {
    None,
    Odd,
    Even,
}
#[derive(Clone, Copy, PartialEq)]
pub enum ConvertError {
    Incompatible,
}

#[derive(Clone, Copy, PartialEq)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
}

#[derive(Clone, Copy, PartialEq)]
pub struct ConfigDTO {
    pub baudrate: HertzU32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: ParityType,
}
impl Default for ConfigDTO {
    fn default() -> Self {
        Self {
            baudrate: HertzU32::from_raw(115_200),
            data_bits: DataBits::Eight,
            stop_bits: StopBits::One,
            parity: ParityType::None,
        }
    }
}
