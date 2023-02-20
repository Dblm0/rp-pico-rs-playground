use usbd_serial::LineCoding;

mod uart_conversions;
mod usbd_conversions;

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub enum StopBits {
    One,
    Two,
}

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub enum ParityType {
    None,
    Odd,
    Even,
}
#[derive(Clone, Copy, PartialEq)]
pub enum ConvertError {
    Incompatible,
}

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
}

#[derive(defmt::Format, Clone, Copy, PartialEq)]
pub struct ConfigDTO {
    pub baudrate: u32,
    pub data_bits: DataBits,
    pub stop_bits: StopBits,
    pub parity: ParityType,
}
impl Default for ConfigDTO {
    fn default() -> Self {
        let default_cdc = LineCoding::default();
        if let Ok(line_coding) = ConfigDTO::try_from(&default_cdc) {
            line_coding
        } else {
            Self {
                baudrate: 115_200,
                data_bits: DataBits::Eight,
                stop_bits: StopBits::One,
                parity: ParityType::None,
            }
        }
    }
}
