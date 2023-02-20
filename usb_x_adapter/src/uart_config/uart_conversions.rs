use super::{ConfigDTO, DataBits, ParityType, StopBits};
use rp_pico::hal::uart;

impl From<StopBits> for uart::StopBits {
    fn from(value: StopBits) -> Self {
        match value {
            StopBits::One => Self::One,
            StopBits::Two => Self::Two,
        }
    }
}

impl From<uart::StopBits> for StopBits {
    fn from(value: uart::StopBits) -> Self {
        match value {
            uart::StopBits::One => Self::One,
            uart::StopBits::Two => Self::Two,
        }
    }
}
impl From<ParityType> for Option<uart::Parity> {
    fn from(value: ParityType) -> Self {
        match value {
            ParityType::None => None,
            ParityType::Even => Some(uart::Parity::Even),
            ParityType::Odd => Some(uart::Parity::Odd),
        }
    }
}
impl From<Option<uart::Parity>> for ParityType {
    fn from(value: Option<uart::Parity>) -> Self {
        match value {
            None => Self::None,
            Some(uart::Parity::Even) => Self::Even,
            Some(uart::Parity::Odd) => Self::Odd,
        }
    }
}

impl From<uart::DataBits> for DataBits {
    fn from(value: uart::DataBits) -> Self {
        match value {
            uart::DataBits::Five => Self::Five,
            uart::DataBits::Six => Self::Six,
            uart::DataBits::Seven => Self::Seven,
            uart::DataBits::Eight => Self::Eight,
        }
    }
}
impl From<DataBits> for uart::DataBits {
    fn from(value: DataBits) -> Self {
        match value {
            DataBits::Five => Self::Five,
            DataBits::Six => Self::Six,
            DataBits::Seven => Self::Seven,
            DataBits::Eight => Self::Eight,
        }
    }
}

impl From<uart::UartConfig> for ConfigDTO {
    fn from(value: uart::UartConfig) -> Self {
        Self {
            data_bits: value.data_bits.into(),
            stop_bits: value.stop_bits.into(),
            parity: value.parity.into(),
            baudrate: value.baudrate,
        }
    }
}
impl From<&ConfigDTO> for uart::UartConfig {
    fn from(value: &ConfigDTO) -> Self {
        let mut config = uart::UartConfig::default();
        config.data_bits = value.data_bits.into();
        config.stop_bits = value.stop_bits.into();
        config.parity = value.parity.into();
        config.baudrate = value.baudrate;
        config
    }
}
