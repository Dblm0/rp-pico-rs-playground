use super::{ConfigDTO, ConvertError, DataBits, ParityType, StopBits};

impl From<ParityType> for usbd_serial::ParityType {
    fn from(value: ParityType) -> Self {
        match value {
            ParityType::None => Self::None,
            ParityType::Even => Self::Event,
            ParityType::Odd => Self::Odd,
        }
    }
}
impl TryFrom<usbd_serial::ParityType> for ParityType {
    type Error = ConvertError;
    fn try_from(value: usbd_serial::ParityType) -> Result<Self, Self::Error> {
        match value {
            usbd_serial::ParityType::None => Ok(Self::None),
            usbd_serial::ParityType::Event => Ok(Self::Even),
            usbd_serial::ParityType::Odd => Ok(Self::Odd),
            _ => Err(Self::Error::Incompatible),
        }
    }
}

impl From<StopBits> for usbd_serial::StopBits {
    fn from(value: StopBits) -> Self {
        match value {
            StopBits::One => Self::One,
            StopBits::Two => Self::Two,
        }
    }
}
impl TryFrom<usbd_serial::StopBits> for StopBits {
    type Error = ConvertError;
    fn try_from(value: usbd_serial::StopBits) -> Result<Self, Self::Error> {
        match value {
            usbd_serial::StopBits::One => Ok(Self::One),
            usbd_serial::StopBits::Two => Ok(Self::Two),
            _ => Err(Self::Error::Incompatible),
        }
    }
}

impl TryFrom<u8> for DataBits {
    type Error = ConvertError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            5 => Ok(Self::Five),
            6 => Ok(Self::Six),
            7 => Ok(Self::Seven),
            8 => Ok(Self::Eight),
            _ => Err(Self::Error::Incompatible),
        }
    }
}
impl TryFrom<&usbd_serial::LineCoding> for ConfigDTO {
    type Error = ConvertError;
    fn try_from(value: &usbd_serial::LineCoding) -> Result<Self, Self::Error> {
        let data_bits = value.data_bits().try_into()?;
        let stop_bits = value.stop_bits().try_into()?;
        let parity = value.parity_type().try_into()?;
        let baudrate = value.data_rate();
        Ok(Self {
            data_bits,
            stop_bits,
            parity,
            baudrate,
        })
    }
}
