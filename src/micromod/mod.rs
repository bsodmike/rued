use anyhow::Result;
use esp_idf_hal::prelude::Peripherals;

#[cfg(esp32)]
pub mod chip {
    use esp_idf_hal::gpio::{Gpio21, Gpio22, InputOutput, Output};

    pub type GpioSda = Gpio21<InputOutput>;
    pub type GpioScl = Gpio22<Output>;
}

pub mod i2c {
    use anyhow::Result;
    use esp_idf_hal::i2c::{self as HalI2c, Master, I2C0};
    use esp_idf_hal::units::FromValueType;

    pub fn configure<T, U>(
        i2c0: HalI2c::I2C0,
        scl: T,
        sda: U,
    ) -> Result<Master<I2C0, U, T>, crate::error::BlanketError>
    where
        T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
        U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
    {
        let i2c = Master::new(
            i2c0,
            HalI2c::MasterPins {
                scl, // O
                sda, // I+O
            },
            HalI2c::config::MasterConfig::new().baudrate(400.kHz().into()),
        )?;

        Ok(i2c)
    }
}

pub enum BoardSKU {
    WRL16781,
}

#[derive(PartialEq)]
pub enum Chip {
    ESP32,
}

pub trait Board<CHIP, SKU> {
    type Processor;
    type BoardSKU;

    fn gpio_sda(&self) -> Result<&chip::GpioSda>;
}

pub struct MicroModMCU {}

pub struct MicroModBoard<CHIP, SKU> {
    _chip: CHIP,
    _sku: SKU,
    sda: chip::GpioSda,
    scl: chip::GpioScl,
}

impl<CHIP, SKU> Board<CHIP, SKU> for MicroModBoard<CHIP, SKU>
where
    CHIP: 'static + std::cmp::PartialEq<super::micromod::Chip>,
{
    type BoardSKU = BoardSKU;
    type Processor = Chip;

    fn gpio_sda(&self) -> Result<&chip::GpioSda> {
        Ok(&self.sda)
    }
}

impl<CHIP, SKU> MicroModBoard<CHIP, SKU> {
    pub fn new(_chip: CHIP, _sku: SKU) -> Result<Self> {
        let (sda, scl) = setup_peripherals()?;

        Ok(Self {
            _chip,
            _sku,
            sda,
            scl,
        })
    }
}

fn setup_peripherals() -> Result<(chip::GpioSda, chip::GpioScl)> {
    let peripherals = Peripherals::take().unwrap();
    // SDA - GPIO pin 21, pad 12 on the MicroMod
    let sda = peripherals.pins.gpio21.into_input_output().unwrap();
    // SCL - GPIO pin 22, pad 14 on the MicroMod
    let scl = peripherals.pins.gpio22.into_output().unwrap();

    Ok((sda, scl))
}

#[cfg(test)]
mod test {
    use super::*;

    fn instantiate_chip() {
        let chip = Chip::ESP32;
        let sku = BoardSKU::WRL16781;

        let board: MicroModBoard<Chip, BoardSKU> = MicroModBoard::new(chip, sku).unwrap();

        board.gpio_sda().unwrap();
    }
}
