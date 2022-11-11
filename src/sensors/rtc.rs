use anyhow::Result;
use esp_idf_hal::i2c::{self, Master, I2C0};
use esp_idf_hal::units::FromValueType;

pub fn setup<E, T, U>(master: Master<I2C0, U, T>) -> Result<Master<I2C0, U, T>>
where
    T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
    U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
    E: std::fmt::Debug,
{
    let i2c = master;

    Ok(i2c)
}
