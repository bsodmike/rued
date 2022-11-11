pub mod rtc;

pub mod i2c {
    use anyhow::Result;
    use esp_idf_hal::i2c::{self as HalI2c, Master, I2C0};
    use esp_idf_hal::units::FromValueType;

    pub fn configure<E, T, U>(i2c0: HalI2c::I2C0, scl: T, sda: U) -> Result<Master<I2C0, U, T>>
    where
        T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
        U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
        E: std::fmt::Debug,
    {
        let i2c = HalI2c::Master::new(
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
