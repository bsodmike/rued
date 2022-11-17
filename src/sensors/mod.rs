pub mod rtc;

// pub mod i2c {
//     use anyhow::Result;
//     use esp_idf_hal::i2c::{self as HalI2c, Master, I2C0};
//     use esp_idf_hal::units::FromValueType;
//     use std::error::Error as StdError;
//     use std::fmt;

//     pub type BoxError = Box<dyn std::error::Error + Send + Sync>;

//     #[derive(Debug)]
//     pub struct BlanketError {
//         inner: BoxError,
//     }

//     #[allow(dead_code)]
//     impl BlanketError {
//         /// Create a new `Error` from a boxable error.
//         pub fn new(error: impl Into<BoxError>) -> Self {
//             Self {
//                 inner: error.into(),
//             }
//         }

//         /// Convert an `Error` back into the underlying boxed trait object.
//         pub fn into_inner(self) -> BoxError {
//             self.inner
//         }
//     }

//     impl fmt::Display for BlanketError {
//         fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
//             self.inner.fmt(f)
//         }
//     }

//     impl StdError for BlanketError {
//         fn source(&self) -> Option<&(dyn StdError + 'static)> {
//             Some(&*self.inner)
//         }
//     }

//     pub fn configure<T, U>(
//         i2c0: HalI2c::I2C0,
//         scl: T,
//         sda: U,
//     ) -> Result<Master<I2C0, U, T>, crate::error::BlanketError>
//     where
//         T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
//         U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
//     {
//         let i2c = Master::new(
//             i2c0,
//             HalI2c::MasterPins {
//                 scl, // O
//                 sda, // I+O
//             },
//             HalI2c::config::MasterConfig::new().baudrate(400.kHz().into()),
//         )?;

//         Ok(i2c)
//     }
// }
