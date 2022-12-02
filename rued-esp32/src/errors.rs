use std::error::Error as StdError;
use std::fmt;
use std::io::Error as IoError;

use edge_executor::SpawnError;
use esp_idf_hal::i2c::I2cError;
use esp_idf_svc::errors::EspIOError;
use esp_idf_sys::EspError;

pub type BoxError = Box<dyn std::error::Error + Send + Sync>;

#[allow(clippy::enum_variant_names)]
#[derive(Debug)]
pub enum InitError {
    AnyhowError(anyhow::Error),
    EspError(EspError),
    SpawnError(SpawnError),
    IoError(IoError),
}

impl From<EspError> for InitError {
    fn from(e: EspError) -> Self {
        Self::EspError(e)
    }
}

impl From<EspIOError> for InitError {
    fn from(e: EspIOError) -> Self {
        Self::EspError(e.0)
    }
}

impl From<SpawnError> for InitError {
    fn from(e: SpawnError) -> Self {
        Self::SpawnError(e)
    }
}

impl From<anyhow::Error> for InitError {
    fn from(e: anyhow::Error) -> Self {
        Self::AnyhowError(e)
    }
}

impl From<IoError> for InitError {
    fn from(e: IoError) -> Self {
        Self::IoError(e)
    }
}

// OTHERS

#[derive(Debug)]
pub struct BlanketError {
    inner: BoxError,
}

impl BlanketError {
    /// Create a new `Error` from a boxable error.
    pub fn new(error: impl Into<BoxError>) -> Self {
        Self {
            inner: error.into(),
        }
    }

    #[allow(dead_code)]
    /// Convert an `Error` back into the underlying boxed trait object.
    pub fn into_inner(self) -> BoxError {
        self.inner
    }
}

impl fmt::Display for BlanketError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.inner.fmt(f)
    }
}

impl StdError for BlanketError {
    fn source(&self) -> Option<&(dyn StdError + 'static)> {
        Some(&*self.inner)
    }
}

impl From<esp_idf_sys::EspError> for BlanketError {
    fn from(error: esp_idf_sys::EspError) -> Self {
        Self {
            inner: error.into(),
        }
    }
}

impl From<std::io::Error> for BlanketError {
    fn from(error: std::io::Error) -> Self {
        Self {
            inner: error.into(),
        }
    }
}

impl From<I2cError> for BlanketError {
    fn from(error: I2cError) -> Self {
        Self {
            inner: error.into(),
        }
    }
}

impl From<anyhow::Error> for BlanketError {
    fn from(error: anyhow::Error) -> Self {
        Self {
            inner: error.into(),
        }
    }
}

// CustomError
// NOTE: This is temporarily being stashed away here, if I need a template for a NewType wrapper.
#[derive(Debug)]
pub struct CustomError(Option<BoxError>);

impl CustomError {
    /// Create a new `Error` from a boxable error.
    pub fn new() -> Self {
        Self(None)
    }

    pub fn set_error(&mut self, error: impl Into<BoxError>) {
        self.0 = Some(error.into());
    }

    #[allow(dead_code)]
    /// Convert an `Error` back into the underlying boxed trait object.
    pub fn into_inner(self) -> BoxError {
        self.0.expect("Unwrapping option for CustomError")
    }
}

impl fmt::Display for CustomError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        self.0
            .as_ref()
            .expect("Unwrapping option for CustomError")
            .fmt(f)
    }
}

impl StdError for CustomError {
    fn source(&self) -> Option<&(dyn StdError + 'static)> {
        Some(
            self.0
                .as_deref()
                .expect("Unwrapping option for CustomError"),
        )
    }
}

impl From<anyhow::Error> for CustomError {
    fn from(error: anyhow::Error) -> Self {
        let mut e = Self::new();
        e.set_error(error);

        e
    }
}

impl From<CustomError> for BlanketError {
    fn from(error: CustomError) -> Self {
        Self::new(error)
    }
}
