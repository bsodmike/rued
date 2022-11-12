pub mod rtc;

pub mod i2c {
    use anyhow::Result;
    use esp_idf_hal::gpio::{InputPin, OutputPin};
    use esp_idf_hal::i2c::{self as HalI2c, config::MasterConfig, I2c, Master, MasterPins, I2C0};
    use esp_idf_hal::units::FromValueType;
    use esp_idf_sys::{
        esp, i2c_config_t, i2c_config_t__bindgen_ty_1, i2c_config_t__bindgen_ty_1__bindgen_ty_1,
        i2c_mode_t_I2C_MODE_MASTER, EspError, ESP_ERR_INVALID_ARG,
    };
    use std::error::Error as StdError;
    use std::fmt;

    // pub struct I2CMaster<I2C, SDA, SCL>(Master<I2C, SDA, SCL>)
    // where
    //     I2C: I2c,
    //     SDA: OutputPin + InputPin,
    //     SCL: OutputPin + InputPin;

    // impl<I2C, SDA, SCL> I2CMaster<I2C, SDA, SCL>
    // where
    //     I2C: I2c,
    //     SDA: OutputPin + InputPin,
    //     SCL: OutputPin + InputPin,
    // {
    //     pub fn new(
    //         i2c: I2C,
    //         pins: MasterPins<SDA, SCL>,
    //         config: MasterConfig,
    //     ) -> Result<I2CMaster<I2C, SDA, SCL>, EspError> {
    //         // i2c_config_t documentation says that clock speed must be no higher than 1 MHz
    //         if config.baudrate > 1.MHz().into() {
    //             return Err(EspError::from(ESP_ERR_INVALID_ARG as i32).unwrap());
    //         }

    //         let sys_config = i2c_config_t {
    //             mode: i2c_mode_t_I2C_MODE_MASTER,
    //             sda_io_num: pins.sda.pin(),
    //             sda_pullup_en: config.sda_pullup_enabled,
    //             scl_io_num: pins.scl.pin(),
    //             scl_pullup_en: config.scl_pullup_enabled,
    //             __bindgen_anon_1: i2c_config_t__bindgen_ty_1 {
    //                 master: i2c_config_t__bindgen_ty_1__bindgen_ty_1 {
    //                     clk_speed: config.baudrate.into(),
    //                 },
    //             },
    //             ..Default::default()
    //         };

    //         esp!(unsafe { i2c_param_config(I2C::port(), &sys_config) })?;

    //         esp!(unsafe {
    //             i2c_driver_install(
    //                 I2C::port(),
    //                 i2c_mode_t_I2C_MODE_MASTER,
    //                 0, // Not used in master mode
    //                 0, // Not used in master mode
    //                 0,
    //             ) // TODO: set flags
    //         })?;

    //         Ok(I2CMaster {
    //             i2c,
    //             pins,
    //             timeout: TickType::from(config.timeout).0,
    //         })
    //     }
    // }

    pub type BoxError = Box<dyn std::error::Error + Send + Sync>;

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

    pub fn configure<E, T, U>(
        i2c0: HalI2c::I2C0,
        scl: T,
        sda: U,
    ) -> Result<Master<I2C0, U, T>, crate::BlanketError>
    where
        T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
        U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
        E: std::fmt::Debug,
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
