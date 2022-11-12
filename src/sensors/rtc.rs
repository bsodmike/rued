use anyhow::{Error, Result};
use esp_idf_hal::i2c::{self, Master, I2C0};
use esp_idf_hal::units::FromValueType;

// pub fn setup<E, T, U>(master: Master<I2C0, U, T>) -> Result<Master<I2C0, U, T>>
// where
//     T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
//     U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
//     E: std::fmt::Debug,
// {
//     let i2c = master;

//     Ok(i2c)
// }

pub mod rv8803 {
    #![deny(unsafe_code)]

    use std::num::TryFromIntError;

    use anyhow::{Error, Result};
    use embedded_hal::blocking::i2c;
    use esp_idf_hal::i2c::I2cError;
    use esp_idf_sys::EspError;

    pub const TIME_ARRAY_LENGTH: usize = 8;

    /// RV-8803 device driver.
    /// Datasheet: https://cdn.sparkfun.com/assets/1/2/4/2/3/RV-8803-C7_App-Manual.pdf
    ///
    #[derive(Debug)]
    pub struct RV8803<I2C> {
        /// The concrete I²C device implementation.
        i2c: I2C,

        /// Device address
        address: DeviceAddr,
    }

    /// see Table 3.3.2
    #[derive(Debug, Clone, Copy, PartialEq)]
    pub enum DeviceAddr {
        B011_0010 = 0x32,
        B110_0101 = 0b110_0101, // Read
        B110_0100 = 0b110_0100, // Write
    }

    #[derive(Debug)]
    pub struct RTCTime {
        sec: u8,
        min: u8,
        hour: u8,
        weekday: u8,
        date: u8,
        month: u8,
        year: u16,
    }

    impl<I2C, E> RV8803<I2C>
    where
        I2C: i2c::WriteRead<Error = E> + i2c::Write<Error = E>,
    {
        /// Create a new instance of the RV8803.
        pub fn new(i2c: I2C, address: DeviceAddr) -> Result<Self, E> {
            let rv8803 = RV8803 { i2c, address };

            Ok(rv8803)
        }

        pub fn set_time(
            &mut self,
            sec: u8,
            min: u8,
            hour: u8,
            weekday: u8,
            date: u8,
            month: u8,
            year: u16,
        ) -> Result<bool, E> {
            self.write_register(Register::Seconds, sec)?;
            self.write_register(Register::Minutes, min)?;
            self.write_register(Register::Hours, hour)?;
            self.write_register(Register::Weekday, 1 << weekday)?;
            self.write_register(Register::Date, date)?;
            self.write_register(Register::Month, month)?;
            self.write_register(Register::Year, (year - 2000) as u8)?;

            Ok(true)
        }

        pub fn update_time(&mut self, dest: &mut [u8]) -> Result<bool, E> {
            if (self.read_multiple_registers(
                Register::Hundredths.address(),
                dest,
                TIME_ARRAY_LENGTH,
            )?) == false
            {
                return Ok(false); // Something went wrong
            };

            // If hundredths are at 99 or seconds are at 59, read again to make sure we didn't accidentally skip a second/minute
            if (bcd_to_dec(dest[0]) == 99 || bcd_to_dec(dest[1]) == 59) {
                let mut temp_time = [0_u8; TIME_ARRAY_LENGTH];

                if (self.read_multiple_registers(
                    Register::Hundredths.address(),
                    &mut temp_time,
                    TIME_ARRAY_LENGTH,
                )?) == false
                {
                    return Ok(false); // Something went wrong
                };

                if (bcd_to_dec(dest[0]) > bcd_to_dec(temp_time[0]))
                // If the reading for hundredths has rolled over, then our new data is correct, otherwise, we can leave the old data.
                {
                    for (i, el) in temp_time.iter().enumerate() {
                        dest[i] = *el
                    }
                }
            }

            // return true;
            Ok(true)
        }

        pub fn read_multiple_registers(
            &mut self,
            addr: u8,
            mut dest: &mut [u8],
            len: usize,
        ) -> Result<bool, E> {
            self.i2c.write_read(self.address as u8, &[addr], dest)?;

            Ok(true)
        }

        pub fn read_seconds(&mut self) -> Result<u8, E> {
            let secs = self.read_register(Register::Seconds)?;

            Ok(bcd_to_dec(secs))
        }

        pub fn read_year(&mut self) -> Result<u8, E> {
            let year = self.read_register(Register::Year)?;

            Ok(bcd_to_dec(year))
        }

        pub fn set_year(&mut self) -> Result<u8, E> {
            let year = dec_to_bcd(22);

            self.write_register(Register::Year, year)?;

            self.read_year()
        }

        /// Read PwrMgmt0 configuration
        pub fn read_pwr_configuration(&mut self) -> Result<PowerManagement, E> {
            let bits = self.read_register(Register::PwrMgmt0)?;
            Ok(PowerManagement { bits })
        }

        /// Write in PwrMgmt0 Register
        fn write_pwr_mgmt(&mut self, value: u8) -> Result<(), E> {
            self.write_register(Register::PwrMgmt0, value)
        }

        fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
            let byte = value as u8;
            self.i2c
                .write(self.address as u8, &[register.address(), byte])
        }

        fn read_register(&mut self, register: Register) -> Result<u8, E> {
            let mut data = [0];
            self.i2c
                .write_read(self.address as u8, &[register.address()], &mut data)?;
            Ok(u8::from_le_bytes(data))
        }
    }

    fn bcd_to_dec(value: u8) -> u8 {
        ((value / 0x10) * 10) + (value % 0x10)
    }

    fn dec_to_bcd(value: u8) -> u8 {
        ((value / 10) * 0x10) + (value % 10)
    }

    pub struct PowerManagement {
        pub bits: u8,
    }

    // Table 14.1
    #[derive(Clone, Copy)]
    pub enum Register {
        PwrMgmt0 = 0x1F,
        Hundredths = 0x10,
        Seconds = 0x11,
        Minutes = 0x12,
        Hours = 0x13,
        Weekday = 0x14,
        Date = 0x15,
        Month = 0x16,
        Year = 0x17,
    }

    impl Register {
        fn address(&self) -> u8 {
            *self as u8
        }
    }
}
