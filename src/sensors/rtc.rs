pub mod rv8803 {
    #![deny(unsafe_code)]

    use crate::error::BlanketError;
    use anyhow::Result;
    use embedded_hal::blocking::i2c;
    use log::{debug, info};
    use std::fmt;

    pub const TIME_ARRAY_LENGTH: usize = 8;
    const RV8803_ENABLE: bool = true;
    const RV8803_DISABLE: bool = false;

    /// RV-8803 device driver.
    /// Datasheet: <https://cdn.sparkfun.com/assets/1/2/4/2/3/RV-8803-C7_App-Manual.pdf>
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
        BlanketError: From<E>,
    {
        /// Create a new instance of the RV8803.
        pub fn new(i2c: I2C, address: DeviceAddr) -> Result<Self, BlanketError> {
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
            self.write_register(Register::Seconds, dec_to_bcd(sec))?;
            self.write_register(Register::Minutes, dec_to_bcd(min))?;
            self.write_register(Register::Hours, dec_to_bcd(hour))?;
            self.write_register(Register::Date, dec_to_bcd(date))?;
            self.write_register(Register::Month, dec_to_bcd(month))?;
            self.write_register(Register::Year, dec_to_bcd((year - 2000) as u8))?;
            self.write_register(Register::Weekday, weekday)?;

            //Set RESET bit to 0 after setting time to make sure seconds don't get stuck.
            self.write_bit(
                Register::Control.address(),
                Register::ControlReset.address(),
                RV8803_DISABLE,
            )?;

            debug!("rv8803::set_time: updated RTC clock");

            Ok(true)
        }

        pub fn update_time(&mut self, dest: &mut [u8]) -> Result<bool, crate::error::BlanketError> {
            if (self.read_multiple_registers(
                Register::Hundredths.address(),
                dest,
                TIME_ARRAY_LENGTH,
            )?) == false
            {
                info!("update_time: attempt read - fail 1");
                return Ok(false); // Something went wrong
            }

            // If hundredths are at 99 or seconds are at 59, read again to make sure we didn't accidentally skip a second/minute
            if bcd_to_dec(dest[0]) == 99 || bcd_to_dec(dest[1]) == 59 {
                let mut temp_time = [0_u8; TIME_ARRAY_LENGTH];

                info!("update_time: if hundredths are at 99 or seconds are at 59, read again to make sure we didn't accidentally skip a second/minute / Hundreths: {} / Seconds: {}", bcd_to_dec(dest[0]),bcd_to_dec(dest[1]));

                if (self.read_multiple_registers(
                    Register::Hundredths.address(),
                    &mut temp_time,
                    TIME_ARRAY_LENGTH,
                )?) == false
                {
                    info!("update_time: attempt read - fail 2");
                    return Ok(false); // Something went wrong
                };

                // If the reading for hundredths has rolled over, then our new data is correct, otherwise, we can leave the old data.
                if bcd_to_dec(dest[0]) > bcd_to_dec(temp_time[0]) {
                    info!("update_time: the reading for hundredths has rolled over, then our new data is correct. / Hundreths: {} / temp_time[0]: {}",
                    bcd_to_dec(dest[0]),
                    bcd_to_dec(temp_time[0]));

                    for (i, el) in temp_time.iter().enumerate() {
                        dest[i] = *el
                    }
                }
            }

            // byte order: https://github.com/sparkfun/SparkFun_RV-8803_Arduino_Library/blob/main/src/SparkFun_RV8803.h#L129-L138
            let mut buf = [0_u8; 8];
            for (i, el) in dest.iter().enumerate() {
                // Note: Weekday does not undergo BCD to Decimal conversion.
                if i != 4 {
                    // println!("Raw: {} / BCD to Dec: {}", *el, bcd_to_dec(*el));
                    buf[i] = bcd_to_dec(*el)
                } else {
                    buf[i] = *el
                }
            }

            std::io::copy(&mut &buf[0..buf.len()], &mut dest.as_mut())?;

            Ok(true)
        }

        pub fn write_bit(
            &mut self,
            reg_addr: u8,
            bit_addr: u8,
            bit_to_write: bool,
        ) -> Result<bool, E> {
            let mut value = 0;
            if let Ok(reg_value) = self.read_register_by_addr(reg_addr) {
                value = reg_value
            }

            value &= !(1 << bit_addr);
            value |= u8::from(bit_to_write) << bit_addr;

            self.write_register_by_addr(reg_addr, value)?;

            Ok(true)
        }

        pub fn read_multiple_registers(
            &mut self,
            addr: u8,
            dest: &mut [u8],
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

        fn write_register(&mut self, register: Register, value: u8) -> Result<(), E> {
            let byte = value as u8;
            self.i2c
                .write(self.address as u8, &[register.address(), byte])
        }

        fn write_register_by_addr(&mut self, reg_addr: u8, value: u8) -> Result<(), E> {
            let byte = value as u8;
            self.i2c.write(self.address as u8, &[reg_addr, byte])
        }

        fn read_register(&mut self, register: Register) -> Result<u8, E> {
            let mut data = [0];
            self.i2c
                .write_read(self.address as u8, &[register.address()], &mut data)?;
            Ok(u8::from_le_bytes(data))
        }

        fn read_register_by_addr(&mut self, reg_addr: u8) -> Result<u8, E> {
            let mut data = [0];
            self.i2c
                .write_read(self.address as u8, &[reg_addr], &mut data)?;
            Ok(u8::from_le_bytes(data))
        }
    }

    fn bcd_to_dec(value: u8) -> u8 {
        ((value / 0x10) * 10) + (value % 0x10)
    }

    fn dec_to_bcd(value: u8) -> u8 {
        ((value / 10) * 0x10) + (value % 10)
    }

    // Table 14.1
    #[derive(Clone, Copy)]
    pub enum Register {
        Hundredths = 0x10,
        Seconds = 0x11,
        Minutes = 0x12,
        Hours = 0x13,
        Weekday = 0x14,
        Date = 0x15,
        Month = 0x16,
        Year = 0x17,
        ControlReset = 0,
        Control = 0x1F,
    }

    impl Register {
        fn address(&self) -> u8 {
            *self as u8
        }
    }

    #[derive(Debug, Clone, Copy)]
    pub enum Weekday {
        Sunday = 1,
        Monday = 2,
        Tuesday = 4,
        Wednesday = 8,
        Thursday = 16,
        Friday = 32,
        Saturday = 64,
    }

    impl fmt::Display for Weekday {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(f, "{}", self.to_s())
        }
    }

    impl Weekday {
        pub fn value(&self) -> u8 {
            *self as u8
        }

        pub fn to_s(&self) -> String {
            match self {
                Weekday::Sunday => "Sunday".to_string(),
                Weekday::Monday => "Monday".to_string(),
                Weekday::Tuesday => "Tuesday".to_string(),
                Weekday::Wednesday => "Wednesday".to_string(),
                Weekday::Thursday => "Thursday".to_string(),
                Weekday::Friday => "Friday".to_string(),
                Weekday::Saturday => "Saturday".to_string(),
            }
        }
    }

    impl From<u8> for Weekday {
        fn from(day: u8) -> Self {
            match day {
                1 => Self::Sunday,
                2 => Self::Monday,
                4 => Self::Tuesday,
                8 => Self::Wednesday,
                16 => Self::Thursday,
                32 => Self::Friday,
                64 => Self::Saturday,
                _ => Self::Sunday,
            }
        }
    }

    impl From<chrono::Weekday> for Weekday {
        fn from(day: chrono::Weekday) -> Self {
            match day {
                chrono::Weekday::Sun => Self::Sunday,
                chrono::Weekday::Mon => Self::Monday,
                chrono::Weekday::Tue => Self::Tuesday,
                chrono::Weekday::Wed => Self::Wednesday,
                chrono::Weekday::Thu => Self::Thursday,
                chrono::Weekday::Fri => Self::Friday,
                chrono::Weekday::Sat => Self::Saturday,
            }
        }
    }
}
