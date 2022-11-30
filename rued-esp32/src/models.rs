use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike};
use esp_idf_hal::i2c::I2cError;
use log::{debug, info, warn};
use rv8803_rs::{i2c0::Bus as I2cBus, Rv8803, TIME_ARRAY_LENGTH};
use std::{env, error::Error as StdError, fmt, ptr, sync::Mutex};

#[derive(Debug)]
pub struct RTClock<I2C> {
    datetime: Option<DateTime<Utc>>,
    rtc: Rv8803<I2cBus<I2C>>,
}

impl<I2C> RTClock<I2C>
where
    I2C: embedded_hal_0_2::blocking::i2c::Write<Error = I2cError>
        + embedded_hal_0_2::blocking::i2c::WriteRead<Error = I2cError>,
{
    pub fn new(i2c: I2C) -> Result<Self> {
        let address = rv8803_rs::i2c0::Address::Default;
        let rtc = Rv8803::from_i2c0(i2c, address).unwrap();
        // .expect("RTC module instantiated with I2c bus");

        Ok(Self {
            datetime: None,
            rtc,
        })
    }

    pub fn update_time(&mut self) -> Result<RTCReading> {
        let mut time = [0_u8; TIME_ARRAY_LENGTH];

        // Fetch time from RTC.
        let update = self.rtc.update_time(&mut time)?;
        if !update {
            warn!("RTC: Failed reading latest time");
            println!("{:?}", time);
        }

        let reading = RTCReading {
            hours: time[3],
            minutes: time[2],
            seconds: time[1],
            hundreths: time[0],
            weekday: time[4],
            date: time[5],
            month: time[6],
            year: format!("20{}", time[7])
                .to_string()
                .parse::<u32>()
                .unwrap_or_default(),
        };

        let naivedatetime_utc = NaiveDate::from_ymd_opt(
            reading.year as i32,
            reading.month as u32,
            reading.date as u32,
        )
        .unwrap()
        .and_hms_opt(
            reading.hours as u32,
            reading.minutes as u32,
            reading.seconds as u32,
        )
        .unwrap();
        let datetime_utc = DateTime::<Utc>::from_utc(naivedatetime_utc, crate::UTC_OFFSET_CHRONO);
        self.datetime = Some(datetime_utc);

        Ok(reading)
    }

    fn to_timestamp(&self) -> Result<i64> {
        let datetime = if let Some(dt) = self.datetime {
            dt
        } else {
            return Err(Error::msg(
                "Unable to unwrap datetime, when attempting to return UNIX timestamp",
            ));
        };

        Ok(datetime.timestamp())
    }

    pub fn datetime(&self) -> Result<DateTime<Utc>> {
        let datetime = if let Some(dt) = self.datetime {
            dt
        } else {
            return Err(Error::msg(
                "Unable to unwrap datetime, when attempting to return UNIX timestamp",
            ));
        };

        Ok(datetime)
    }
}

#[derive(Debug)]
pub struct RTCReading {
    hours: u8,
    minutes: u8,
    seconds: u8,
    hundreths: u8,
    weekday: u8,
    date: u8,
    month: u8,
    year: u32,
}

impl RTCReading {
    pub fn to_s(&self) -> Result<String> {
        let weekday: Weekday = self.weekday.into();

        Ok(format!(
            "{} {}-{:0>2}-{:0>2} {:0>2}:{:0>2}:{:0>2} {}",
            weekday,
            self.year,
            self.month,
            self.date,
            self.hours,
            self.minutes,
            self.seconds,
            crate::UTC_OFFSET_CHRONO
        ))
    }
}

#[derive(Debug)]
pub struct SystemTimeBuffer {
    pub date: u8,
    pub month: u8,
    pub year: u16,
    pub hours: u8,
    pub minutes: u8,
    pub seconds: u8,
    pub datetime: DateTime<Utc>,
}

impl SystemTimeBuffer {
    pub fn to_s(&self) -> Result<String> {
        Ok(format!(
            "{} {}-{:0>2}-{:0>2} {:0>2}:{:0>2}:{:0>2} {}",
            self.weekday()?,
            self.year,
            self.month,
            self.date,
            self.hours,
            self.minutes,
            self.seconds,
            crate::UTC_OFFSET_CHRONO
        ))
    }

    fn to_timestamp(&self) -> Result<i64> {
        Ok(self.datetime.timestamp())
    }

    pub fn to_rfc3339(&self) -> Result<String> {
        Ok(self.datetime.to_rfc3339())
    }

    pub fn weekday(&self) -> Result<Weekday> {
        let rtc_weekday: Weekday = self.datetime.weekday().into();

        Ok(rtc_weekday)
    }

    pub fn datetime(&self) -> Result<DateTime<Utc>> {
        Ok(self.datetime)
    }
}

// RTC Extra
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
        write!(f, "{}", self.to_str())
    }
}

impl Weekday {
    pub fn value(&self) -> u8 {
        *self as u8
    }

    pub fn to_str(&self) -> &'static str {
        match self {
            Self::Sunday => "Sunday",
            Self::Monday => "Monday",
            Self::Tuesday => "Tuesday",
            Self::Wednesday => "Wednesday",
            Self::Thursday => "Thursday",
            Self::Friday => "Friday",
            Self::Saturday => "Saturday",
        }
    }

    // Returns the first 3-letters of the day of the week
    pub fn to_short(&self) -> Result<String> {
        let day = self.to_str();
        let result: String = day.chars().take(3).collect();

        Ok(result)
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
