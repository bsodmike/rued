use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike};
use esp_idf_hal::i2c::{I2cDriver, I2cError};
use esp_idf_sys::settimeofday;
use esp_idf_sys::timeval;
use esp_idf_sys::timezone;
use log::{debug, info, warn};
use rv8803_rs::{i2c0::Bus as I2cBus, Rv8803, TIME_ARRAY_LENGTH};
use serde::Deserialize;
use serde::Serialize;
use shared_bus::{BusManager, I2cProxy};
use std::{env, error::Error as StdError, fmt, marker::PhantomData, ptr, sync::Mutex};

use crate::CURRENT_YEAR;

use self::rtc_external::get_system_time;
use self::rtc_external::prelude::*;
use self::rtc_external::Weekday;

pub struct RTClock<'a, I2C> {
    datetime: Option<DateTime<Utc>>,
    bus_manager: &'a BusManager<Mutex<I2cDriver<'a>>>,
    phantom: PhantomData<Rv8803<I2cBus<I2C>>>,
}

impl<'a, I2C> RTClock<'a, I2C>
where
    I2C: embedded_hal_0_2::blocking::i2c::Write<Error = I2cError>
        + embedded_hal_0_2::blocking::i2c::WriteRead<Error = I2cError>,
{
    pub fn new(bus_manager: &'a BusManager<Mutex<I2cDriver<'a>>>) -> Result<Self> {
        let address = rv8803_rs::i2c0::Address::Default;
        let rtc = Rv8803::from_i2c0(bus_manager.acquire_i2c(), address)?;

        Ok(Self {
            datetime: None,
            bus_manager,
            phantom: PhantomData,
        })
    }

    pub fn rtc(&self) -> Result<Rv8803<I2cBus<I2cProxy<'_, std::sync::Mutex<I2cDriver<'a>>>>>> {
        let proxy = self.bus_manager.acquire_i2c();

        let bus = rv8803_rs::i2c0::Bus::new(proxy, rv8803_rs::i2c0::Address::Default);

        Ok(Rv8803::new(bus)?)
    }
}

impl<'a, I2C> RtcExternal for RTClock<'a, I2C>
where
    I2C: embedded_hal_0_2::blocking::i2c::Write<Error = I2cError>
        + embedded_hal_0_2::blocking::i2c::WriteRead<Error = I2cError>,
{
    type Reading = RTCReading;

    fn update_time(&mut self) -> Result<Self::Reading> {
        let mut time = [0_u8; TIME_ARRAY_LENGTH];

        // Fetch time from RTC.
        let update = self.rtc()?.update_time(&mut time)?;
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
            year: format!("20{}", time[7]).parse::<u32>().unwrap_or_default(),
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

    unsafe fn get_system_time_with_fallback(&mut self) -> Result<SystemTimeBuffer> {
        let rtc = self.rtc()?;
        let system_time = get_system_time()?;

        // debug!(
        //     "System time Year: ({} / {}) / Current Year {}",
        //     &system_time.year,
        //     &system_time.to_rfc3339()?,
        //     CURRENT_YEAR
        // );
        if system_time.year >= CURRENT_YEAR {
            // Update RTC clock from System time due to SNTP update

            // FIXME: Disabled as this is done via the callback flag.
            // update_rtc_from_local(rtc, &system_time)?;
        } else {
            // When SNTP is unavailable, this will be called once where the system
            // clock will be updated from RTC and the conditional above (that compares the years), will prevent this from being called further.
            //
            // It's better to re-establish a valid SNTP update.
            self.update_local_from_rtc(&system_time)?;
        }

        Ok(system_time)
    }

    unsafe fn update_local_from_rtc(&mut self, system_time: &SystemTimeBuffer) -> Result<bool> {
        // This should be from the RTC clock

        self.update_time()?;
        let dt = self.datetime()?;

        // If the System time has not been updated due to any SNTP failure,
        // update the System time from the RTC clock.
        let tz = timezone {
            tz_minuteswest: 0,
            tz_dsttime: 0,
        };

        let tv_sec = dt.timestamp();
        let tv_usec = dt.timestamp_subsec_micros() as i32;
        let tm = timeval { tv_sec, tv_usec };

        settimeofday(&tm, &tz);
        info!(
            "Updated System Clock from RTC time: {} / Timestamp: {} sec, {} usec",
            system_time.to_rfc3339()?,
            tv_sec,
            tv_usec,
        );

        Ok(true)
    }

    fn update_rtc_from_local(&self, system_time: &SystemTimeBuffer) -> Result<bool> {
        let mut rtc = self.rtc()?;
        let weekday = system_time.weekday()?;

        let resp = rtc.set_time(
            system_time.seconds,
            system_time.minutes,
            system_time.hours,
            weekday.value(),
            system_time.date,
            system_time.month,
            system_time.year,
        )?;

        Ok(resp)
    }

    fn datetime(&self) -> Result<DateTime<Utc>> {
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

impl<'a, I2C> self::rtc_external::private::Sealed for RTClock<'a, I2C>
where
    I2C: embedded_hal_0_2::blocking::i2c::Write<Error = I2cError>
        + embedded_hal_0_2::blocking::i2c::WriteRead<Error = I2cError>,
{
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

#[derive(Clone, PartialEq, Debug)]
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
            self.weekday().expect("Unwrapping weekday"),
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

#[derive(Clone, PartialEq, Debug)]
pub struct SntpSyncStatus {
    pub synced: bool,
    pub last_synced: String,
}

pub(crate) mod rtc_external {
    pub(crate) mod prelude {
        pub use super::RtcExternal;
    }

    use super::*;

    use crate::UTC_OFFSET_CHRONO;
    use chrono::{offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
    use esp_idf_sys::time_t;

    type I2cDriverType<'a> = I2cDriver<'a>;

    pub(crate) mod private {
        use anyhow::Result;
        pub trait Sealed {
            fn to_timestamp(&self) -> Result<i64>;
        }
    }
    pub trait RtcExternal: private::Sealed {
        type Reading;

        unsafe fn get_system_time_with_fallback(&mut self) -> Result<SystemTimeBuffer>;

        unsafe fn update_local_from_rtc(&mut self, system_time: &SystemTimeBuffer) -> Result<bool>;

        fn update_time(&mut self) -> Result<Self::Reading>;

        fn update_rtc_from_local(&self, system_time: &SystemTimeBuffer) -> Result<bool>;

        fn datetime(&self) -> Result<DateTime<Utc>>;
    }

    pub(crate) unsafe fn get_system_time() -> Result<SystemTimeBuffer> {
        let timer: *mut time_t = ptr::null_mut();
        let mut timestamp = esp_idf_sys::time(timer);

        // NOTE: Handle system time providing a wierdly large value,
        // > Friday, 1 January 2100 00:00:00
        // FIXME this should trigger an NTP update.
        if timestamp > 4102444800 {
            log::warn!("System timestamp: {}. This has now been reset to Thursday, 1 January 1970 00:00:00.", timestamp);

            timestamp = 0;
        }

        // FIXME this causes a crash?
        // let system_time = esp_idf_svc::systime::EspSystemTime {};
        // let timestamp = esp_idf_svc::systime::EspSystemTime::now(&system_time).as_secs();

        let naive_dt_opt = NaiveDateTime::from_timestamp_opt(timestamp, 0);
        let naivedatetime_utc = if let Some(value) = naive_dt_opt {
            value
        } else {
            return Err(Error::msg(
                "Unable to unwrap NaiveDateTime, when attempting to parse UNIX timestamp",
            ));
        };

        let datetime_utc = DateTime::<Utc>::from_utc(naivedatetime_utc, UTC_OFFSET_CHRONO);

        let buf = SystemTimeBuffer {
            date: datetime_utc.day() as u8,
            year: datetime_utc.year() as u16,
            hours: datetime_utc.hour() as u8,
            minutes: datetime_utc.minute() as u8,
            seconds: datetime_utc.second() as u8,
            month: datetime_utc.month() as u8,
            datetime: datetime_utc,
        };

        Ok(buf)
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
            write!(f, "{}", self.as_str())
        }
    }

    impl Weekday {
        pub fn value(&self) -> u8 {
            *self as u8
        }

        pub fn as_str(&self) -> &'static str {
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
        pub fn as_short(&self) -> Result<String> {
            let day = self.as_str();
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
}
