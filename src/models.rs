use crate::sensors::rtc;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike};
use log::{debug, info, warn};
use std::{env, fmt, ptr, sync::Mutex};

#[derive(Debug)]
pub struct RTClock {
    datetime: Option<DateTime<Utc>>,
}

impl RTClock {
    pub fn new() -> Self {
        Self { datetime: None }
    }

    pub fn update_time(
        &mut self,
        rtc: &mut crate::sensors::rtc::rv8803::RV8803,
    ) -> Result<RTCReading> {
        let mut time = [0_u8; rtc::rv8803::TIME_ARRAY_LENGTH];

        // Fetch time from RTC.
        let update = rtc.update_time(&mut time)?;
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
        let weekday: crate::rtc::rv8803::Weekday = self.weekday.into();

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

    pub fn weekday(&self) -> Result<rtc::rv8803::Weekday> {
        let rtc_weekday: rtc::rv8803::Weekday = self.datetime.weekday().into();

        Ok(rtc_weekday)
    }

    pub fn datetime(&self) -> Result<DateTime<Utc>> {
        Ok(self.datetime)
    }
}
