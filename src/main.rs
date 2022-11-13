#![feature(const_btree_new)]

use core::time::Duration;
use cstr_core::CString;
use std::os::unix::raw::time_t;
use std::{ptr, time::*};

use crate::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
use crate::sensors::rtc;
use crate::sensors::rtc::rv8803::Weekday;
#[allow(unused_imports)]
use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output};
use esp_idf_hal::i2c::{self, I2cError, Master, I2C0};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::timer::EspTimerService;
use esp_idf_sys::{
    self as _, sntp_init, sntp_set_sync_interval, sntp_set_time_sync_notification_cb, sntp_stop,
    timeval,
}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

// Time stuff
use embedded_svc::sys_time::SystemTime;
use esp_idf_svc::systime::EspSystemTime;

use time::macros::offset;
use time::{Date, OffsetDateTime, Time, UtcOffset};

use esp_idf_svc::sntp::{self, EspSntp};
use esp_idf_svc::sntp::{OperatingMode, SntpConf, SyncMode, SyncStatus};

use anyhow::{Error, Result};
use log::{debug, info, warn};
use shared_bus::{I2cProxy, NullMutex};
use std::error::Error as StdError;
use std::fmt;

mod display;
mod http;
mod http_client;
mod http_server;
mod sensors;
mod wifi;

const SSID: &str = "foo"; // env!("SSID");
const PASSWORD: &str = "foo"; // env!("PASSWORD");

type GpioSda = Gpio21<InputOutput>;
type GpioScl = Gpio22<Output>;

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
    fn to_s(&self) -> Result<String> {
        let date_str = format!("{}-{}-{}", self.date, self.month, self.year);
        let time_str = format!("{}:{}:{}", self.hours, self.minutes, self.seconds);

        Ok(format!("RTC Clock: {} @ {}", time_str, date_str))
    }
}

#[derive(Debug)]
pub struct SystemTimeBuffer {
    date: u8,
    month: u8,
    year: u16,
    hours: u8,
    minutes: u8,
    seconds: u8,
    full_date: Date,
    full_time: Time,
}

impl SystemTimeBuffer {
    fn to_s(&self) -> Result<String> {
        let date_str = format!("{:0>2}-{:0>2}-{}", self.date, self.month, self.year);
        let time_str = format!(
            "{:0>2}:{:0>2}:{:0>2}",
            self.hours, self.minutes, self.seconds
        );

        Ok(format!("System Clock: {} @ {}", time_str, date_str))
    }

    fn to_rfc3339(&self) -> Result<String> {
        todo!();
    }

    fn weekday(&self) -> Result<time::Weekday> {
        Ok(self.full_date.weekday())
    }
}

const UTC_OFFSET: UtcOffset = UtcOffset::UTC;

pub unsafe extern "C" fn sntp_set_time_sync_notification_cb_custom(tv: *mut timeval) {
    info!(
        r#"

        ->      SNTP Sync Callback called: sec: {}, usec: {}

        "#,
        (*tv).tv_sec,
        (*tv).tv_usec,
    );

    let err_value: i64 = 0;
    let offset = match OffsetDateTime::from_unix_timestamp((*tv).tv_sec as i64) {
        Ok(value) => value,
        Err(_) => OffsetDateTime::from_unix_timestamp(err_value).unwrap(),
    };

    let utc_with_offset = offset.to_offset(UTC_OFFSET);
    let actual_time = utc_with_offset.time();
    let actual_date = utc_with_offset.date();
    let offset = utc_with_offset.offset();

    let (hours, mins, secs) = actual_time.as_hms();
    let (year, month, day) = actual_date.to_calendar_date();

    if year == 1970 {
        // Do not update RTC.
    } else {
        // Update RTC

        let date_str = format!("{}-{}-{}", day, month, year);
        let time_str = format!("{}:{}:{}", hours, mins, secs);
        info!(
            r#"
                
        ->      SNTP Sync: {} @ {}
                
            "#,
            time_str, date_str
        );
    }
}

fn main() -> Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();
    EspLogger::initialize_default();

    info!("Hello, Rust from an ESP32!");
    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    //  configure peripherals
    let peripherals = Peripherals::take().unwrap();
    // let mut led = peripherals.pins.gpio17.into_output().unwrap();
    // let mut led_onboard = peripherals.pins.gpio18.into_output().unwrap();

    // SDA - GPIO pin 21, pad 12 on the MicroMod
    let sda = peripherals.pins.gpio21.into_input_output().unwrap();
    // SCL - GPIO pin 22, pad 14 on the MicroMod
    let scl = peripherals.pins.gpio22.into_output().unwrap();

    // D0 - GPIO pin 14, pad 10 on the MicroMod
    let gpio_d0: Gpio14<Output> = peripherals.pins.gpio14.into_output().unwrap();
    let mut led_onboard = gpio_d0;
    // D1 - GPIO pin 27, pad 18 on the MicroMod
    let _gpio_d1: Gpio27<Output> = peripherals.pins.gpio27.into_output().unwrap();

    // wifi
    let wifi = wifi::connect();
    let ip = match &wifi {
        Err(e) => {
            println!("Wifi error: {:?}", e);
            format!("ERR: {:?}", e)
        }
        Ok(s) => s.1.ip.to_string(),
    };

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    let dns = match &wifi {
        Err(e) => {
            // println!("Wifi error: {:?}", e);
            format!("ERR: {:?}", e)
        }
        Ok(s) => {
            if let Some(value) = s.1.dns {
                value.to_string()
            } else {
                format!("ERR: Unable to unwrap DNS value")
            }
        }
    };

    let _wifi_client = wifi?.0;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    // setup http server
    // let server_config = HttpServerConfiguration::default();
    // let mut server = EspHttpServer::new(&server_config)?;
    // let _resp = http_server::configure_handlers(&mut server)?;

    // setup I2C Master
    let i2c_master = sensors::i2c::configure::<GpioScl, GpioSda>(peripherals.i2c0, scl, sda)?;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    // Instantiate the bus manager, pass the i2c bus.
    let bus = shared_bus::BusManagerSimple::new(i2c_master);

    // Create two proxies. Now, each sensor can have their own instance of a proxy i2c, which resolves the ownership problem.
    let proxy_1 = bus.acquire_i2c();
    let proxy_2 = bus.acquire_i2c();

    info!("Reading RTC Sensor");

    // setup RTC sensor
    let mut rtc =
        sensors::rtc::rv8803::RV8803::new(proxy_1, sensors::rtc::rv8803::DeviceAddr::B011_0010)?;

    // rtc.set_time(
    //     0,
    //     48,
    //     3,
    //     rtc::rv8803::Weekday::Saturday.value(),
    //     13,
    //     11,
    //     2022,
    // )?;

    let mut _time = [0_u8; rtc::rv8803::TIME_ARRAY_LENGTH];

    // // setup display
    // if let Err(e) = display::display_test::<Error, GpioScl, GpioSda>(i2c_master, &ip, &dns) {
    //     println!("Display error: {:?}", e)
    // } else {
    //     println!("Display ok");
    // }

    // heart-beat sequence
    // for i in 0..3 {
    //     println!("Toggling LED now: {}", i);
    //     toggle_led::<anyhow::Error, Gpio14<Output>>(&mut led_onboard);
    // }

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    unsafe {
        let sntp = sntp_setup()?;
        let latest_system_time = get_system_time()?;

        let weekday: time::Weekday = latest_system_time.weekday()?;
        let rtc_weekday: rtc::rv8803::Weekday = weekday.into();

        rtc.set_time(
            latest_system_time.seconds,
            latest_system_time.minutes,
            latest_system_time.hours,
            rtc_weekday.value(),
            latest_system_time.date,
            latest_system_time.month,
            latest_system_time.year,
        )?;

        loop {
            toggle_led::<anyhow::Error, Gpio14<Output>>(&mut led_onboard);

            std::thread::sleep(Duration::from_millis(500));

            // Fetch time from RTC.
            let update = rtc.update_time(&mut _time)?;
            if !update {
                warn!("RTC: Failed reading latest time");
                println!("{:?}", _time);
            }
            let weekday: Weekday = _time[4].into();

            // FIXME: wierd bug where hundreth's value does not update inside this loop, without the use of debugging output
            println!("_time: {:?}", &_time);

            let reading = RTCReading {
                hours: _time[3],
                minutes: _time[2],
                seconds: _time[1],
                hundreths: _time[0],
                weekday: _time[4],
                date: _time[5],
                month: _time[6],
                year: format!("20{}", _time[7])
                    .to_string()
                    .parse::<u32>()
                    .unwrap_or_default(),
            };
            let latest_system_time = get_system_time()?;

            info!(
                r#"
                
        ->      {}
        ->      {}
                
                "#,
                latest_system_time.to_s()?,
                reading.to_s()?,
            );

            let sync_status = match sntp.get_sync_status() {
                SyncStatus::Reset => "SNTP_SYNC_STATUS_RESET",
                SyncStatus::Completed => "SNTP_SYNC_STATUS_COMPLETED",
                SyncStatus::InProgress => "SNTP_SYNC_STATUS_IN_PROGRESS",
            };

            debug!("sntp_get_sync_status: {}", sync_status);
        }
    }
}

/// Configure SNTP
/// - <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html#sntp-time-synchronization>
/// - <https://wokwi.com/projects/342312626601067091>
unsafe fn sntp_setup() -> Result<EspSntp> {
    let mut sntp_conf = SntpConf::default();
    sntp_conf.operating_mode = OperatingMode::Poll;
    sntp_conf.sync_mode = SyncMode::Immediate;

    sntp_set_sync_interval(15 * 1000);
    let sntp = sntp::EspSntp::new(&sntp_conf)?;

    // stop the sntp instance to redefine the callback
    // https://github.com/esp-rs/esp-idf-svc/blob/v0.42.5/src/sntp.rs#L155-L158
    sntp_stop();

    esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

    // redefine and restart the callback.
    sntp_set_time_sync_notification_cb(Some(sntp_set_time_sync_notification_cb_custom));
    sntp_init();

    esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

    info!("SNTP initialized, waiting for status!");

    while sntp.get_sync_status() != SyncStatus::Completed {
        esp_idf_sys::esp_task_wdt_reset(); // Reset WDT
    }

    info!("SNTP status received!");
    esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

    Ok(sntp)
}

unsafe fn get_system_time() -> Result<SystemTimeBuffer> {
    let timer: *mut time_t = ptr::null_mut();
    let timestamp = esp_idf_sys::time(timer);
    let utc_with_offset =
        OffsetDateTime::from_unix_timestamp(timestamp as i64)?.to_offset(UTC_OFFSET);

    let actual_time = utc_with_offset.time();
    let actual_date = utc_with_offset.date();

    let (hours, mins, secs) = actual_time.as_hms();
    let (year, month, date) = actual_date.to_calendar_date();

    let year_short = year - 2000;
    let buf = SystemTimeBuffer {
        date,
        year: year as u16,
        hours,
        minutes: mins,
        seconds: secs,
        month: month.into(),
        full_date: actual_date,
        full_time: actual_time,
    };

    let mut now: u64 = 0;
    let mut time_buf: u64 = 0;

    Ok(buf)
}

fn toggle_led<E, T>(pin: &mut T)
where
    T: embedded_hal::digital::v2::ToggleableOutputPin,
    E: std::fmt::Debug,
    <T as embedded_hal::digital::v2::ToggleableOutputPin>::Error: std::fmt::Debug,
{
    pin.toggle().unwrap();

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    std::thread::sleep(Duration::from_millis(500));
}
