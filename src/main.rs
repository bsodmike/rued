#![feature(const_btree_new)]

use ::core::time::Duration;
use chrono::{
    naive::NaiveDate,
    offset::{FixedOffset, Utc},
    DateTime, Datelike, NaiveDateTime, Timelike, Weekday,
};
use cstr_core::CString;
use once_cell::sync::Lazy;
use std::os::unix::raw::time_t;
use std::{ptr, sync::Mutex, time::*};

use crate::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
use crate::sensors::rtc;
#[allow(unused_imports)]
use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::i2c::{self, I2cError, Master, I2C0};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::{
    gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output},
    serial::config::DataBits,
};
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::timer::EspTimerService;
use esp_idf_sys::{
    self as _, settimeofday, sntp_init, sntp_set_sync_interval, sntp_set_time_sync_notification_cb,
    sntp_stop, timeval, timezone,
}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

// Time stuff
use embedded_svc::sys_time::SystemTime;
use esp_idf_svc::systime::EspSystemTime;

use esp_idf_svc::sntp::{self, EspSntp};
use esp_idf_svc::sntp::{OperatingMode, SntpConf, SyncMode, SyncStatus};

use anyhow::{Error, Result};
use log::{debug, info, warn};
use shared_bus::{I2cProxy, NullMutex};
use std::error::Error as StdError;
use std::fmt;

mod core;
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
pub struct RTClock {
    datetime: Option<DateTime<Utc>>,
}

impl RTClock {
    fn new() -> Self {
        Self { datetime: None }
    }

    fn update_time(
        &mut self,
        rtc: &mut crate::sensors::rtc::rv8803::RV8803<
            I2cProxy<NullMutex<Master<I2C0, Gpio21<InputOutput>, Gpio22<Output>>>>,
        >,
    ) -> Result<RTCReading> {
        let mut time = [0_u8; rtc::rv8803::TIME_ARRAY_LENGTH];

        // Fetch time from RTC.
        let update = rtc.update_time(&mut time)?;
        if !update {
            warn!("RTC: Failed reading latest time");
            println!("{:?}", time);
        }

        // FIXME: wierd bug where hundreth's value does not update inside this loop, without the use of debugging output
        println!("_time: {:?}", &time);

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
        let datetime_utc = DateTime::<Utc>::from_utc(naivedatetime_utc, UTC_OFFSET_CHRONO);
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
            UTC_OFFSET_CHRONO
        ))
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
    datetime: DateTime<Utc>,
}

impl SystemTimeBuffer {
    fn to_s(&self) -> Result<String> {
        Ok(format!(
            "{} {}-{:0>2}-{:0>2} {:0>2}:{:0>2}:{:0>2} {}",
            self.weekday()?,
            self.year,
            self.month,
            self.date,
            self.hours,
            self.minutes,
            self.seconds,
            UTC_OFFSET_CHRONO
        ))
    }

    fn to_timestamp(&self) -> Result<i64> {
        Ok(self.datetime.timestamp())
    }

    fn to_rfc3339(&self) -> Result<String> {
        Ok(self.datetime.to_rfc3339())
    }

    fn weekday(&self) -> Result<rtc::rv8803::Weekday> {
        let rtc_weekday: rtc::rv8803::Weekday = self.datetime.weekday().into();

        Ok(rtc_weekday)
    }

    fn datetime(&self) -> Result<DateTime<Utc>> {
        Ok(self.datetime)
    }
}

const UTC_OFFSET_CHRONO: Utc = Utc;
static UPDATE_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));
static FALLBACK_TO_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

pub unsafe extern "C" fn sntp_set_time_sync_notification_cb_custom(tv: *mut timeval) {
    let naive_dt_opt = NaiveDateTime::from_timestamp_opt((*tv).tv_sec as i64, 0);
    let naive_dt = if let Some(value) = naive_dt_opt {
        value
    } else {
        // FIXME this is bad.
        NaiveDateTime::default()
    };
    let dt = DateTime::<Utc>::from_utc(naive_dt, Utc);

    if dt.year() == 1970 {
        // Do not update RTC.
        core::fallback_to_rtc_enable();

        info!("SNTP Sync Callback: Falling back to RTC for sync.");
    } else {
        // Update RTC
        core::update_rtc_enable();

        debug!(
            "SNTP Sync Callback called: sec: {}, usec: {}",
            (*tv).tv_sec,
            (*tv).tv_usec,
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
        let mut rtc_clock = RTClock::new();

        let sntp = sntp_setup()?;
        get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

        loop {
            toggle_led::<anyhow::Error, Gpio14<Output>>(&mut led_onboard);

            let sync_status = match sntp.get_sync_status() {
                SyncStatus::Reset => "SNTP_SYNC_STATUS_RESET",
                SyncStatus::Completed => "SNTP_SYNC_STATUS_COMPLETED",
                SyncStatus::InProgress => "SNTP_SYNC_STATUS_IN_PROGRESS",
            };
            debug!("sntp_get_sync_status: {}", sync_status);

            std::thread::sleep(Duration::from_millis(500));
            esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

            let rtc_reading = &rtc_clock.update_time(&mut rtc)?;
            let latest_system_time = get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

            info!(
                r#"
                Local time: {}
            Universal time: 
                  RTC time: {}
                 Time zone: 
 System clock synchronized: 
               NTP service: 
           RTC in local TZ: 
                "#,
                latest_system_time.to_s()?,
                rtc_reading.to_s()?,
            );

            if core::get_update_rtc_flag() {
                update_rtc_from_local(&mut rtc, &latest_system_time)?;
                info!("[OK]: Updated RTC Clock from Local time");

                esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

                core::update_rtc_disable();
            }

            if core::get_fallback_to_rtc_flag() {
                todo!();
                info!("[OK]: Updated System Clock from RTC time");

                esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

                core::fallback_to_rtc_disable()
            }
        }
    }
}

unsafe fn get_system_time_with_fallback(
    rtc: &mut crate::sensors::rtc::rv8803::RV8803<
        I2cProxy<NullMutex<Master<I2C0, Gpio21<InputOutput>, Gpio22<Output>>>>,
    >,
    rtc_clock: &mut RTClock,
) -> Result<SystemTimeBuffer> {
    let system_time = get_system_time()?;

    if system_time.year >= 2022 {
        // Update RTC clock from System time due to SNTP update
        update_rtc_from_local(rtc, &system_time)?;
    } else {
        // This should be from the RTC clock
        rtc_clock.update_time(rtc)?;
        let dt = rtc_clock.datetime()?;

        // If the System time has not been updated due to any SNTP failure,
        // update the System time from the RTC clock.
        let tz = timezone {
            tz_minuteswest: 0,
            tz_dsttime: 0,
        };

        let tv_sec = dt.timestamp() as i32;
        let tv_usec = dt.timestamp_subsec_micros() as i32;
        let tm = timeval { tv_sec, tv_usec };

        settimeofday(&tm, &tz);
        info!(
            "Updated System Clock from RTC time: {} / sec: {}, usec: {}",
            system_time.to_rfc3339()?,
            tv_sec,
            tv_usec
        );
        info!("timestamp: {}", dt.timestamp());
    }

    Ok(system_time)
}

fn update_rtc_from_local(
    rtc: &mut crate::sensors::rtc::rv8803::RV8803<
        I2cProxy<NullMutex<Master<I2C0, Gpio21<InputOutput>, Gpio22<Output>>>>,
    >,
    latest_system_time: &SystemTimeBuffer,
) -> Result<bool> {
    let weekday = latest_system_time.weekday()?;

    Ok(rtc.set_time(
        latest_system_time.seconds,
        latest_system_time.minutes,
        latest_system_time.hours,
        weekday.value(),
        latest_system_time.date,
        latest_system_time.month,
        latest_system_time.year,
    )?)
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
    // sntp_init();

    esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

    info!("SNTP initialized, waiting for status!");

    let mut i: u16 = 0;
    let mut success = true;
    while sntp.get_sync_status() != SyncStatus::Completed {
        esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

        i += 1;

        if i >= 30 {
            info!("SNTP attempted connection {} times. Now quitting.", i);
            success = false;
            break;
        }
    }

    if success {
        info!("SNTP status received!");
    }

    Ok(sntp)
}

unsafe fn get_system_time() -> Result<SystemTimeBuffer> {
    let timer: *mut time_t = ptr::null_mut();
    let timestamp = esp_idf_sys::time(timer);

    let naive_dt_opt = NaiveDateTime::from_timestamp_opt(timestamp as i64, 0);
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
