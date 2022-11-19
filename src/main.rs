#![feature(const_btree_new)]
#![allow(dead_code, unused_variables)]

use ::core::time::Duration;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
use esp_idf_hal::i2c::I2cError;
use esp_idf_sys::{c_types, timer_group_t, timer_idx_t};
use log::{debug, info, warn};
use once_cell::sync::Lazy;
use shared_bus::{I2cProxy, NullMutex};
use std::{env, error::Error as StdError, fmt, ptr, sync::Mutex};

use crate::error::BoxError;
use crate::sensors::rtc;
use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output, PinDriver};
use esp_idf_hal::{
    i2c::{config::Config as I2cConfig, I2cDriver, I2C0},
    units::Hertz,
};
use esp_idf_hal::{peripheral::PeripheralRef, peripherals::Peripherals};
// use esp_idf_hal::timer;
use embedded_hal::i2c::I2c;
use esp_idf_svc::{
    log::EspLogger,
    sntp::{self, EspSntp, OperatingMode, SntpConf, SyncMode, SyncStatus},
};

#[allow(unused_imports)]
use esp_idf_sys::{
    self as _, settimeofday, sntp_init, sntp_set_sync_interval, sntp_set_time_sync_notification_cb,
    sntp_stop, time_t, timeval, timezone,
}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

const TIMER_DIVIDER: u32 = 16;
use esp_idf_sys::{
    c_types::c_void, esp, timer_alarm_t_TIMER_ALARM_EN, timer_autoreload_t_TIMER_AUTORELOAD_EN,
    timer_config_t, timer_count_dir_t_TIMER_COUNT_UP, timer_enable_intr,
    timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0, timer_init,
    timer_intr_mode_t_TIMER_INTR_LEVEL, timer_isr_callback_add, timer_isr_t, timer_set_alarm_value,
    timer_set_counter_value, timer_start_t_TIMER_START, xQueueGenericCreate, xQueueGiveFromISR,
    xQueueReceive, QueueHandle_t, TIMER_BASE_CLK,
};

extern crate rust_esp32_blinky as blinky;
use blinky::micromod;

mod core;
mod display;
mod error;
mod http;
mod http_client;
mod http_server;
mod sensors;
mod wifi;

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("WIFI_PASS");

// This `static mut` holds the queue handle we are going to get from `xQueueGenericCreate`.
// This is unsafe, but we are careful not to enable our GPIO interrupt handler until after this value has been initialised, and then never modify it again
static mut EVENT_QUEUE: Option<QueueHandle_t> = None;

type TimerInterruptHandler = unsafe extern "C" fn(arg1: *mut c_void) -> bool;

#[link_section = ".iram0.text"]
unsafe extern "C" fn timer0_interrupt(arg1: *mut c_void) -> bool {
    xQueueGiveFromISR(EVENT_QUEUE.unwrap(), std::ptr::null_mut());

    true
}

struct UnsafeCallback(*mut Box<dyn FnMut() + 'static>);

impl UnsafeCallback {
    #[allow(clippy::type_complexity)]
    pub fn from(boxed: &mut Box<Box<dyn FnMut() + 'static>>) -> Self {
        Self(boxed.as_mut())
    }

    pub unsafe fn from_ptr(ptr: *mut c_types::c_void) -> Self {
        Self(ptr as *mut _)
    }

    pub fn as_ptr(&self) -> *mut c_types::c_void {
        self.0 as *mut _
    }

    pub unsafe fn call(&mut self) {
        let reference = self.0.as_mut().unwrap();

        (reference)();
    }
}

// REST OF THE STUFF

#[derive(Debug)]
pub struct RTClock {
    datetime: Option<DateTime<Utc>>,
}

impl RTClock {
    fn new() -> Self {
        Self { datetime: None }
    }

    fn update_time(&mut self, rtc: &mut crate::sensors::rtc::rv8803::RV8803) -> Result<RTCReading> {
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

const CURRENT_YEAR: u16 = 2022;
const UTC_OFFSET_CHRONO: Utc = Utc;
const SNTP_RETRY_COUNT: u32 = 1_000_000;
static UPDATE_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));
static FALLBACK_TO_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

// false: SNTP is enabled
static DISABLE_SNTP: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

// false: HTTPd is enabled
static DISABLE_HTTPD: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

pub unsafe extern "C" fn sntp_set_time_sync_notification_cb_custom(tv: *mut timeval) {
    let naive_dt_opt = NaiveDateTime::from_timestamp_opt((*tv).tv_sec as i64, 0);
    let naive_dt = if let Some(value) = naive_dt_opt {
        value
    } else {
        // FIXME this is bad.
        NaiveDateTime::default()
    };
    let dt = DateTime::<Utc>::from_utc(naive_dt, Utc);

    info!(
        "SNTP Sync Callback fired. Timestamp: {}",
        dt.timestamp().to_string()
    );

    if dt.year() < CURRENT_YEAR.into() {
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

pub struct TimerInfo {
    timer_group: timer_group_t,
    timer_idx: timer_idx_t,
    alarm_interval: u32,
    auto_reload: bool,
}

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
            &*self
                .0
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

impl From<CustomError> for crate::error::BlanketError {
    fn from(error: CustomError) -> Self {
        Self::new(error)
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

    // wifi
    let mut ip = String::default();
    let mut dns = String::default();
    #[cfg(feature = "wifi")]
    {
        let wifi = wifi::connect();
        ip = match &wifi {
            Err(e) => {
                println!("Wifi error: {:?}", e);
                format!("ERR: {:?}", e)
            }
            Ok(s) => s.1.ip.to_string(),
        };

        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        dns = match &wifi {
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
    }

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    // setup http server
    #[cfg(feature = "httpd_server")]
    {
        let server_config = HttpServerConfiguration::default();
        let mut server = EspHttpServer::new(&server_config)?;

        if !core::get_disable_httpd_flag() {
            info!("Httpd Server handlers: Enabled");
            let _resp = http_server::configure_handlers(&mut server)?;
        } else {
            warn!("Httpd Server handlers: Disabled");
        }
    }

    //  configure peripherals
    let peripherals = Peripherals::take().unwrap();

    // Create two proxies. Now, each sensor can have their own instance of a proxy i2c, which resolves the ownership problem.
    // let (mut active_peripherals, bus) = micromod::chip::configure(peripherals)?;

    // let mut led_onboard = active_peripherals.led_onboard();

    let mut i2c0 = peripherals.i2c0;
    let sda = PeripheralRef::new(peripherals.pins.gpio21);
    let scl = PeripheralRef::new(peripherals.pins.gpio22);

    let mut config = I2cConfig::new();
    config.baudrate(Hertz::from(400 as u32));
    let mut i2c_driver = I2cDriver::new(i2c0, sda, scl, &config)?;

    // let bus = shared_bus::BusManagerSimple::new(i2c_driver);

    // // FIXME
    // let mut proxy_1 = bus.acquire_i2c();
    // let proxy_2 = bus.acquire_i2c();

    info!("Reading RTC Sensor");

    // setup RTC sensor
    let rtc_err = CustomError::new();
    let mut rtc =
        sensors::rtc::rv8803::RV8803::new(i2c_driver, sensors::rtc::rv8803::DeviceAddr::B011_0010)?;

    //  setup display
    // #[cfg(not(feature = "wifi"))]
    // {
    //     ip = "WIFI_DISABLED".to_string();
    //     dns = "WIFI_DISABLED".to_string();
    // }

    // if let Err(e) = display::display_test(i2c_driver, &ip, &dns) {
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

        #[cfg(feature = "sntp")]
        {
            let sntp = sntp_setup()?;
        }
        get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

        // Queue configurations
        const QUEUE_TYPE_BASE: u8 = 0;
        const ITEM_SIZE: u32 = 0; // we're not posting any actual data, just notifying
        const QUEUE_SIZE: u32 = 1;

        const TIMER_SCALE: u32 = TIMER_BASE_CLK / TIMER_DIVIDER;
        let timer_conf = timer_config_t {
            alarm_en: timer_alarm_t_TIMER_ALARM_EN,
            counter_en: timer_start_t_TIMER_START,
            intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
            counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
            auto_reload: timer_autoreload_t_TIMER_AUTORELOAD_EN,
            divider: TIMER_DIVIDER,
        };

        // Instantiates the event queue
        EVENT_QUEUE = Some(xQueueGenericCreate(QUEUE_SIZE, ITEM_SIZE, QUEUE_TYPE_BASE));

        timer_set_counter_value(timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0, 0);

        timer_init(
            timer_group_t_TIMER_GROUP_0,
            timer_idx_t_TIMER_0,
            &timer_conf,
        );

        let timer_interval_sec: u32 = 5;
        timer_set_alarm_value(
            timer_group_t_TIMER_GROUP_0,
            timer_idx_t_TIMER_0,
            (timer_interval_sec * TIMER_SCALE) as u64,
        );
        timer_enable_intr(timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);

        // let callback: Box<dyn FnMut() + 'static> = Box::new(|| unsafe { timer0_interrupt });

        let mut timer_info = &TimerInfo {
            timer_group: timer_group_t_TIMER_GROUP_0,
            timer_idx: timer_idx_t_TIMER_0,
            alarm_interval: timer_interval_sec,
            auto_reload: true,
        };

        // let h: TimerInterruptHandler = timer0_interrupt;
        // timer_isr_callback_add(
        //     timer_group_t_TIMER_GROUP_0,
        //     timer_idx_t_TIMER_0,
        //     Some(h),
        //     t_info_ptr,
        //     todo!(),
        // );

        loop {
            // FIXME
            // toggle_led::<anyhow::Error, micromod::chip::OnboardLed>(&mut led_onboard);

            let mut sync_status = "";
            #[cfg(feature = "sntp")]
            {
                let sync_status = match sntp.get_sync_status() {
                    SyncStatus::Reset => "SNTP_SYNC_STATUS_RESET",
                    SyncStatus::Completed => "SNTP_SYNC_STATUS_COMPLETED",
                    SyncStatus::InProgress => "SNTP_SYNC_STATUS_IN_PROGRESS",
                };
                debug!("sntp_get_sync_status: {}", sync_status);
            }
            #[cfg(not(feature = "sntp"))]
            {
                sync_status = "SNTP_DISABLED";
            }

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
                NTP status: {}
                "#,
                latest_system_time.to_s()?,
                rtc_reading.to_s()?,
                sync_status
            );

            if core::get_update_rtc_flag() {
                update_rtc_from_local(&mut rtc, &latest_system_time)?;
                info!("[OK]: Updated RTC Clock from Local time");

                esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

                core::update_rtc_disable();
            }

            // This only occurs if SNTP update resolves in a valid update, but the updated
            // year is < the current year.
            if core::get_fallback_to_rtc_flag() {
                update_local_from_rtc(&latest_system_time, &mut rtc, &mut rtc_clock)?;
                info!("[OK]: Updated System Clock from RTC time");

                esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

                core::fallback_to_rtc_disable()
            }
        }
    }
}

type I2cDriverType<'a> = I2cDriver<'a>;

unsafe fn get_system_time_with_fallback(
    rtc: &mut crate::sensors::rtc::rv8803::RV8803,
    rtc_clock: &mut RTClock,
) -> Result<SystemTimeBuffer> {
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
        update_local_from_rtc(&system_time, rtc, rtc_clock)?;
    }

    Ok(system_time)
}

unsafe fn update_local_from_rtc(
    system_time: &SystemTimeBuffer,
    rtc: &mut crate::sensors::rtc::rv8803::RV8803,
    rtc_clock: &mut RTClock,
) -> Result<bool> {
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

    Ok(true)
}

fn update_rtc_from_local(
    rtc: &mut crate::sensors::rtc::rv8803::RV8803,
    latest_system_time: &SystemTimeBuffer,
) -> Result<bool> {
    let weekday = latest_system_time.weekday()?;

    let resp = rtc.set_time(
        latest_system_time.seconds,
        latest_system_time.minutes,
        latest_system_time.hours,
        weekday.value(),
        latest_system_time.date,
        latest_system_time.month,
        latest_system_time.year,
    )?;

    info!(
        "Updated RTC Clock from local time: {}",
        latest_system_time.to_rfc3339()?
    );

    Ok(resp)
}

#[cfg(feature = "sntp")]
/// Configure SNTP
/// - <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html#sntp-time-synchronization>
/// - <https://wokwi.com/projects/342312626601067091>
unsafe fn sntp_setup() -> Result<EspSntp> {
    let server_array: [&str; 4] = [
        "0.sg.pool.ntp.org",
        "1.sg.pool.ntp.org",
        "0.pool.ntp.org",
        "3.pool.ntp.org",
    ];

    let mut sntp_conf = SntpConf::default();
    sntp_conf.operating_mode = OperatingMode::Poll;
    sntp_conf.sync_mode = SyncMode::Immediate;

    let mut servers: [&str; 1 as usize] = Default::default();
    let copy_len = ::core::cmp::min(servers.len(), server_array.len());

    servers[..copy_len].copy_from_slice(&server_array[..copy_len]);
    sntp_conf.servers = servers;
    info!("SNTP Servers: {:?}", servers);

    sntp_set_sync_interval(15 * 1000);
    let sntp = sntp::EspSntp::new(&sntp_conf)?;

    // stop the sntp instance to redefine the callback
    // https://github.com/esp-rs/esp-idf-svc/blob/v0.42.5/src/sntp.rs#L155-L158
    sntp_stop();

    esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

    // redefine and restart the callback.
    sntp_set_time_sync_notification_cb(Some(sntp_set_time_sync_notification_cb_custom));

    if !core::get_disable_sntp_flag() {
        info!("SNTP: Enabled");
        sntp_init();
    } else {
        warn!("SNTP: Disabled");
    }

    info!("SNTP initialized, waiting for status!");

    let mut i: u32 = 0;
    let mut success = true;
    while sntp.get_sync_status() != SyncStatus::Completed {
        esp_idf_sys::esp_task_wdt_reset(); // Reset WDT

        i += 1;

        if i >= SNTP_RETRY_COUNT {
            warn!("SNTP attempted connection {} times. Now quitting.", i);
            success = false;
            break;
        }
    }

    if success {
        info!("SNTP status received! / Re-try count: {}", i);
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
    T: embedded_hal::digital::ToggleableOutputPin,
    E: std::fmt::Debug,
{
    pin.toggle().unwrap();

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    std::thread::sleep(Duration::from_millis(500));
}
