#![feature(const_btree_new)]
#![allow(dead_code, unused_variables)]

use ::core::time::Duration;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
use log::{debug, error, info, warn};
use once_cell::sync::Lazy;
use std::{
    env, fmt, ptr,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread::sleep,
};

use crate::{
    models::{RTClock, SystemTimeBuffer},
    sensors::rtc::rv8803::RV8803,
};

use embedded_hal::i2c::I2c;
use esp_idf_hal::timer::{Timer, TimerConfig, TimerDriver, TIMER00};
use esp_idf_hal::{delay::FreeRtos, peripheral::PeripheralRef, peripherals::Peripherals};
use esp_idf_hal::{
    gpio::PinDriver,
    i2c::{config::Config as I2cConfig, I2cDriver, I2C0},
    units::Hertz,
};
use esp_idf_svc::{
    log::EspLogger,
    sntp::{self, EspSntp, OperatingMode, SntpConf, SyncMode, SyncStatus},
};

use esp_idf_sys as _;
#[allow(unused_imports)]
use esp_idf_sys::{
    self as _, esp, esp_wifi_connect, esp_wifi_set_ps, settimeofday, sntp_init,
    sntp_set_sync_interval, sntp_set_time_sync_notification_cb, sntp_stop, time_t, timeval,
    timezone, wifi_ps_type_t_WIFI_PS_NONE,
}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use embedded_svc::wifi::{self, AuthMethod, ClientConfiguration, Wifi};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    netif::IpEvent,
    nvs::EspDefaultNvsPartition,
    wifi::{EspWifi, WifiEvent, WifiWait},
};

extern crate rust_esp32_blinky as blinky;
use blinky::micromod;

mod core;
mod display;
mod error;
mod http;
mod http_client;
mod http_server;
mod models;
mod sensors;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");

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

pub enum SysLoopMsg {
    WifiDisconnect,
    IpAddressAcquired,
}

static LOGGER: EspLogger = EspLogger;

fn wifi_connect() -> Result<()> {
    info!("Connect requested");

    esp!(unsafe { esp_wifi_connect() })?;

    info!("Connecting");

    Ok(())
}

fn main() -> Result<()> {
    esp_idf_sys::link_patches();
    // log::set_logger(&LOGGER).map(|()| LOGGER.initialize())?;
    // LOGGER.set_target_level("", log::LevelFilter::Debug);
    log::set_logger(&LOGGER).map(|()| {
        LOGGER.initialize();
        log::set_max_level(log::LevelFilter::Debug)
    })?;

    info!("Hello, Rust from an ESP32!");
    warn!("Hello, Rust from an ESP32!");
    debug!("Hello, Rust from an ESP32!");

    //  configure peripherals
    let (active_gpio, i2c_bus) = micromod::chip::configure()?;

    let mut led_onboard = PinDriver::output(active_gpio.led_onboard)?;
    led_onboard.set_low()?;

    let i2c_proxy1 = i2c_bus.acquire_i2c();
    let i2c_proxy2 = i2c_bus.acquire_i2c();

    // wifi mpsc
    let (tx, rx) = mpsc::channel::<SysLoopMsg>();

    // wifi
    let mut ip = String::default();
    let mut dns = String::default();

    info!("WIFI: Wifi name {}", crate::WIFI_SSID);
    let auth_method = AuthMethod::WPA2WPA3Personal; // Todo: add this setting - router dependent
    if crate::WIFI_SSID.is_empty() {
        anyhow::bail!("missing WiFi name")
    }
    if crate::WIFI_PSK.is_empty() {
        anyhow::bail!("Wifi password is empty")
    }

    let sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut wifi = EspWifi::new::<Modem>(
        active_gpio.modem,
        sysloop.clone(),
        Some(nvs_default_partition),
    )?;

    wifi.set_configuration(&wifi::Configuration::Client(ClientConfiguration {
        ssid: crate::WIFI_SSID.into(),
        password: crate::WIFI_PSK.into(),
        auth_method,
        ..Default::default()
    }))?;

    let wait = WifiWait::new(&sysloop)?;

    if let Err(err) = wifi.start() {
        error!("ERROR: Wifi start failure {:?}", err.to_string());
        unsafe {
            esp_idf_sys::esp_restart();
        }
    }

    // disable power save
    esp!(unsafe { esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE) })?;

    wait.wait(|| {
        debug!("WIFI: Waiting for start...");

        match wifi.is_started() {
            Ok(value) => value,
            Err(err) => {
                println!("...still starting!");
                false
            }
        }
    });
    info!("Wifi started");

    sleep(Duration::from_millis(2000));

    if let Err(err) = wifi.connect() {
        error!("ERROR: Wifi connect failure {:?}", err.to_string());
        unsafe {
            esp_idf_sys::esp_restart();
        }
    }

    let tx1 = tx.clone();
    let _wifi_event_sub = sysloop.subscribe(move |event: &WifiEvent| match event {
        WifiEvent::StaConnected => {
            info!("******* Received STA Connected Event");
        }
        WifiEvent::StaDisconnected => {
            info!("******* Received STA Disconnected event");
            tx.send(SysLoopMsg::WifiDisconnect)
                .expect("wifi event channel closed");

            sleep(Duration::from_millis(10));

            // NOTE: this is a hack
            if let Err(err) = wifi_connect() {
                info!("Error calling wifi.connect in wifi reconnect {:?}", err);
            }
        }
        _ => info!("Received other Wifi event"),
    })?;

    let _ip_event_sub = sysloop.subscribe(move |event: &IpEvent| match event {
        IpEvent::DhcpIpAssigned(_assignment) => {
            info!("************ Received IPEvent address assigned");
            tx1.send(SysLoopMsg::IpAddressAcquired)
                .expect("IP event channel closed");
        }
        _ => info!("Received other IPEvent"),
    })?;

    // SPACE

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

    // Create two proxies. Now, each sensor can have their own instance of a proxy i2c, which resolves the ownership problem.

    // let mut led_onboard = active_peripherals.led_onboard();

    // Working example
    // let mut i2c0 = peripherals.i2c0;
    // let sda = PeripheralRef::new(peripherals.pins.gpio21);
    // let scl = PeripheralRef::new(peripherals.pins.gpio22);

    // let mut config = I2cConfig::new();
    // config.baudrate(Hertz::from(400 as u32));
    // let mut i2c_driver = I2cDriver::new(i2c0, sda, scl, &config)?;

    // let bus = shared_bus::BusManagerSimple::new(i2c_driver);

    // // FIXME
    // let mut proxy_1 = bus.acquire_i2c();
    // let proxy_2 = bus.acquire_i2c();

    // setup RTC sensor
    let mut rtc =
        sensors::rtc::rv8803::RV8803::new(i2c_proxy1, sensors::rtc::rv8803::DeviceAddr::B011_0010)?;

    // if let Err(e) = display::display_test(i2c_driver, &ip, &dns) {
    //     println!("Display error: {:?}", e)
    // } else {
    //     println!("Display ok");
    // }

    // heart-beat sequence
    for i in 0..3 {
        println!("Toggling LED now: {}", i);
        led_onboard.toggle()?;
    }

    unsafe {
        let mut rtc_clock = RTClock::new();

        let sntp = sntp_setup()?;
        get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

        // // Queue configurations
        // const QUEUE_TYPE_BASE: u8 = 0;
        // const ITEM_SIZE: u32 = 0; // we're not posting any actual data, just notifying
        // const QUEUE_SIZE: u32 = 1;

        // const TIMER_SCALE: u32 = TIMER_BASE_CLK / TIMER_DIVIDER;
        // let timer_conf = timer_config_t {
        //     alarm_en: timer_alarm_t_TIMER_ALARM_EN,
        //     counter_en: timer_start_t_TIMER_START,
        //     intr_type: timer_intr_mode_t_TIMER_INTR_LEVEL,
        //     counter_dir: timer_count_dir_t_TIMER_COUNT_UP,
        //     auto_reload: timer_autoreload_t_TIMER_AUTORELOAD_EN,
        //     divider: TIMER_DIVIDER,
        // };

        // // Instantiates the event queue
        // EVENT_QUEUE = Some(xQueueGenericCreate(QUEUE_SIZE, ITEM_SIZE, QUEUE_TYPE_BASE));

        // timer_set_counter_value(timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0, 0);

        // timer_init(
        //     timer_group_t_TIMER_GROUP_0,
        //     timer_idx_t_TIMER_0,
        //     &timer_conf,
        // );

        // let timer_interval_sec: u32 = 5;
        // timer_set_alarm_value(
        //     timer_group_t_TIMER_GROUP_0,
        //     timer_idx_t_TIMER_0,
        //     (timer_interval_sec * TIMER_SCALE) as u64,
        // );
        // timer_enable_intr(timer_group_t_TIMER_GROUP_0, timer_idx_t_TIMER_0);

        // // let callback: Box<dyn FnMut() + 'static> = Box::new(|| unsafe { timer0_interrupt });

        // let mut timer_info = &TimerInfo {
        //     timer_group: timer_group_t_TIMER_GROUP_0,
        //     timer_idx: timer_idx_t_TIMER_0,
        //     alarm_interval: timer_interval_sec,
        //     auto_reload: true,
        // };

        // // let h: TimerInterruptHandler = timer0_interrupt;
        // // timer_isr_callback_add(
        // //     timer_group_t_TIMER_GROUP_0,
        // //     timer_idx_t_TIMER_0,
        // //     Some(h),
        // //     t_info_ptr,
        // //     todo!(),
        // // );

        loop {
            // FIXME
            // toggle_led::<anyhow::Error, micromod::chip::OnboardLed>(&mut led_onboard);

            let sync_status = match sntp.get_sync_status() {
                SyncStatus::Reset => "SNTP_SYNC_STATUS_RESET",
                SyncStatus::Completed => "SNTP_SYNC_STATUS_COMPLETED",
                SyncStatus::InProgress => "SNTP_SYNC_STATUS_IN_PROGRESS",
            };
            debug!("sntp_get_sync_status: {}", sync_status);

            std::thread::sleep(Duration::from_millis(500));

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

                core::update_rtc_disable();
            }

            // This only occurs if SNTP update resolves in a valid update, but the updated
            // year is < the current year.
            if core::get_fallback_to_rtc_flag() {
                update_local_from_rtc(&latest_system_time, &mut rtc, &mut rtc_clock)?;
                info!("[OK]: Updated System Clock from RTC time");

                core::fallback_to_rtc_disable()
            }

            // MPSC Handler for Wifi
            match rx.try_recv() {
                Ok(SysLoopMsg::WifiDisconnect) => {
                    info!("mpsc loop: WifiDisconnect received");

                    // httpd.clear();
                }
                Ok(SysLoopMsg::IpAddressAcquired) => {
                    info!("mpsc loop: IpAddressAcquired received");

                    let netif = wifi.sta_netif();
                    let ip_info = netif.get_ip_info()?;
                    let ip = ip_info.ip.to_string();
                    let dns = if let Some(value) = ip_info.dns {
                        value.to_string()
                    } else {
                        format!("ERR: Unable to unwrap DNS value")
                    };
                    info!(
                        r#"

                    ->   Wifi Connected: IP: {} / DNS: {}

                                    "#,
                        ip, dns
                    );

                    // let server_config = Configuration::default();
                    // let mut s = httpd.create(&server_config);

                    // if let Err(err) = s.fn_handler("/", embedded_svc::http::Method::Get, move |req| {
                    //     // url_handler::home_page_handler(req)
                    //     todo!()
                    // }) {
                    //     info!(
                    //         "mpsc loop: failed to register http handler /temperature: {:?} - restarting device",
                    //         err
                    //     );
                    //     unsafe {
                    //         esp_idf_sys::esp_restart();
                    //     }
                    // }

                    // if let Err(err) = s.fn_handler(
                    //     "/api/version",
                    //     embedded_svc::http::Method::Get,
                    //     move |req| {
                    //         // url_handler::api_version_handler(req)
                    //         todo!()
                    //     },
                    // ) {
                    //     info!(
                    //         "mpsc loop: failed to register http handler /api/version: {:?} - restarting device",
                    //         err
                    //     );
                    //     unsafe {
                    //         esp_idf_sys::esp_restart();
                    //     }
                    // }

                    // if let Err(err) =
                    //     s.fn_handler("/api/ota", embedded_svc::http::Method::Post, move |req| {
                    //         // url_handler::ota_update_handler(req)
                    //         todo!()
                    //     })
                    // {
                    //     info!(
                    //         "mpsc loop: failed to register http handler /api/ota: {:?} - restarting device",
                    //         err
                    //     );
                    //     unsafe {
                    //         esp_idf_sys::esp_restart();
                    //     }
                    // }

                    // if let Err(err) = s.fn_handler(
                    //     "/temperature",
                    //     embedded_svc::http::Method::Get,
                    //     move |req| {
                    //         // url_handler::temperature_handler(req)
                    //         todo!()
                    //     },
                    // ) {
                    //     info!(
                    //         "mpsc loop: failed to register http handler /temperature: {:?} - restarting device",
                    //         err
                    //     );
                    //     unsafe {
                    //         esp_idf_sys::esp_restart();
                    //     }
                    // }
                }
                Err(err) => {
                    if err == mpsc::TryRecvError::Disconnected {
                        warn!("mpsc loop: error recv {:?} - Disconnected", err);
                        // unsafe {
                        //     esp_idf_sys::esp_restart();
                        // }
                    }
                }
            }

            FreeRtos::delay_ms(200);
        }
    }
}

type I2cDriverType<'a> = I2cDriver<'a>;

unsafe fn get_system_time_with_fallback(
    rtc: &mut RV8803,
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
    rtc: &mut RV8803,
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

fn update_rtc_from_local(rtc: &mut RV8803, latest_system_time: &SystemTimeBuffer) -> Result<bool> {
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

    FreeRtos::delay_ms(100);
}
