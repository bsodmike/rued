#![feature(const_btree_new)]
#![allow(dead_code, unused_variables)]
#![feature(type_alias_impl_trait)]

extern crate alloc;

// use ::core::time::Duration;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
use log::{debug, error, info, warn};
use once_cell::sync::Lazy;
use peripherals::{ButtonsPeripherals, PulseCounterPeripherals};
use std::{
    env, fmt, ptr,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread::sleep as other_sleep,
    time::Duration as StdDuration,
};

use embedded_hal::i2c::I2c;
use esp_idf_hal::{
    delay::FreeRtos,
    peripheral::Peripheral,
    peripheral::{self, PeripheralRef},
    peripherals::Peripherals,
};
use esp_idf_hal::{
    gpio::PinDriver,
    i2c::{config::Config as I2cConfig, I2cDriver, I2C0},
    units::Hertz,
};
use esp_idf_hal::{
    task::executor::FreeRtosMonitor,
    timer::{Timer, TimerConfig, TimerDriver, TIMER00},
};
use esp_idf_svc::{
    log::EspLogger,
    sntp::{self, EspSntp, OperatingMode, SntpConf, SyncMode, SyncStatus},
};

use esp_idf_sys as _;
#[allow(unused_imports)]
use esp_idf_sys::{
    self as _, esp_wifi_connect, esp_wifi_set_ps, settimeofday, sntp_get_sync_status, sntp_init,
    sntp_set_sync_interval, sntp_set_time_sync_notification_cb, sntp_stop, time_t, timeval,
    timezone, wifi_ps_type_t_WIFI_PS_NONE,
}; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use embedded_svc::wifi::{self, AuthMethod, ClientConfiguration, Wifi};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
    netif::IpEvent,
    wifi::{EspWifi, WifiEvent, WifiWait},
};

#[cfg(feature = "nvs")]
use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Duration;

#[cfg(feature = "nvs")]
use embedded_svc::storage::Storage;

use esp_idf_hal::adc::*;
use esp_idf_hal::gpio::*;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::task::executor::EspExecutor;
use esp_idf_hal::task::thread::ThreadSpawnConfiguration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use esp_idf_sys::esp;

// use ruwm::button;
// use ruwm::spawn;
// use ruwm::wm::WaterMeterState;

extern crate rued_esp32 as blinky;
use blinky::micromod;

use crate::{
    core::internal::spawn,
    errors::*,
    models::{RTClock, SystemTimeBuffer},
    sensors::rtc::rv8803::RV8803,
};

mod core;
mod display;
mod errors;
mod http;
mod http_client;
mod http_server;
mod models;
mod peripherals;
mod sensors;
mod services;

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
        "SNTP Sync Callback fired. Timestamp: {} / Year: {}",
        dt.timestamp().to_string(),
        dt.year().to_string()
    );

    if dt.year() < CURRENT_YEAR.into() {
        // Do not update RTC.
        core::fallback_to_rtc_enable();

        info!("SNTP Sync Callback: Falling back to RTC for sync.");
    } else {
        info!("RTC update flag: enabled");
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

const SLEEP_TIME: Duration = Duration::from_secs(2);
const MQTT_MAX_TOPIC_LEN: usize = 64;

// Make sure that the firmware will contain
// up-to-date build time and package info coming from the binary crate
esp_idf_sys::esp_app_desc!();

fn main() -> Result<(), InitError> {
    esp_idf_hal::task::critical_section::link();
    esp_idf_svc::timer::embassy_time::queue::link();

    let wakeup_reason = WakeupReason::get();

    init()?;

    log::info!("Wakeup reason: {:?}", wakeup_reason);

    run(wakeup_reason)?;
    log::info!("Running...");

    sleep()?;

    unreachable!()
}

fn init() -> Result<(), InitError> {
    esp_idf_sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    esp!(unsafe {
        #[allow(clippy::needless_update)]
        esp_idf_sys::esp_vfs_eventfd_register(&esp_idf_sys::esp_vfs_eventfd_config_t {
            max_fds: 5,
            ..Default::default()
        })
    })?;

    Ok(())
}

fn sleep() -> Result<(), InitError> {
    unsafe {
        #[cfg(feature = "ulp")]
        esp!(esp_idf_sys::esp_sleep_enable_ulp_wakeup())?;

        esp!(esp_idf_sys::esp_sleep_enable_timer_wakeup(
            SLEEP_TIME.as_micros() as u64
        ))?;

        log::info!("Going to sleep");

        esp_idf_sys::esp_deep_sleep_start();
    }

    Ok(())
}

fn run(wakeup_reason: WakeupReason) -> Result<(), InitError> {
    let peripherals = peripherals::SystemPeripherals::take();

    // Deep sleep wakeup init

    mark_wakeup_pins(&peripherals.pulse_counter, &peripherals.buttons)?;

    // ESP-IDF basics

    let nvs_default_partition = EspDefaultNvsPartition::take()?;
    let sysloop = EspSystemEventLoop::take()?;

    // Pulse counter

    #[cfg(feature = "ulp")]
    let (pulse_counter, pulse_wakeup) = services::pulse(peripherals.pulse_counter, wakeup_reason)?;

    #[cfg(not(feature = "ulp"))]
    let (pulse_counter, pulse_wakeup) = services::pulse(peripherals.pulse_counter)?;

    // Wifi

    // let (wifi, wifi_notif) = services::wifi(
    //     peripherals.modem,
    //     sysloop.clone(),
    //     Some(nvs_default_partition.clone()),
    // )?;

    // Httpd

    // let (_httpd, ws_acceptor) = services::httpd()?;

    // Mqtt

    // let (mqtt_topic_prefix, mqtt_client, mqtt_conn) = services::mqtt()?;

    // High-prio tasks

    let mut high_prio_executor = EspExecutor::<16, _>::new();
    let mut high_prio_tasks = heapless::Vec::<_, 16>::new();

    spawn::high_prio_test(
        &mut high_prio_executor,
        &mut high_prio_tasks,
        services::button(
            peripherals.buttons.button1,
            &core::internal::button::BUTTON1_PIN_EDGE,
        )?,
        // (wifi, wifi_notif),
    )?;

    // spawn::high_prio(
    //     &mut high_prio_executor,
    //     &mut high_prio_tasks,
    //     AdcDriver::new(peripherals.battery.adc, &AdcConfig::new().calibration(true))?,
    //     AdcChannelDriver::<_, Atten0dB<_>>::new(peripherals.battery.voltage)?,
    //     PinDriver::input(peripherals.battery.power)?,
    //     false,
    //     services::button(
    //         peripherals.buttons.button1,
    //         &core::internal::button::BUTTON1_PIN_EDGE,
    //     )?,
    //     services::button(
    //         peripherals.buttons.button2,
    //         &core::internal::button::BUTTON2_PIN_EDGE,
    //     )?,
    //     services::button(
    //         peripherals.buttons.button3,
    //         &core::internal::button::BUTTON3_PIN_EDGE,
    //     )?,
    // )?;

    // Mid-prio tasks

    // log::info!("Starting mid-prio executor");

    // ThreadSpawnConfiguration {
    //     name: Some(b"async-exec-mid\0"),
    //     ..Default::default()
    // }
    // .set()
    // .unwrap();

    // // let display_peripherals = peripherals.display;

    // let mid_prio_execution = services::schedule::<8, _>(50000, move || {
    //     let mut executor = EspExecutor::new();
    //     let mut tasks = heapless::Vec::new();

    //     // spawn::mid_prio(
    //     //     &mut executor,
    //     //     &mut tasks,
    //     //     services::display(display_peripherals).unwrap(),
    //     //     move |_new_state| {
    //     //         #[cfg(feature = "nvs")]
    //     //         flash_wm_state(storage, _new_state);
    //     //     },
    //     // )?;

    //     spawn::wifi(&mut executor, &mut tasks, wifi, wifi_notif)?;

    //     // spawn::mqtt_receive(&mut executor, &mut tasks, mqtt_conn)?;

    //     Ok((executor, tasks))
    // });

    // Low-prio tasks

    // log::info!("Starting low-prio executor");

    // ThreadSpawnConfiguration {
    //     name: Some(b"async-exec-low\0"),
    //     ..Default::default()
    // }
    // .set()
    // .unwrap();

    // let low_prio_execution = services::schedule::<4, _>(50000, move || {
    //     let mut executor = EspExecutor::new();
    //     let mut tasks = heapless::Vec::new();

    //     // spawn::mqtt_send::<MQTT_MAX_TOPIC_LEN, 4, _>(
    //     //     &mut executor,
    //     //     &mut tasks,
    //     //     mqtt_topic_prefix,
    //     //     mqtt_client,
    //     // )?;

    //     // spawn::ws(&mut executor, &mut tasks, ws_acceptor)?;

    //     Ok((executor, tasks))
    // });

    // Start main execution

    log::info!("Starting high-prio executor");
    spawn::run(&mut high_prio_executor, high_prio_tasks);

    log::info!("Execution finished, waiting for 2s to workaround a STD/ESP-IDF pthread (?) bug");

    std::thread::sleep(crate::StdDuration::from_millis(2000));

    // mid_prio_execution.join().unwrap();
    // low_prio_execution.join().unwrap();

    log::info!("Finished execution");

    Ok(())
}

fn mark_wakeup_pins(
    pulse_counter_peripherals: &PulseCounterPeripherals<impl RTCPin + InputPin>,
    buttons_peripherals: &ButtonsPeripherals<
        impl RTCPin + InputPin,
        impl RTCPin + InputPin,
        impl RTCPin + InputPin,
    >,
) -> Result<(), InitError> {
    unsafe {
        let mut mask = (1 << buttons_peripherals.button1.pin())
            | (1 << buttons_peripherals.button2.pin())
            | (1 << buttons_peripherals.button3.pin());

        #[cfg(not(feature = "ulp"))]
        {
            mask |= 1 << pulse_counter_peripherals.pulse.pin();
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        esp!(esp_idf_sys::esp_sleep_enable_ext1_wakeup(
            mask,
            esp_idf_sys::esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ALL_LOW,
        ))?;

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        esp!(esp_idf_sys::esp_deep_sleep_enable_gpio_wakeup(
            mask,
            esp_idf_sys::esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_LOW,
        ))?;
    }

    Ok(())
}

fn main2() -> Result<()> {
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

    // info!("WIFI: Wifi name {}", crate::WIFI_SSID);
    // let auth_method = AuthMethod::WPA2WPA3Personal; // Todo: add this setting - router dependent
    // if crate::WIFI_SSID.is_empty() {
    //     anyhow::bail!("missing WiFi name")
    // }
    // if crate::WIFI_PSK.is_empty() {
    //     anyhow::bail!("Wifi password is empty")
    // }

    let sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut wifi = EspWifi::new::<Modem>(
        active_gpio.modem,
        sysloop.clone(),
        Some(nvs_default_partition),
    )?;

    // wifi.set_configuration(&wifi::Configuration::Client(ClientConfiguration {
    //     ssid: crate::WIFI_SSID.into(),
    //     password: crate::WIFI_PSK.into(),
    //     // auth_method,
    //     ..Default::default()
    // }))?;

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

    other_sleep(StdDuration::from_millis(2000));

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

            other_sleep(StdDuration::from_millis(10));

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

    // setup RTC sensor
    let mut rtc = RV8803::new(i2c_proxy1)?;

    // if let Err(e) = display::display_test(i2c_driver, &ip, &dns) {
    //     println!("Display error: {:?}", e)
    // } else {
    //     println!("Display ok");
    // }

    // heart-beat sequence
    // for i in 0..3 {
    //     println!("Toggling LED now: {}", i);
    //     toggle_led(&mut led_onboard);
    // }

    unsafe {
        let mut rtc_clock = RTClock::new();

        get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

        // Setup SNTP
        let server_array: [&str; 4] = [
            "time.aws.com",
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

        sntp_set_sync_interval(30 * 1000);
        let sntp = sntp::EspSntp::new(&sntp_conf)?;

        // stop the sntp instance to redefine the callback
        // https://github.com/esp-rs/esp-idf-svc/blob/v0.42.5/src/sntp.rs#L155-L158
        // sntp_stop();

        // redefine and restart the callback.
        sntp_set_time_sync_notification_cb(Some(sntp_set_time_sync_notification_cb_custom));

        loop {
            info!("loop start...");
            std::thread::sleep(StdDuration::from_millis(500));

            let rtc_reading = &rtc_clock.update_time(&mut rtc)?;
            let latest_system_time = get_system_time_with_fallback(&mut rtc, &mut rtc_clock)?;

            //             info!(
            //                 r#"
            //                 Local time: {}
            //             Universal time:
            //                   RTC time: {}
            //                  Time zone:
            //  System clock synchronized:
            //                NTP service:
            //            RTC in local TZ:
            //                 NTP status: {}
            //                 "#,
            //                 latest_system_time.to_s()?,
            //                 rtc_reading.to_s()?,
            //                 sync_status
            //             );

            let update_rtc_flag = core::get_update_rtc_flag();
            info!("update_rtc_flag: {}", update_rtc_flag);
            if update_rtc_flag {
                update_rtc_from_local(&mut rtc, &latest_system_time)?;
                info!("[OK]: Updated RTC Clock from Local time");

                core::update_rtc_disable();
            } else {
                info!("RTC update flag: false");
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

                    // Get ip address details
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

                    // Get SNTP status
                    info!("SNTP: Enabled / Server: {:?}", servers);
                    // sntp_init();
                    let sync_status = SyncStatus::from(unsafe { sntp_get_sync_status() });

                    info!("SNTP initialized, waiting for status!");

                    let mut i: u32 = 0;
                    let mut success = true;
                    while sync_status != SyncStatus::Completed {
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

                    let sync_status = match sntp.get_sync_status() {
                        SyncStatus::Reset => "SNTP_SYNC_STATUS_RESET",
                        SyncStatus::Completed => "SNTP_SYNC_STATUS_COMPLETED",
                        SyncStatus::InProgress => "SNTP_SYNC_STATUS_IN_PROGRESS",
                    };
                    debug!("sntp_get_sync_status: {}", sync_status);

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
// unsafe fn sntp_setup<'a>() -> Result<(EspSntp, [&'a str; 1])> {

//     Ok((sntp, servers))
// }

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

fn toggle_led<P>(driver: &mut P)
where
    P: esp_idf_hal::gpio::Pin + embedded_hal::digital::ToggleableOutputPin,
{
    driver.toggle().unwrap();

    FreeRtos::delay_ms(100);
}

fn take_gpio<'d>(some_gpio: impl Peripheral<P = impl OutputPin + InputPin> + 'd) {}
