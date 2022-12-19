#![feature(const_btree_new)]
#![feature(type_alias_impl_trait)]
#![allow(dead_code, unused_variables)]

extern crate alloc;

// use ::core::time::Duration;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
use http::StatusCode;
use log::{debug, error, info, warn};
use once_cell::sync::Lazy;
use peripherals::{ButtonsPeripherals, PulseCounterPeripherals};
use rv8803_rs::{i2c0::Bus as I2cBus, Rv8803, Rv8803Bus, TIME_ARRAY_LENGTH};
use serde::{Deserialize, Serialize};
use serde_json::json;
use shared_bus::{BusManager, I2cProxy};
use std::{
    cell::RefCell,
    env,
    error::Error as StdError,
    fmt,
    fmt::Write,
    ops::Deref,
    ptr,
    sync::{
        mpsc::{self, Receiver, Sender},
        Arc, Mutex,
    },
    thread::sleep as other_sleep,
    time::Duration as StdDuration,
};

use embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_WriteRead;
use embedded_hal_0_2::{prelude::_embedded_hal_blocking_i2c_Write, PwmPin};
use esp_idf_hal::{
    delay::FreeRtos,
    i2c::I2cError,
    ledc::{LedcDriver, LedcTimerDriver},
    peripheral::Peripheral,
};
use esp_idf_hal::{
    i2c::{config::Config as I2cConfig, I2cDriver, I2C0},
    ledc::config::TimerConfig,
    units::{FromValueType, Hertz},
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

use embedded_svc::{
    http::{
        server::{Connection, Io, Request, Response},
        Method, Query,
    },
    io::Read,
    wifi::{self, AuthMethod, ClientConfiguration, Wifi},
};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
    netif::IpEvent,
    wifi::{EspWifi, WifiEvent, WifiWait},
};

use embassy_sync::blocking_mutex::raw::RawMutex;
#[cfg(feature = "nvs")]
use embassy_sync::blocking_mutex::Mutex as EmbassyMutex;
use embassy_time::Duration;

#[cfg(feature = "nvs")]
use embedded_svc::storage::Storage;

use esp_idf_hal::adc::*;
use esp_idf_hal::gpio::*;
use esp_idf_hal::reset::WakeupReason;
use esp_idf_hal::spi::config::Duplex;
use esp_idf_hal::spi::*;
use esp_idf_hal::task::executor::EspExecutor;
use esp_idf_hal::task::thread::ThreadSpawnConfiguration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use esp_idf_sys::esp;

use crate::{
    core::internal::{pwm, spawn},
    errors::*,
    models::{RTClock, SystemTimeBuffer},
};

mod core;
mod display;
mod errors;
mod http_client;
mod httpd;
pub(crate) mod models;
mod peripherals;
mod services;

const CURRENT_YEAR: u16 = 2022;
const UTC_OFFSET_CHRONO: Utc = Utc;
const SNTP_RETRY_COUNT: u32 = 1_000_000;
static UPDATE_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));
static FALLBACK_TO_RTC: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

// // false: SNTP is enabled
// static DISABLE_SNTP: Lazy<Mutex<bool>> = Lazy::new(|| Mutex::new(false));

///
/// # Safety
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
const SLEEP_TIME: Duration = Duration::from_secs(5);
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
    // esp_idf_svc::log::EspLogger::initialize_default();
    log::set_logger(&LOGGER)
        .map(|()| {
            LOGGER.initialize();
            log::set_max_level(log::LevelFilter::Debug)
        })
        .expect("Configure and set logger with log level");

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

    // unreachable
}

fn run(wakeup_reason: WakeupReason) -> Result<(), InitError> {
    let peripherals = peripherals::SystemPeripherals::take();

    // Configure I2C0

    let i2c0_peripherals = peripherals.i2c0;
    let config = I2cConfig::new();
    let _ = config.baudrate(Hertz::from(400_u32));

    let i2c0_driver = I2cDriver::new(
        i2c0_peripherals.i2c,
        i2c0_peripherals.sda,
        i2c0_peripherals.scl,
        &config,
    )
    .expect("Expected to initialise I2C for i2c0 peripherals");

    // Create a shared-bus for the I2C devices that supports threads
    let i2c0_bus_manager: &'static _ = shared_bus::new_std!(I2cDriver = i2c0_driver).unwrap();

    // Deep sleep wakeup init

    mark_wakeup_pins(&peripherals.pulse_counter, &peripherals.buttons)?;

    // ESP-IDF basics

    let nvs_default_partition = EspDefaultNvsPartition::take()?;
    let sysloop = EspSystemEventLoop::take()?;

    // Storage

    #[cfg(feature = "nvs")]
    let (nvs_default_state, storage): (NvsDataState, &EmbassyMutex<_, RefCell<_>>) = {
        let storage = services::storage(nvs_default_partition.clone())?;

        if let Some(nvs_default_state) = storage
            .lock(|storage| storage.borrow().get::<NvsDataState>("nvs-state"))
            .unwrap()
        {
            (nvs_default_state, storage)
        } else {
            log::warn!("No `nvs-state` found in NVS, assuming new device");

            (Default::default(), storage)
        }
    };
    log::info!("NVS: Reading at bootup {:?}", nvs_default_state);

    // Pulse counter

    #[cfg(feature = "ulp")]
    let (pulse_counter, pulse_wakeup) = services::pulse(peripherals.pulse_counter, wakeup_reason)?;

    #[cfg(not(feature = "ulp"))]
    let (pulse_counter, pulse_wakeup) = services::pulse(peripherals.pulse_counter)?;

    // Wifi

    let (wifi, wifi_notif) = services::wifi(
        peripherals.modem,
        sysloop.clone(),
        Some(nvs_default_partition),
        AuthMethod::default(),
    )?;

    info!("******* Wifi: Subscribing to events");
    let _wifi_event_sub = sysloop.subscribe(move |event: &WifiEvent| match event {
        WifiEvent::StaConnected => {
            info!("******* Received STA Connected Event");
            core::internal::wifi::COMMAND.signal(core::internal::wifi::WifiCommand::StaConnected);
        }
        WifiEvent::StaDisconnected => {
            info!("******* Received STA Disconnected event");

            FreeRtos::delay_ms(1000);

            // NOTE: calling the FFI binding directly to prevent casusing a move
            // on the the EspWifi instance.

            if let Err(err) = esp!(unsafe { esp_wifi_connect() }) {
                info!("Error calling wifi.connect in wifi reconnect {:?}", err);
            }
        }
        _ => info!("Received other Wifi event"),
    })?;

    let _ip_event_sub = sysloop.subscribe(move |event: &IpEvent| match event {
        IpEvent::DhcpIpAssigned(_) => {
            core::internal::wifi::COMMAND.signal(core::internal::wifi::WifiCommand::DhcpIpAssigned);

            // NOTE: Update PWM Duty-cycle from value stored in NVS.
            let duty_cycle = nvs_default_state.pwm_duty_cycle;
            if duty_cycle > 0 && duty_cycle <= 100 {
                pwm::COMMAND.signal(pwm::PwmCommand::SetDutyCycle(duty_cycle));
            }
        }
        _ => info!("Received other IPEvent"),
    })?;

    // RTC module (External)

    let rtc_clock: Option<RTClock<I2cProxy<Mutex<I2cDriver>>>>;
    #[cfg(feature = "external-rtc")]
    {
        rtc_clock = Some(RTClock::new(i2c0_bus_manager)?);
    }
    #[cfg(not(feature = "external-rtc"))]
    {
        rtc_clock = None;
    }

    // Httpd

    let (mut httpd, ws_acceptor) = services::httpd()?;
    httpd::configure_handlers(&mut httpd)?;

    // Mqtt

    // let (mqtt_topic_prefix, mqtt_client, mqtt_conn) = services::mqtt()?;

    // PWM
    println!("Setting up PWM output channels");

    let config = TimerConfig::new().frequency(500.Hz().into());
    let timer = Arc::new(LedcTimerDriver::new(peripherals.timer0.0, &config)?);

    let channel0 = LedcDriver::new(
        peripherals.ledc0.chan,
        timer.clone(),
        peripherals.ledc0.pin,
        &config,
    )?;
    let channel1 = LedcDriver::new(
        peripherals.ledc1.chan,
        timer.clone(),
        peripherals.ledc1.pin,
        &config,
    )?;
    let channel2 = LedcDriver::new(
        peripherals.ledc2.chan,
        timer,
        peripherals.ledc2.pin,
        &config,
    )?;

    // SD/MMC Card

    let driver: Arc<SpiDriver<'static>> = peripherals.spi1.driver.clone();

    let mut spi_config = SpiConfig::new();
    spi_config.duplex = Duplex::Full;
    let _ = spi_config.baudrate(24.MHz().into());
    // let baudrate = 40.MHz().into(); // Not supported on ESP32

    let sdmmc_spi = SpiDeviceDriver::new(driver, Option::<Gpio27>::None, &spi_config)?;
    let sdmmc_cs = PinDriver::output(peripherals.sd_card.cs)?;
    let sdmmc_spi = embedded_sdmmc::SdMmcSpi::new(sdmmc_spi, sdmmc_cs);

    // High-prio tasks

    #[cfg(feature = "display-i2c")]
    let display = {
        let proxy1 = i2c0_bus_manager.acquire_i2c();

        services::display(proxy1).expect("Return display service to the high_prio executor")
    };

    #[cfg(not(feature = "display-i2c"))]
    let (display, spi_bus) = { services::display(peripherals.display, peripherals.spi1).unwrap() };

    let mut high_prio_executor = EspExecutor::<16, _>::new();
    let mut high_prio_tasks = heapless::Vec::<_, 16>::new();
    spawn::high_prio(
        &mut high_prio_executor,
        &mut high_prio_tasks,
        services::button(
            peripherals.buttons.button1,
            &core::internal::button::BUTTON1_PIN_EDGE,
        )?,
        display,
        (wifi, wifi_notif),
        &mut httpd,
        ws_acceptor,
        (channel0, channel1, channel2),
        rtc_clock,
        move |_new_state| {
            #[cfg(feature = "nvs")]
            flash_default_state(storage, _new_state);
        },
    )?;

    // Mid-prio tasks

    // log::info!("Starting mid-prio executor");

    // ThreadSpawnConfiguration {
    //     name: Some(b"async-exec-mid\0"),
    //     ..Default::default()
    // }
    // .set()
    // .unwrap();

    // // let display_peripherals = peripherals.display_i2c;
    // // let proxy = display_peripherals.bus.bus.acquire_i2c();

    // let mid_prio_execution = services::schedule::<8, _>(50000, move || {
    //     let mut executor = EspExecutor::new();
    //     let mut tasks = heapless::Vec::new();

    //     spawn::mid_prio(
    //         &mut executor,
    //         &mut tasks,
    //         services::display(display_peripherals)
    //             .expect("Return display service to the mid_prio executor"),
    //         // move |_new_state| {
    //         //     #[cfg(feature = "nvs")]
    //         //     flash_wm_state(storage, _new_state);
    //         // },
    //     )?;

    //     // spawn::wifi(&mut executor, &mut tasks, wifi, wifi_notif)?;

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

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct NvsDataState {
    pub pwm_duty_cycle: u32,
    // pub text: [u8; 32],
}

#[cfg(feature = "nvs")]
fn flash_default_state<S>(
    storage: &'static EmbassyMutex<
        impl embassy_sync::blocking_mutex::raw::RawMutex,
        ::core::cell::RefCell<S>,
    >,
    new_state: NvsDataState,
) where
    S: Storage,
{
    core::internal::error::log_err!(storage.lock(|storage| {
        let old_state = storage.borrow().get("nvs-state")?;
        if old_state != Some(new_state) {
            storage.borrow_mut().set("nvs-state", &new_state)?;
        }

        Ok::<_, S::Error>(())
    }));
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

        // FIXME for troubleshooting later on.
        // mask = 0;
        // log::info!(
        //     "Wakeup pins mask: {} / button2: {} / button3 {}",
        //     format!("{mask:#b}"),
        //     buttons_peripherals.button2.pin(),
        //     buttons_peripherals.button3.pin()
        // );

        #[cfg(not(feature = "ulp"))]
        {
            // Enable power for RTC IO, sensors and ULP co-processor during Deep-sleep
            esp!(esp_idf_sys::esp_sleep_pd_config(
                esp_idf_sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                esp_idf_sys::esp_sleep_pd_option_t_ESP_PD_OPTION_ON
            ))?;
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

/// Configure SNTP
/// - <https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/system_time.html#sntp-time-synchronization>
/// - <https://wokwi.com/projects/342312626601067091>
// unsafe fn sntp_setup<'a>() -> Result<(EspSntp, [&'a str; 1])> {

//     Ok((sntp, servers))
// }

fn toggle_led<P>(driver: &mut P)
where
    P: esp_idf_hal::gpio::Pin + embedded_hal::digital::ToggleableOutputPin,
{
    driver.toggle().unwrap();

    FreeRtos::delay_ms(100);
}

fn take_gpio<'d>(some_gpio: impl Peripheral<P = impl OutputPin + InputPin> + 'd) {}
