#![allow(stable_features)]
#![allow(unknown_lints)]
#![feature(async_fn_in_trait)]
#![allow(async_fn_in_trait)]
#![feature(impl_trait_projections)]
#![feature(impl_trait_in_assoc_type)]
#![feature(type_alias_impl_trait)]
#![allow(dead_code, unused_variables, unused_imports)]
#![feature(generic_arg_infer)]

extern crate alloc;

// use ::core::time::Duration;
use crate::core::internal::quit;
use anyhow::{Error, Result};
use chrono::{naive::NaiveDate, offset::Utc, DateTime, Datelike, NaiveDateTime, Timelike};
use http::{header::SEC_WEBSOCKET_ACCEPT, StatusCode};
use log::{debug, error, info, warn};
use peripherals::{ButtonsPeripherals, PulseCounterPeripherals, SPI_BUS_FREQ};
use rv8803::{i2c0::Bus as I2cBus, Rv8803, Rv8803Bus, TIME_ARRAY_LENGTH};
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
// use esp_idf_hal::{
//     delay::FreeRtos,
//     i2c::I2cError,
//     ledc::{LedcDriver, LedcTimerDriver},
//     peripheral::Peripheral,
// };
// use esp_idf_hal::{
//     i2c::{config::Config as I2cConfig, I2cDriver, I2C0},
//     ledc::config::TimerConfig,
//     units::{FromValueType, Hertz},
// };

use esp_idf_svc::{
    hal::{
        adc::{AdcConfig, AdcDriver},
        delay::FreeRtos,
        gpio::{self, DriveStrength, Gpio5, PinDriver, RTCPin},
        i2c::{I2cConfig, I2cDriver},
        interrupt::InterruptType,
        ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver},
        modem::Modem,
        peripheral::Peripheral,
        spi::{SpiConfig, SpiDeviceDriver, SpiDriver},
        units::{FromValueType, Hertz},
    },
    http::server::ws::EspHttpWsProcessor,
    log::EspLogger,
    netif::IpEvent,
    sntp::{self, EspSntp, OperatingMode, SntpConf, SyncMode, SyncStatus},
    wifi::{EspWifi, WifiEvent},
};

// use esp_idf_svc::{self as _, esp_ota_mark_app_valid_cancel_rollback};
// #[allow(unused_imports)]
// use esp_idf_svc::{
//     self as _, esp_wifi_connect, esp_wifi_set_ps, settimeofday, sntp_get_sync_status, sntp_init,
//     sntp_restart, sntp_set_sync_interval, sntp_set_time_sync_notification_cb, sntp_stop, time_t,
//     timeval, timezone, wifi_ps_type_t_WIFI_PS_NONE,
// }; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use embedded_svc::{
    http::{
        server::{Connection, Request, Response},
        Method, Query,
    },
    io::Read,
    utils::asyncify::Asyncify,
    wifi::{self, AuthMethod, ClientConfiguration, Wifi},
    ws::server::Acceptor,
};

use edge_executor::LocalExecutor;

use embassy_sync::blocking_mutex::raw::RawMutex;
#[cfg(feature = "nvs")]
use embassy_sync::blocking_mutex::Mutex as EmbassyMutex;
use embassy_time::Duration;

#[cfg(feature = "nvs")]
use embedded_svc::storage::Storage;

use esp_idf_svc::hal::adc::attenuation;
use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::reset::WakeupReason;
use esp_idf_svc::hal::task::block_on;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use esp_idf_svc::sys::esp;
use esp_idf_svc::timer::EspTaskTimerService;

use channel_bridge::{
    asynch::Receiver as AsynchReceiver,
    asynch::{pubsub, Sender as AsyncSender},
};

use crate::{
    core::internal::{
        external_rtc, keepalive, pwm, spawn,
        wifi::WifiConnection,
        ws::{self, WS_MAX_CONNECTIONS},
    },
    errors::*,
    models::{NetworkStateChange, RTClock, SystemTimeBuffer, NETWORK_EVENT_CHANNEL},
};

mod core;
mod display;
mod errors;
mod http_client;
mod httpd;
pub(crate) mod models;
mod mqtt_msg;
mod peripherals;
mod services;

// ///
// /// # Safety
// pub unsafe extern "C" fn sntp_set_time_sync_notification_cb_custom(
//     tv: *mut esp_idf_svc::sys::timeval,
// ) {
//     let naive_dt_opt = NaiveDateTime::from_timestamp_opt((*tv).tv_sec as i64, 0);
//     let naive_dt = if let Some(value) = naive_dt_opt {
//         value
//     } else {
//         NaiveDateTime::default()
//     };
//     let dt = DateTime::<Utc>::from_utc(naive_dt, Utc);

//     debug!(
//         "SNTP Sync Callback fired. Timestamp: {} / Year: {}",
//         dt.timestamp().to_string(),
//         dt.year().to_string()
//     );

//     if dt.year() < CURRENT_YEAR.into() {
//         debug!("SNTP Sync Callback: Falling back to RTC for sync.");

//         // FIXME
//         todo!();
//     } else {
//         debug!(
//             "SNTP Sync Callback: Update RTC from local time. sec: {}, usec: {}",
//             (*tv).tv_sec,
//             (*tv).tv_usec
//         );

//         external_rtc::COMMAND.signal(external_rtc::RtcExternalCommand::SntpSyncCallbackUpdateRtc);

//         // FIXME simulating, battery status update
//         // keepalive::NOTIF.notify();
//     }
// }

pub fn sntp_sync_callback(time: ::core::time::Duration) {}

const CURRENT_YEAR: u16 = 2022;
const MQTT_MAX_TOPIC_LEN: usize = 64;
const SLEEP_TIME: Duration = Duration::from_secs(5);
const SNTP_RETRY_COUNT: u32 = 1_000_000;
const SNTP_SYNC_INTERVAL: u32 = 30_000; // milli-secs
const UTC_OFFSET_CHRONO: Utc = Utc;

static LOGGER: EspLogger = EspLogger;

// Make sure that the firmware will contain
// up-to-date build time and package info coming from the binary crate
esp_idf_svc::sys::esp_app_desc!();

fn main() -> Result<(), InitError> {
    esp_idf_svc::hal::task::critical_section::link();
    esp_idf_svc::timer::embassy_time::driver::link();

    let wakeup_reason = WakeupReason::get();

    init()?;

    log::info!("Wakeup reason: {:?}", wakeup_reason);

    run(wakeup_reason)?;
    log::info!("Running...");

    sleep()?;

    unreachable!()
}

fn init() -> Result<(), InitError> {
    esp_idf_svc::sys::link_patches();

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
        esp_idf_svc::sys::esp_vfs_eventfd_register(&esp_idf_svc::sys::esp_vfs_eventfd_config_t {
            max_fds: 5,
            ..Default::default()
        })
    })?;

    Ok(())
}

fn sleep() -> Result<(), InitError> {
    unsafe {
        #[cfg(feature = "ulp")]
        esp!(esp_idf_svc::sys::esp_sleep_enable_ulp_wakeup())?;

        esp!(esp_idf_svc::sys::esp_sleep_enable_timer_wakeup(
            SLEEP_TIME.as_micros() as u64
        ))?;

        log::info!("Going to sleep");

        esp_idf_svc::sys::esp_deep_sleep_start();
    }

    #[allow(unreachable_code)]
    Ok(())
}

fn run(wakeup_reason: WakeupReason) -> Result<(), InitError> {
    let peripherals = peripherals::SystemPeripherals::take();

    // Configure I2C0

    let i2c0_peripherals = peripherals.i2c0;
    let config = I2cConfig::new();
    let _ = config.clone().baudrate(Hertz::from(400_u32));

    let i2c0_driver = I2cDriver::new(
        i2c0_peripherals.i2c,
        i2c0_peripherals.sda,
        i2c0_peripherals.scl,
        &config,
    )
    .expect("Expected to initialise I2C for i2c0 peripherals");

    // Create a shared-bus for the I2C devices that supports threads
    let i2c0_bus_manager: &'static _ = shared_bus::new_std!(I2cDriver = i2c0_driver).unwrap();

    // FIXME just for testing with slave device
    // let mut i2c_driver = i2c0_bus_manager.acquire_i2c();
    // i2c_driver.write(0x28, String::from("CommandOne").as_bytes())?;

    // Deep sleep wakeup init

    mark_wakeup_pins(&peripherals.pulse_counter, &peripherals.buttons)?;

    // ESP-IDF basics

    let nvs_default_partition = EspDefaultNvsPartition::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let timer_service = EspTaskTimerService::new()?;

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

    // SNTP

    let servers: [&str; 4] = [
        "time.aws.com",
        "1.sg.pool.ntp.org",
        "0.pool.ntp.org",
        "1.pool.ntp.org",
    ];

    let mut sntp_conf = SntpConf::default();
    sntp_conf.servers = servers;

    let sntp = sntp::EspSntp::new(&sntp_conf)?;

    // FIXME
    // NOTE upgrade
    // unsafe {
    //     // stop the sntp instance to redefine the callback
    //     // https://github.com/esp-rs/esp-idf-svc/blob/v0.42.5/src/sntp.rs#L155-L158
    //     esp_idf_svc::sys::sntp_stop();

    //     // redefine and restart the callback.
    //     esp_idf_svc::sys::sntp_set_time_sync_notification_cb(Some(
    //         esp_idf_svc::sys::sntp_set_time_sync_notification_cb_custom,
    //     ));

    //     esp_idf_svc::sys::sntp_init()
    // };

    // FIXME enable in v0.46.0
    // https://github.com/esp-rs/esp-idf-svc/pull/207
    // let sntp = sntp::EspSntp::new_with_callback(&sntp_conf, sntp_sync_callback)?;

    // Wifi

    let (wifi, wifi_notif) = services::wifi(
        peripherals.modem,
        sysloop.clone(),
        timer_service,
        Some(nvs_default_partition),
        AuthMethod::default(),
    )?;

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

    let mut httpd = services::httpd()?;
    // let ws_acceptor = httpd::configure_websockets(&mut httpd)?;

    // Mqtt

    let (mqtt_topic_prefix, mqtt_client, mqtt_conn) = services::mqtt()?;

    // PWM

    #[cfg(feature = "pwm")]
    let pwm = {
        log::info!("Setting up PWM output channels");

        let config = TimerConfig::new().frequency(500.Hz().into());
        let timer = Arc::new(LedcTimerDriver::new(peripherals.timer0.0, &config)?);

        let channel0 =
            LedcDriver::new(peripherals.ledc0.chan, timer.clone(), peripherals.ledc0.pin)?;
        let channel1 =
            LedcDriver::new(peripherals.ledc1.chan, timer.clone(), peripherals.ledc1.pin)?;
        let channel2 = LedcDriver::new(peripherals.ledc2.chan, timer, peripherals.ledc2.pin)?;

        Some((channel0, channel1, channel2))
    };

    #[cfg(not(feature = "pwm"))]
    let pwm = {
        log::warn!("PWM output disabled.");
        let value: Option<(LedcDriver, LedcDriver, LedcDriver)> = None;

        value
    };

    // SD/MMC Card

    let driver: Arc<SpiDriver<'static>> = peripherals.spi1.driver.clone();

    #[cfg(esp32)]
    #[cfg(any(
        feature = "micromod-qwiic-carrier-single",
        feature = "micromod-main-board-single"
    ))]
    let sdmmc_spi = SpiDeviceDriver::new(
        driver,
        Option::<Gpio27>::None,
        &SpiConfig::default().baudrate(SPI_BUS_FREQ.MHz().into()),
    )?;

    #[cfg(esp32)]
    #[cfg(feature = "micromod-data-logging-carrier")]
    let sdmmc_spi = SpiDeviceDriver::new(
        driver,
        Option::<Gpio5>::None,
        &SpiConfig::default().baudrate(SPI_BUS_FREQ.MHz().into()),
    )?;

    #[cfg(esp32c3)]
    let sdmmc_spi = SpiDeviceDriver::new(
        driver,
        Option::<Gpio7>::None,
        &SpiConfig::default().baudrate(SPI_BUS_FREQ.MHz().into()),
    )?;

    let sdmmc_cs = PinDriver::output(peripherals.sd_card.cs)?;

    // FIXME
    // NOTE upgrade
    // let sdmmc_spi = embedded_sdmmc::SdMmcSpi::new(sdmmc_spi, sdmmc_cs);

    // High-prio tasks

    #[cfg(feature = "display-i2c")]
    let display = {
        let proxy1 = i2c0_bus_manager.acquire_i2c();

        services::display(proxy1).expect("Return display service to the high_prio executor")
    };

    #[cfg(feature = "display-spi")]
    let display = { services::display(peripherals.display, peripherals.spi1).unwrap() };

    let mut high_prio_executor = LocalExecutor::<16>::new();

    let battery_voltage = services::adc::<{ attenuation::NONE }, _, _>(
        peripherals.battery.adc,
        peripherals.battery.voltage,
    )
    .expect("Reads battery pin");

    // TODO: Move off the main thread, as it has a fixed, low priority (1)
    spawn::high_prio(
        &mut high_prio_executor,
        services::button(
            peripherals.buttons.button1,
            &core::internal::button::BUTTON1_PIN_EDGE,
        )?,
        battery_voltage,
        PinDriver::input(peripherals.battery.power)?,
        // display,
        &mut httpd,
        pwm,
        rtc_clock,
        move |_new_state| {
            #[cfg(feature = "nvs")]
            flash_default_state(storage, _new_state);
        },
        netif_notifier(sysloop.clone()).unwrap(),
        mqtt_topic_prefix,
    );

    // Mid-prio tasks

    log::info!("Starting mid-prio executor");

    ThreadSpawnConfiguration {
        name: Some(b"async-exec-mid\0"),
        ..Default::default()
    }
    .set()
    .unwrap();

    // // let display_peripherals = peripherals.display_i2c;
    // // let proxy = display_peripherals.bus.bus.acquire_i2c();

    let mid_prio_execution = services::schedule::<8>(50000, quit::QUIT[1].wait(), move || {
        let executor = LocalExecutor::new();

        spawn::mid_prio(
            &executor,
            display,
            // move |_new_state| {
            //     #[cfg(feature = "nvs")]
            //     flash_wm_state(storage, _new_state);
            // },
        );

        spawn::wifi(&executor, wifi, wifi_notif).expect("Spawn Wifi in Executor");
        spawn::mqtt_receive(&executor, mqtt_conn);

        executor
    });

    // Low-prio tasks

    log::info!("Starting low-prio executor");

    ThreadSpawnConfiguration {
        name: Some(b"async-exec-low\0"),
        ..Default::default()
    }
    .set()
    .unwrap();

    let low_prio_execution = services::schedule::<4>(50000, quit::QUIT[2].wait(), move || {
        let executor = LocalExecutor::new();

        spawn::mqtt_send::<MQTT_MAX_TOPIC_LEN, 4>(&executor, mqtt_topic_prefix, mqtt_client);
        // spawn::ws(&executor, ws_acceptor);

        executor
    });

    // Start main execution

    log::info!("Starting high-prio executor");
    block_on(high_prio_executor.run(crate::core::internal::quit::QUIT[0].wait()));

    log::info!("Execution finished, waiting for 2s to workaround a STD/ESP-IDF pthread (?) bug");
    // This is required to allow the low prio thread to start
    std::thread::sleep(crate::StdDuration::from_millis(2000));

    mid_prio_execution.join().unwrap();
    low_prio_execution.join().unwrap();

    log::info!("Finished execution");

    Ok(())
}

#[inline(always)]
pub fn netif_notifier(
    mut sysloop: EspSystemEventLoop,
) -> Result<impl AsynchReceiver<Data = IpEvent>, InitError> {
    Ok(pubsub::SvcReceiver::new(sysloop.as_async().subscribe()?))
}

pub async fn process_netif_state_change(
    mut state_changed_source: impl AsynchReceiver<Data = IpEvent>,
) {
    loop {
        let event = state_changed_source.recv().await.unwrap();

        match event {
            IpEvent::DhcpIpAssigned(assignment) => {
                let ip = assignment.ip_settings.ip.to_string();
                let mut dns = String::default();

                info!(
                    "IpEvent: DhcpIpAssigned: IP = {}",
                    assignment.ip_settings.ip.to_string()
                );
                if let Some(address) = assignment.ip_settings.dns {
                    dns = address.to_string();
                    info!("IpEvent: DhcpIpAssigned: DNS = {}", &dns);
                } else {
                    warn!("Unable to unwrap DNS address");
                };

                // Update wifi state
                core::internal::wifi::STATE.update(Some(WifiConnection { ip, dns }));

                // if an IP address has been succesfully assigned we consider
                // the application working, no rollback required.
                unsafe { esp_idf_svc::sys::esp_ota_mark_app_valid_cancel_rollback() };

                let mut publisher = NETWORK_EVENT_CHANNEL.publisher().unwrap();
                let _ = publisher
                    .send(NetworkStateChange::IpAddressAssigned {
                        ip: assignment.ip_settings.ip,
                    })
                    .await;

                // FIXME this needs to be handled in another task
                // NOTE: Update PWM Duty-cycle from value stored in NVS.
                // let duty_cycle = nvs_default_state.pwm_duty_cycle;
                // if duty_cycle > 0 && duty_cycle <= 100 {
                //     pwm::COMMAND.signal(pwm::PwmCommand::SetDutyCycle(duty_cycle));
                // }

                // NOTE: Start SNTP
                log::info!("Starting SNTP");
                unsafe {
                    esp_idf_svc::sys::sntp_restart();
                }
            }
            _ => {
                info!("IpEvent: other .....");
            }
        }
    }
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

        #[cfg(esp32)]
        #[cfg(not(feature = "ulp"))]
        {
            // Enable power for RTC IO, sensors and ULP co-processor during Deep-sleep
            esp!(esp_idf_svc::sys::esp_sleep_pd_config(
                esp_idf_svc::sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_RTC_PERIPH,
                esp_idf_svc::sys::esp_sleep_pd_option_t_ESP_PD_OPTION_ON
            ))?;
        }
        #[cfg(esp32c3)]
        #[cfg(not(feature = "ulp"))]
        {
            // Enable power for RTC IO, sensors and ULP co-processor during Deep-sleep
            esp!(esp_idf_svc::sys::esp_sleep_pd_config(
                esp_idf_svc::sys::esp_sleep_pd_domain_t_ESP_PD_DOMAIN_CPU,
                esp_idf_svc::sys::esp_sleep_pd_option_t_ESP_PD_OPTION_ON
            ))?;
        }

        #[cfg(any(esp32, esp32s2, esp32s3))]
        esp!(esp_idf_svc::sys::esp_sleep_enable_ext1_wakeup(
            mask,
            esp_idf_svc::sys::esp_sleep_ext1_wakeup_mode_t_ESP_EXT1_WAKEUP_ALL_LOW,
        ))?;

        #[cfg(not(any(esp32, esp32s2, esp32s3)))]
        esp!(esp_idf_svc::sys::esp_deep_sleep_enable_gpio_wakeup(
            mask,
            esp_idf_svc::sys::esp_deepsleep_gpio_wake_up_mode_t_ESP_GPIO_WAKEUP_GPIO_LOW,
        ))?;
    }

    Ok(())
}

fn toggle_led<P>(driver: &mut P)
where
    P: esp_idf_svc::hal::gpio::Pin + embedded_hal::digital::ToggleableOutputPin,
{
    driver.toggle().unwrap();

    FreeRtos::delay_ms(100);
}

fn take_gpio<'d>(some_gpio: impl Peripheral<P = impl OutputPin + InputPin> + 'd) {}
