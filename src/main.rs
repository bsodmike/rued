#![feature(const_btree_new)]

use core::time::Duration;

use crate::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
use crate::sensors::rtc;
use crate::sensors::rtc::rv8803::Weekday;
#[allow(unused_imports)]
use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output};
use esp_idf_hal::i2c::{self, I2cError, Master, I2C0};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use anyhow::{Error, Result};
use log::{info, warn};
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
    // let wifi = wifi::connect();
    // let ip = match &wifi {
    //     Err(e) => {
    //         println!("Wifi error: {:?}", e);
    //         format!("ERR: {:?}", e)
    //     }
    //     Ok(s) => s.1.ip.to_string(),
    // };

    // unsafe {
    //     esp_idf_sys::esp_task_wdt_reset();
    // } // Reset WDT

    // let dns = match &wifi {
    //     Err(e) => {
    //         // println!("Wifi error: {:?}", e);
    //         format!("ERR: {:?}", e)
    //     }
    //     Ok(s) => {
    //         if let Some(value) = s.1.dns {
    //             value.to_string()
    //         } else {
    //             format!("ERR: Unable to unwrap DNS value")
    //         }
    //     }
    // };

    // let _wifi_client = wifi?.0;

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
    //     30,
    //     27,
    //     9,
    //     rtc::rv8803::Weekday::Saturday.value(),
    //     12,
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

    loop {
        toggle_led::<anyhow::Error, Gpio14<Output>>(&mut led_onboard);

        std::thread::sleep(Duration::from_millis(500));

        let update = rtc.update_time(&mut _time)?;
        if !update {
            warn!("RTC: Failed reading latest time");
            println!("{:?}", _time);
        }
        let weekday: Weekday = _time[4].into();

        // FIXME: wierd bug where hundreth's value does not update inside this loop, without the use of debugging output
        println!("Hundreths: {:?}", &_time);

        info!(
            r#"
            
    ->      Hours: {} / Minutes: {} / Seconds: {} / Hundreths: {}
    ->      {}, Day: {} Month: {}, Year: {}
            
            "#,
            _time[3],
            _time[2],
            _time[1],
            _time[0],
            weekday.to_s(),
            _time[5], // day
            _time[6], // month
            _time[7], // year
        );
    }
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
