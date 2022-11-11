#![feature(const_btree_new)]

use core::time::Duration;

use crate::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
#[allow(unused_imports)]
use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::gpio::{Gpio14, Gpio27, Output};
use esp_idf_hal::i2c;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;

use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use anyhow::Result;
use log::info;

mod display;
mod http;
mod http_client;
mod http_server;
mod wifi;

const SSID: &str = "foo"; // env!("SSID");
const PASSWORD: &str = "foo"; // env!("PASSWORD");

fn main() -> Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    // FIXME: Stack overflow?
    // I (19149) esp_idf_svc::wifi: STA IP event DhcpIpAssigned(DhcpIpAssignment { netif_handle: 0x3ffc659c, ip_se
    //     ttings: ClientSettings { ip: 10.0.1.121, subnet: Subnet { gateway: 10.0.1.1, mask: Mask(24) }, dns: None, s
    //     econdary_dns: None }, ip_changed: true }) handled, set status: Status(Started(Connected(Done(ClientSettings
    //      { ip: 10.0.1.121, subnet: Subnet { gateway: 10.0.1.1, mask: Mask(24) }
    //     ***ERROR*** A stack overflow in task sys_evt has been detected.
    //     Backtrace: 0x40081f72:0x3ffbdfb0 0x40088db5:0x3ffbdfd0 0x4008bd9a:0x3ffbdff0 0x4008a85d:0x3ffbe070 0x40088e
    //     b0:0x3ffbe0a0 0x40088e62:0x3ffaf034 |<-CORRUPTED
    //     0x40081f72 - panic_abort
    //         at /home/mdesilva/esp/rust-esp32-blinky/.embuild/espressif/esp-idf/release-v4.4/components/esp_system/p
    //     anic.c:402
    //     0x40088db5 - esp_system_abort
    //         at /home/mdesilva/esp/rust-esp32-blinky/.embuild/espressif/esp-idf/release-v4.4/components/esp_system/e
    //     sp_system.c:128
    EspLogger::initialize_default();

    info!("Hello, Rust from an ESP32!");
    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    //  configure peripherals
    let peripherals = Peripherals::take().unwrap();
    // let mut led = peripherals.pins.gpio17.into_output().unwrap();
    // let mut led_onboard = peripherals.pins.gpio18.into_output().unwrap();

    // D0 - GPIO pin 14, pad 10 on the MicroMod
    let gpio_d0: Gpio14<Output> = peripherals.pins.gpio14.into_output().unwrap();
    // D1 - GPIO pin 27, pad 18 on the MicroMod
    let _gpio_d1: Gpio27<Output> = peripherals.pins.gpio27.into_output().unwrap();
    let mut led_onboard = gpio_d0;

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
    let server_config = HttpServerConfiguration::default();
    let mut server = EspHttpServer::new(&server_config)?;
    let _resp = http_server::configure_handlers(&mut server)?;

    // setup display
    if let Err(e) = display::display_test(
        peripherals.i2c0,
        peripherals.pins.gpio4,
        peripherals.pins.gpio5,
        &ip,
        &dns,
    ) {
        println!("Display error: {:?}", e)
    } else {
        println!("Display ok");
    }

    // heart-beat sequence
    for i in 0..20 {
        println!("Toggling LED now: {}", i);
        toggle_led::<anyhow::Error, Gpio14<esp_idf_hal::gpio::Output>>(&mut led_onboard);
    }

    loop {
        toggle_led::<anyhow::Error, Gpio14<esp_idf_hal::gpio::Output>>(&mut led_onboard);
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
