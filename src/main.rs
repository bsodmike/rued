use core::time::Duration;
use std::{
    io::{BufRead, BufReader},
    sync::Arc,
};

use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::gpio::{Gpio14, Gpio27, Gpio4, Gpio5, Output, Unknown};
use esp_idf_hal::i2c;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::http::client::EspHttpClient;
use esp_idf_svc::http::server::{Configuration as HttpServerConfiguration, EspHttpServer};
use esp_idf_svc::netif::EspNetifStack;
use esp_idf_svc::nvs::EspDefaultNvs;
use esp_idf_svc::{sysloop::EspSysLoopStack, wifi::EspWifi};
use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported

use embedded_svc::{
    http::client::{Client, Request, RequestWrite, Response, Status},
    http::server::{registry::Registry, Response as ServerResponse},
    io::{Read, Write},
    ipv4::ClientSettings,
    wifi::{
        ClientConfiguration, ClientConnectionStatus, ClientIpStatus, ClientStatus, Configuration,
        Status as WifiStatus, Wifi,
    },
};

use anyhow::Result;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

const SSID: &str = "foo"; // env!("SSID");
const PASSWORD: &str = "foo"; // env!("PASSWORD");

fn display_test(
    i2c0: i2c::I2C0,
    scl: Gpio4<Unknown>,
    sda: Gpio5<Unknown>,
    ip: &str,
    dns: &str,
) -> Result<()> {
    let i2c = i2c::Master::new(
        i2c0,
        i2c::MasterPins {
            scl: scl.into_output().unwrap(),       // O
            sda: sda.into_input_output().unwrap(), // I+O
        },
        i2c::config::MasterConfig::new().baudrate(400.kHz().into()),
    )?;

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display
        .init()
        .map_err(|e| anyhow::anyhow!("Init error: {:?}", e))?;

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .map_err(|e| anyhow::anyhow!("Txt error: {:?}", e))?;

    Text::with_baseline(
        &format!("IP: {}", ip),
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .map_err(|e| anyhow::anyhow!("Txt2 error: {:?}", e))?;

    Text::with_baseline(
        &format!("DNS: {}", dns),
        Point::new(0, 32),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .map_err(|e| anyhow::anyhow!("Txt3 error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Flush error: {:?}", e))?;

    Ok(())
}

fn test_wifi() -> Result<(EspWifi, ClientSettings)> {
    let netif_stack = Arc::new(EspNetifStack::new()?);
    let sys_look_stack = Arc::new(EspSysLoopStack::new()?);
    let nvs = Arc::new(EspDefaultNvs::new()?);

    let wifi = EspWifi::new(netif_stack, sys_look_stack, nvs)?;
    let mut wifi_interface = wifi;

    println!("{:?}", wifi_interface.get_status());

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    println!("Start Wifi Scan");
    let res = wifi_interface.scan_n::<10>();

    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    println!("Call wifi_connect");
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: SSID.into(),
        password: PASSWORD.into(),
        ..Default::default()
    });
    let res = wifi_interface.set_configuration(&client_config);
    println!("wifi_connect returned {:?}", res);

    println!("{:?}", wifi_interface.get_capabilities());
    println!("{:?}", wifi_interface.get_status());

    // wait to get connected
    println!("Wait to get connected");
    loop {
        if let WifiStatus(ClientStatus::Started(_), _) = wifi_interface.get_status() {
            break;
        }
    }
    let status = wifi_interface.get_status();
    println!("{:?}", status);

    // wifi_interface
    //     .wait_status_with_timeout(Duration::from_secs(30), |s| !s.is_transitional())
    //     .map_err(|e| anyhow::anyhow!("Wait timeout: {:?}", e))?;

    // let status = wifi_interface.get_status();
    // println!("Status: {:?}", status);

    // if let ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(
    //     client_settings,
    // ))) = status.0
    // {
    //     Ok((wifi_interface, client_settings))
    // } else {
    //     Err(anyhow::anyhow!("Failed to connect in time."))
    // }

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    // wait for getting an ip address
    println!("Wait to get an ip address");
    loop {
        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        // wifi_interface.poll_dhcp().unwrap();

        // wifi_interface
        //     .network_interface()
        //     .poll(timestamp())
        //     .unwrap();

        if let WifiStatus(
            ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(config))),
            _,
        ) = wifi_interface.get_status()
        {
            println!("Got IP: {:?}", config.ip.to_string());

            return Ok((wifi_interface, config));
        }
    }
}

fn main() -> Result<()> {
    // Temporary. Will disappear once ESP-IDF 4.4 is released, but for now it is necessary to call this function once,
    // or else some patches to the runtime implemented by esp-idf-sys might not link properly.
    esp_idf_sys::link_patches();

    println!("Hello, Rust from an ESP32!");
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

    let wifi = test_wifi();
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

    server.handle_get("/test", move |_request, response| {
        let resp = get("http://info.cern.ch/")?;
        let body = if let Some(body) = resp {
            println!("Response body: {}", body);
            body
        } else {
            String::default()
        };

        let mut writer = response.into_writer()?;
        writer.write_all(body.as_bytes())?;
        Ok(())
    })?;

    // setup display
    if let Err(e) = display_test(
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

fn get(url: impl AsRef<str>) -> anyhow::Result<Option<String>> {
    // 1. Create a new EspHttpClient. (Check documentation)
    let mut client = EspHttpClient::new_default()?;

    // 2. Open a GET request to `url`
    let request = client.get(url.as_ref())?;

    // 3. Requests *may* send data to the server. Turn the request into a writer, specifying 0 bytes as write length
    // (since we don't send anything - but have to do the writer step anyway)
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html
    // If this were a POST request, you'd set a write length > 0 and then writer.do_write(&some_buf);

    let writer = request.into_writer(0)?;

    // 4. Submit our write request and check the status code of the response.
    // Successful http status codes are in the 200..=299 range.

    let mut response = writer.submit()?;
    let status = response.status();
    let mut _total_size = 0;

    println!("response code: {}\n", status);

    match status {
        200..=299 => {
            // 5. if the status is OK, read response data chunk by chunk into a buffer and print it until done
            let mut buf = [0_u8; 256];
            let mut reader = response.reader();
            loop {
                if let Ok(size) = Read::read(&mut reader, &mut buf) {
                    if size == 0 {
                        break Ok(None);
                    }
                    _total_size += size;

                    // Read raw data from buffer
                    // let reader = BufReader::new(&buf[..size]);
                    // for line in reader.lines() {
                    //     println!("{:?}", line?);
                    // }

                    // 6. try converting the bytes into a Rust (UTF-8) string and print it
                    let response_text = std::str::from_utf8(&buf[..size])?;
                    // println!("{}", response_text);

                    return Ok(Some(String::from(response_text)));
                }
            }
        }
        _ => anyhow::bail!("unexpected response code: {}", status),
    }
}
