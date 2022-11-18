use anyhow::{Context, Result};
use log::info;

use embedded_svc::wifi::{ClientConfiguration, Configuration};
use esp_idf_hal::{modem::Modem, peripheral::Peripheral, peripherals::Peripherals};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition, wifi::EspWifi};

pub fn connect() -> Result<(EspWifi<'static>, String, String)> {
    let mut sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;
    let peripherals = Peripherals::take().context("failed to take Peripherals")?;

    let mut wifi = EspWifi::new::<Modem>(
        peripherals.modem.into_ref(),
        sysloop.clone(),
        Some(nvs_default_partition),
    )?;
    println!("{:?}", wifi.get_capabilities());

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: crate::SSID.into(),
        password: crate::PASSWORD.into(),
        ..Default::default()
    }))?;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    println!("Start Wifi Scan");
    let res = wifi.scan_n::<10>();

    if let Ok((res, _count)) = res {
        for ap in res {
            println!("{:?}", ap);
        }
    }

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    wifi.connect()?;
    println!("Call wifi_connect");
    println!("{:?}", wifi.get_capabilities()?);

    // // wait to get connected
    println!("Wait to get connected");
    loop {
        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        if !wifi.is_up()? {
            break;
        }
    }
    println!("Wifi up: {}", wifi.is_up()?);

    // OLD

    // wifi
    //     .wait_status_with_timeout(Duration::from_secs(30), |s| !s.is_transitional())
    //     .map_err(|e| anyhow::anyhow!("Wait timeout: {:?}", e))?;

    // let status = wifi.get_status();
    // println!("Status: {:?}", status);

    // if let ClientStatus::Started(ClientConnectionStatus::Connected(ClientIpStatus::Done(
    //     client_settings,
    // ))) = status.0
    // {
    //     Ok((wifi, client_settings))
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

        let netif = wifi.ap_netif();
        if netif.is_up()? {
            let ip_info = netif.get_ip_info()?;
            let ip = ip_info.ip.to_string();

            unsafe {
                esp_idf_sys::esp_task_wdt_reset();
            } // Reset WDT

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

            return Ok((wifi, ip, dns));
        }
    }
}
