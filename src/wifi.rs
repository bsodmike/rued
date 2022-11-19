use anyhow::{Context, Result};
use log::{debug, info, warn};
use std::time::Duration;

use embedded_svc::wifi::{ClientConfiguration, Configuration, Wifi};
use esp_idf_hal::{
    modem::Modem,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Peripherals,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    netif::{EspNetif, EspNetifWait},
    nvs::EspDefaultNvsPartition,
    wifi::EspWifi,
};

pub enum SysLoopMsg {
    WifiDisconnect,
    IpAddressAsquired,
}

pub fn connect(modem: PeripheralRef<Modem>) -> Result<EspWifi> {
    let mut sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut wifi = EspWifi::new::<Modem>(modem, sysloop.clone(), Some(nvs_default_partition))?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: crate::SSID.into(),
        password: crate::PASSWORD.into(),
        ..Default::default()
    }))?;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT

    // println!("Start Wifi Scan");
    // let res = wifi.scan_n::<10>();

    // if let Ok((res, _count)) = res {
    //     for ap in res {
    //         println!("{:?}", ap);
    //     }
    // }

    println!("Wait for start");
    wifi.start()?;
    loop {
        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        if wifi.is_started()? {
            info!("Wifi driver started!");
            break;
        } else {
            println!("Wifi driver yet to start...");
        }
    }

    loop {
        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        println!("Call wifi_connect");
        debug!("{:?}", wifi.get_capabilities()?);
        wifi.connect()?;

        // wait to get connected
        println!("Wait to get connected");
        loop {
            unsafe {
                esp_idf_sys::esp_task_wdt_reset();
            } // Reset WDT

            if wifi.is_connected()? {
                info!("WIFI: Connected!");
                break;
            } else {
                println!("Wifi not connected yet...");
            }
        }

        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        let netif_wait = EspNetifWait::new::<EspNetif>(wifi.sta_netif(), &sysloop)?;

        // Wait for DHCP to deliver IP
        let got_ip = netif_wait.wait_with_timeout(core::time::Duration::from_secs(60), || {
            wifi.sta_netif().get_ip_info().unwrap().ip != std::net::Ipv4Addr::UNSPECIFIED
        });

        if got_ip {
            unsafe {
                esp_idf_sys::esp_task_wdt_reset();
            } // Reset WDT

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

            return Ok(wifi);
        } else {
            warn!("WIFI: Failed to obtain IP via DHCP... re-attempting now.");

            wifi.disconnect();
            std::thread::sleep(Duration::from_millis(250));
        }
    }
}
