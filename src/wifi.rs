use anyhow::Result;
use log::{debug, error, info};
use std::{thread::sleep, time::Duration};

use embedded_svc::wifi::{self, AuthMethod, ClientConfiguration};
use esp_idf_hal::modem::Modem;
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    netif::IpEvent,
    nvs::EspDefaultNvsPartition,
    wifi::{EspWifi, WifiEvent, WifiWait},
};
use esp_idf_sys as _;
use esp_idf_sys::{self as sys, esp, esp_wifi_set_ps, wifi_ps_type_t_WIFI_PS_NONE};

use std::sync::{mpsc::Sender, Arc};

pub enum SysLoopMsg {
    WifiDisconnect,
    IpAddressAcquired,
}

pub fn connect(modem: Modem, tx: Sender<SysLoopMsg>) -> Result<()> {
    info!("WIFI: Wifi name {}", crate::WIFI_SSID);
    let mut auth_method = AuthMethod::WPA2WPA3Personal; // Todo: add this setting - router dependent
    if crate::WIFI_SSID.is_empty() {
        anyhow::bail!("missing WiFi name")
    }
    if crate::WIFI_PSK.is_empty() {
        anyhow::bail!("Wifi password is empty")
    }

    let mut sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut wifi = EspWifi::new::<Modem>(modem, sysloop.clone(), Some(nvs_default_partition))?;

    wifi.set_configuration(&wifi::Configuration::Client(ClientConfiguration {
        ssid: crate::WIFI_SSID.into(),
        password: crate::WIFI_PSK.into(),
        // auth_method,
        ..Default::default()
    }))?;

    let wait = WifiWait::new(&sysloop)?;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT
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

            if let Err(err) = wifi.connect() {
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

    Ok(())
}
