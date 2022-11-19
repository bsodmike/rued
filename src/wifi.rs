use anyhow::{Context, Result};
use log::{debug, info, warn};
use std::{thread::sleep, time::Duration};

use embedded_svc::wifi::{self, AuthMethod, ClientConfiguration, Configuration, Wifi};
use esp_idf_hal::rmt::config::TransmitConfig;
use esp_idf_hal::rmt::{FixedLengthSignal, PinState, Pulse, TxRmtDriver};
use esp_idf_hal::{
    modem::Modem,
    peripheral::{Peripheral, PeripheralRef},
    peripherals::Peripherals,
};
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    netif::IpEvent,
    nvs::EspDefaultNvsPartition,
    wifi::{EspWifi, WifiEvent, WifiWait},
};
use esp_idf_sys as _;
use esp_idf_sys::{self as sys, esp, esp_wifi_set_ps, wifi_ps_type_t_WIFI_PS_NONE};

use std::cell::RefCell;
use std::sync::{
    mpsc::{self, Receiver, Sender},
    Arc,
};

pub enum SysLoopMsg {
    WifiDisconnect,
    IpAddressAcquired,
}

pub fn connect(modem: Modem, tx: Sender<SysLoopMsg>) -> Result<()> {
    println!("Wifi name {}", crate::WIFI_SSID);
    let mut auth_method = AuthMethod::WPA2WPA3Personal; // Todo: add this setting - router dependent

    // if CONFIG.wifi_ssid.is_empty() {
    //     anyhow::bail!("missing WiFi name")
    // }
    // if CONFIG.wifi_psk.is_empty() {
    //     auth_method = AuthMethod::None;
    //     info!("Wifi password is empty");
    // }

    let mut sysloop = EspSystemEventLoop::take()?;
    let nvs_default_partition = EspDefaultNvsPartition::take()?;

    let mut wifi = EspWifi::new::<Modem>(modem, sysloop.clone(), Some(nvs_default_partition))?;

    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: crate::WIFI_SSID.into(),
        password: crate::WIFI_PSK.into(),
        ..Default::default()
    }))?;

    let wait = WifiWait::new(&sysloop)?;

    unsafe {
        esp_idf_sys::esp_task_wdt_reset();
    } // Reset WDT
    wifi.start()?;

    // disable power save
    esp!(unsafe { esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_NONE) })?;

    wait.wait(|| wifi.is_started().expect("Wifi should have started!"));

    sleep(Duration::from_millis(2000));

    info!("Wifi started");
    wifi.connect()?;

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
