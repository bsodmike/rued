use anyhow::Result;

use std::sync::Arc;

use esp_idf_svc::netif::EspNetifStack;
use esp_idf_svc::nvs::EspDefaultNvs;
use esp_idf_svc::{sysloop::EspSysLoopStack, wifi::EspWifi};

use embedded_svc::{
    ipv4::ClientSettings,
    wifi::{
        ClientConfiguration, ClientConnectionStatus, ClientIpStatus, ClientStatus, Configuration,
        Status as WifiStatus, Wifi,
    },
};

pub fn connect() -> Result<(EspWifi, ClientSettings)> {
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
        ssid: crate::SSID.into(),
        password: crate::PASSWORD.into(),
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
