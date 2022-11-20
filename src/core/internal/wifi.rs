use core::fmt::Debug;

use esp_idf_svc::wifi::EspWifi;
use serde::{Deserialize, Serialize};

use embassy_futures::select::{select, Either};

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use embedded_svc::wifi::{Configuration, Wifi as WifiTrait};

use channel_bridge::asynch::Receiver;

use super::state::State;

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum WifiCommand {
    SetConfiguration(Configuration),
}

pub static STATE: State<Option<bool>> = State::new(
    "WIFI",
    None,
    &[
        // &crate::keepalive::NOTIF,
        // &crate::screen::WIFI_STATE_NOTIF,
        &super::mqtt::WIFI_STATE_NOTIF,
        // &crate::web::WIFI_STATE_NOTIF,
    ],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, WifiCommand> = Signal::new();

pub async fn process<'a, D>(
    mut wifi: EspWifi<'a>,
    mut state_changed_source: impl Receiver<Data = D>,
) {
    loop {
        match select(state_changed_source.recv(), COMMAND.wait()).await {
            Either::First(_) => {
                STATE.update(Some(wifi.is_connected().unwrap()));

                // Get ip address details
                if let Ok(connected) = wifi.is_connected() {
                    if connected {
                        let netif = wifi.sta_netif();
                        if let Ok(ip_info) = netif.get_ip_info() {
                            let ip = ip_info.ip.to_string();
                            let dns = if let Some(value) = ip_info.dns {
                                value.to_string()
                            } else {
                                format!("ERR: Unable to unwrap DNS value")
                            };
                            log::info!(
                                r#"
        
        ->   Wifi Connected: IP: {} / DNS: {}
        
        "#,
                                ip,
                                dns
                            );
                        };
                    }
                }
            }
            Either::Second(command) => match command {
                WifiCommand::SetConfiguration(conf) => wifi.set_configuration(&conf).unwrap(),
            },
        }
    }
}
