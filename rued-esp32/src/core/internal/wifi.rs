use core::fmt::Debug;

use esp_idf_svc::wifi::{EspWifi, WifiEvent};
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

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize, Default)]
pub struct WifiConnection {
    pub ip: String,
    pub dns: String,
}

impl WifiConnection {
    pub fn ip(&self) -> String {
        self.ip.to_string()
    }

    pub fn dns(&self) -> String {
        self.dns.to_string()
    }
}

pub static STATE: State<Option<WifiConnection>> = State::new(
    "WIFI",
    None,
    &[
        // &super::keepalive::NOTIF,
        &super::screen::WIFI_STATE_NOTIF,
        // &super::mqtt::WIFI_STATE_NOTIF,
        // &crate::web::WIFI_STATE_NOTIF,
    ],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, WifiCommand> = Signal::new();

pub async fn process<'a, D>(
    mut wifi: impl WifiTrait,
    mut state_changed_source: impl Receiver<Data = D>,
) where
    WifiEvent: From<D>,
{
    loop {
        match select(state_changed_source.recv(), COMMAND.wait()).await {
            Either::First(data) => {
                let event: WifiEvent = data.unwrap().into();
                match event {
                    WifiEvent::StaConnected => {
                        log::info!("WifiEvent: STAConnected");
                        if wifi.is_connected().unwrap() {
                            log::info!("WifiEvent: Connected");
                        }
                    }
                    WifiEvent::StaDisconnected => {
                        log::info!("WifiEvent: STADisconnected");
                        wifi.connect().expect("Establish Wifi connection");
                    }
                    _ => (),
                }
            }
            Either::Second(command) => match command {
                WifiCommand::SetConfiguration(conf) => wifi.set_configuration(&conf).unwrap(),
            },
        }
    }
}
