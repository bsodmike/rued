use core::cell::RefCell;
use core::fmt::Debug;
use embedded_svc::http::server::Middleware;
use embedded_svc::io::{Read, Write as EmbeddedSvcIoWrite};
use log::trace;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::fmt::Write;
use std::ops::Deref;
use std::primitive::u8;

use channel_bridge::Receiver;
use embedded_hal_0_2::PwmPin;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::http::server::{fn_handler, EspHttpServer};

use embassy_futures::select::{select, select_array, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::executor::asynch::Unblocker;
use embedded_svc::http::{Headers, Method, Query};

use channel_bridge::notification::Notification;

use crate::httpd::middleware::DefaultMiddleware;
use crate::models::{
    ApplicationStateChange, NetworkStateChange, APPLICATION_EVENT_CHANNEL, NETWORK_EVENT_CHANNEL,
};
use crate::services::httpd::LazyInitHttpServer;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum HttpdCommand {
    Noop,
}

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, HttpdCommand> = Signal::new();

pub async fn process<'a>(httpd: &'a mut LazyInitHttpServer) {
    const FIRMWARE_VERSION: &str = env!("CARGO_PKG_VERSION");

    let mut network_event = NETWORK_EVENT_CHANNEL.subscriber().unwrap();
    let mut app_event = APPLICATION_EVENT_CHANNEL.subscriber().unwrap();

    loop {
        // We are interested in network events (wifi disconnected or
        // IP address assigned), or OTA update started. On all of
        // those events we need to react.
        let (network_state, app_state) = match select(
            network_event.next_message_pure(),
            app_event.next_message_pure(),
        )
        .await
        {
            Either::First(network_state) => {
                log::info!("HTTPd: network change event received");
                (Some(network_state), None)
            }
            Either::Second(app_state) => {
                log::info!("HTTPd: app state change event received");
                (None, Some(app_state))
            }
        };

        if let Some(ApplicationStateChange::OTAUpdateStarted) = app_state {
            log::info!("OTA Update started shutting down http server");
            httpd.clear();

            break;
        }

        match network_state {
            Some(NetworkStateChange::IpAddressAssigned { ip }) => {
                let mut s = httpd.create();

                log::info!("http_server_task: starting httpd on address: {:?}", ip);
                if let Err(err) = s.handler(
                    "/",
                    embedded_svc::http::Method::Get,
                    DefaultMiddleware {}.compose(
                        //
                        fn_handler(move |req| {
                            let mut headers = embedded_svc::utils::http::Headers::<1>::new();
                            headers.set_cache_control("no-store");

                            let mut resp = req.into_response(200, None, headers.as_slice())?;
                            resp.write_all(FIRMWARE_VERSION.as_bytes())?;

                            log::info!("Processing '/' request");
                            Ok(())
                        }),
                    ),
                ) {
                    log::info!(
                        "http_server_task: failed to register http handler /: {:?} - restarting device",
                        err
                    );
                    unsafe {
                        esp_idf_sys::esp_restart();
                    }
                }

                crate::httpd::configure_handlers(&mut s)
                    .expect("Configure handlers for HTTPd server");
            }
            Some(NetworkStateChange::WifiDisconnected) => {
                log::info!("http_server_task: stopping httpd");
                httpd.clear();
            }
            None => {}
        }
    }
    log::info!("HTTPd: http_server_task shutdown");
}

//
