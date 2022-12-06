use core::cell::RefCell;
use core::fmt::Debug;
use embedded_svc::io::{Read, Write as EmbeddedSvcIoWrite};
use log::trace;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::fmt::Write;
use std::primitive::u8;

use channel_bridge::Receiver;
use embedded_hal_0_2::PwmPin;
use esp_idf_hal::delay::FreeRtos;
use esp_idf_svc::http::server::EspHttpServer;

use embassy_futures::select::{select, select_array, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::executor::asynch::Unblocker;
use embedded_svc::http::{Headers, Method, Query};

use channel_bridge::notification::Notification;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum HttpdCommand {
    Noop,
}

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, HttpdCommand> = Signal::new();

pub async fn process<'a>(httpd: &'a mut EspHttpServer) {
    loop {
        let (future, index) = select_array([COMMAND.wait()]).await;

        {
            match index {
                0 => match future {
                    HttpdCommand::Noop => {}
                },
                _ => unreachable!(),
            }
        }
    }
}
