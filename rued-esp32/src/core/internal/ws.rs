use core::future::Future;

use embassy_futures::select::select_array;

use embedded_svc::ws::asynch::server::Acceptor;

use channel_bridge::asynch::{ws, *};
use channel_bridge::notification::Notification;

use super::web::{self, *};

#[cfg(any(
    feature = "ws-max-connections-2",
    not(any(
        feature = "ws-max-connections-4",
        feature = "ws-max-connections-8",
        feature = "ws-max-connections-16"
    ))
))]
pub const WS_MAX_CONNECTIONS: usize = 2;
#[cfg(feature = "ws-max-connections-4")]
pub const WS_MAX_CONNECTIONS: usize = 4;
#[cfg(feature = "ws-max-connections-8")]
pub const WS_MAX_CONNECTIONS: usize = 8;
#[cfg(feature = "ws-max-connections-16")]
pub const WS_MAX_CONNECTIONS: usize = 16;

pub const WS_MAX_FRAME_LEN: usize = 512;

const NOTIF: Notification = Notification::new();

static HANDLERS_BATTERY_STATE_NOTIF: [Notification; WS_MAX_CONNECTIONS] =
    [NOTIF; WS_MAX_CONNECTIONS];
static HANDLERS_REMAINING_TIME_STATE_NOTIF: [Notification; WS_MAX_CONNECTIONS] =
    [NOTIF; WS_MAX_CONNECTIONS];
static HANDLERS_MQTT_STATE_NOTIF: [Notification; WS_MAX_CONNECTIONS] = [NOTIF; WS_MAX_CONNECTIONS];
static HANDLERS_WIFI_STATE_NOTIF: [Notification; WS_MAX_CONNECTIONS] = [NOTIF; WS_MAX_CONNECTIONS];

struct WebHandler;

impl ws::AcceptorHandler for WebHandler {
    type SendData = WebEvent;

    type ReceiveData = WebRequest;

    async fn handle<S, R>(&self, sender: S, receiver: R, index: usize) -> Result<(), S::Error>
    where
        S: Sender<Data = Self::SendData>,
        R: Receiver<Error = S::Error, Data = Option<Self::ReceiveData>>,
        S::Error: core::fmt::Debug,
    {
        web::handle(sender, receiver, &HANDLERS_BATTERY_STATE_NOTIF[index]).await
    }
}

pub async fn process<A: Acceptor>(acceptor: A) {
    ws::accept::<WS_MAX_CONNECTIONS, 1, WS_MAX_FRAME_LEN, _, _>(acceptor, WebHandler).await;
}

pub async fn broadcast() {
    loop {
        let targets = match select_array([
            BATTERY_STATE_NOTIF.wait(),
            REMAINING_TIME_STATE_NOTIF.wait(),
            MQTT_STATE_NOTIF.wait(),
            WIFI_STATE_NOTIF.wait(),
        ])
        .await
        .1
        {
            0 => &HANDLERS_BATTERY_STATE_NOTIF,
            1 => &HANDLERS_REMAINING_TIME_STATE_NOTIF,
            2 => &HANDLERS_MQTT_STATE_NOTIF,
            3 => &HANDLERS_WIFI_STATE_NOTIF,
            _ => unreachable!(),
        };

        for target in targets {
            target.notify();
        }
    }
}
