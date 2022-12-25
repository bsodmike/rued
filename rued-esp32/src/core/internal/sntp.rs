use core::cell::RefCell;
use core::fmt::Debug;

use channel_bridge::Receiver;
use embassy_time::{Duration, Timer};
use esp_idf_hal::delay::FreeRtos;
use serde::{Deserialize, Serialize};

use log::trace;

use embassy_futures::select::{self, select, select_array, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

use super::state::State;

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub struct SntpSyncState {
    pub synced: bool,
    pub last_synced: String,
}

impl SntpSyncState {
    pub fn new(synced: bool, last_synced: &str) -> Self {
        Self {
            synced,
            last_synced: last_synced.to_string(),
        }
    }
}

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum SntpCommand {
    Pending,
    SyncCallback(SntpSyncState),
}

#[derive(Clone, PartialEq, Debug)]
pub enum SntpState {
    Initial,
    Sync(SntpSyncState),
}

pub(crate) static NOTIF: Notification = Notification::new();

pub static STATE: State<SntpState> = State::new(
    "SNTP",
    SntpState::Initial,
    &[&super::screen::SNTP_STATE_NOTIF],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, SntpCommand> = Signal::new();

pub async fn process() {
    loop {
        let result = select(NOTIF.wait(), COMMAND.wait()).await;

        if matches!(result, Either::Second(_)) {
            match result {
                Either::Second(command) => match command {
                    SntpCommand::SyncCallback(sync_state) => {
                        // NOTE: State is updated, skipping calling notifications
                        STATE.update_without_notifications(SntpState::Sync(sync_state));

                        // NOTE: Ensure time is updated as well
                        super::external_rtc::NOTIF.notify();
                    }
                    _ => unreachable!(),
                },
                _ => unreachable!(),
            }
        }
    }
}
