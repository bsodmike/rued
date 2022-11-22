use core::cell::RefCell;
use core::fmt::Debug;

use serde::{Deserialize, Serialize};

use log::trace;

use embassy_futures::select::select_array;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};

pub(crate) static BUTTON1_PRESSED_NOTIF: Notification = Notification::new();

#[derive(Default, Clone, Debug, Eq, PartialEq)]
pub struct InspectorState {}

impl InspectorState {
    pub const fn new() -> Self {
        Self {}
    }
}

static STATE: Mutex<CriticalSectionRawMutex, RefCell<InspectorState>> =
    Mutex::new(RefCell::new(InspectorState::new()));

#[allow(clippy::too_many_arguments)]
pub async fn process() {
    loop {
        let (_future, index) = select_array([BUTTON1_PRESSED_NOTIF.wait()]).await;

        {
            STATE.lock(|inspector_state| {
                let mut inspector_state = inspector_state.borrow_mut();

                match index {
                    0 => {
                        log::info!("[INSPECTOR: BUTTON1 Pressed]");
                    }

                    _ => unreachable!(),
                }
            });
        }

        // DRAW_REQUEST_NOTIF.notify();
    }
}
