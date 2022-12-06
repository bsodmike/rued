use core::cell::RefCell;
use core::fmt::Debug;

use channel_bridge::Receiver;
use embassy_time::{Duration, Timer};
use embedded_hal_0_2::PwmPin;
use esp_idf_hal::delay::FreeRtos;
use serde::{Deserialize, Serialize};

use log::trace;

use embassy_futures::select::{select, select3, select_array, Either, Either3};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

use crate::models::rtc_external::RtcExternal;
use crate::models::SystemTimeBuffer;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};
use super::state::State;

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum RtcExternalCommand {
    Pending,
}

#[derive(Clone, PartialEq, Debug)]
pub enum RtcExternalState {
    Initial,
    UpdateScreen(SystemTimeBuffer),
}

pub(crate) static NOTIF: Notification = Notification::new();

pub static STATE: State<RtcExternalState> = State::new(
    "RTC EXTERNAL",
    RtcExternalState::Initial,
    &[
        &super::screen::EXT_RTC_NOTIF,
        // &super::inspector::REMAINING_TIME_NOTIF,
        // &crate::web::REMAINING_TIME_STATE_NOTIF,
    ],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, RtcExternalCommand> = Signal::new();

pub async fn process<'a>(mut rtc: impl RtcExternal + 'a) {
    loop {
        let result = select3(
            NOTIF.wait(),
            COMMAND.wait(),
            Timer::after(Duration::from_millis(500)),
        )
        .await;

        if matches!(result, Either3::First(_)) {
            unimplemented!()
        } else if matches!(result, Either3::Second(_)) {
            match result {
                Either3::Second(command) => match command {
                    RtcExternalCommand::Pending => {}
                },
                _ => unreachable!(),
            }
        } else if matches!(result, Either3::Third(_)) {
            unsafe {
                let resp = rtc.get_system_time_with_fallback().unwrap();

                if resp.seconds == 0 {
                    STATE.update(RtcExternalState::UpdateScreen(resp));
                }
            }
        }
    }
}
