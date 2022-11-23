use core::cmp::max;
use core::fmt::Debug;
use std::ops::Rem;

use embassy_futures::select::{select, Either};
use embassy_time::{Duration, Instant, Timer};

use channel_bridge::notification::Notification;

use super::state::State;
use super::{battery, quit};

const TIMEOUT: Duration = Duration::from_secs(3);

pub static STATE: State<RemainingTime> = State::new(
    "REMAINING TIME",
    RemainingTime::Indefinite,
    &[
        &super::inspector::REMAINING_TIME_NOTIF,
        // &crate::screen::REMAINING_TIME_NOTIF,
        // &crate::web::REMAINING_TIME_STATE_NOTIF,
    ],
);

pub(crate) static NOTIF: Notification = Notification::new();

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum RemainingTime {
    Indefinite,
    Duration(Duration),
}

pub async fn process() {
    let mut quit_time = None;
    let mut remaining_time_sent = None;

    loop {
        let result = select(
            NOTIF.wait(),
            Timer::after(Duration::from_secs(2)), /*Duration::from_millis(500)*/
        )
        .await;

        let now = Instant::now();

        let is_battery_powered = battery::STATE.get().powered.unwrap_or(false);

        // FIXME this is to simulate Deep-sleep
        if false {
            quit_time = None;
        } else if matches!(result, Either::First(_)) {
            // Only if this recieves a NOTIF trigger
            quit_time = Some(now + TIMEOUT);
        }

        let remaining_time = if let Some(quit_time) = quit_time {
            let mut diff = Duration::from_secs(0);
            if now < quit_time {
                diff = quit_time - now
            }

            RemainingTime::Duration(max(diff, Duration::from_secs(0)))
        } else {
            RemainingTime::Indefinite
        };

        if Some(remaining_time) != remaining_time_sent {
            remaining_time_sent = Some(remaining_time);

            STATE.update(remaining_time);
        }

        if quit_time.map(|quit_time| now >= quit_time).unwrap_or(false) {
            quit::QUIT.notify();
        }
    }
}
