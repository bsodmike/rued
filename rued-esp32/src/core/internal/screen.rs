use core::cell::RefCell;
use core::fmt::Debug;

use serde::{Deserialize, Serialize};

use log::trace;

use enumset::{enum_set, EnumSet, EnumSetType};

use embassy_futures::select::select_array;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};
use super::screen::shapes::util::clear;

pub use adaptors::*;
pub use shapes::Color;

use self::pages::{Battery, Summary};
use self::shapes::Action;

mod adaptors;
mod pages;
mod shapes;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Page {
    Summary = 0,
    Battery = 1,
}

impl Page {
    pub const fn new() -> Self {
        Self::Summary
    }

    pub fn prev(&self) -> Self {
        match self {
            Self::Summary => Self::Battery,
            Self::Battery => Self::Summary,
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Summary => Self::Battery,
            Self::Battery => Self::Summary,
        }
    }

    pub fn actions(&self) -> EnumSet<Action> {
        let actions = match self {
            Self::Summary => Action::OpenValve | Action::CloseValve | Action::Arm | Action::Disarm,
            Self::Battery => EnumSet::empty(),
        };

        let mut actions = actions.intersection(Action::active());

        if !actions.is_empty() {
            actions |= Action::Dismiss;
        }

        actions
    }
}

impl Default for Page {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, EnumSetType)]
pub enum DataSource {
    Page,
    Battery,
    RemainingTime,
}

#[derive(Default, Clone, Debug, Eq, PartialEq)]
pub struct ScreenState {
    changeset: EnumSet<DataSource>,
    active_page: Page,
    page_actions: Option<(EnumSet<Action>, Action)>,
}

impl ScreenState {
    pub const fn new() -> Self {
        Self {
            changeset: enum_set!(
                DataSource::Page | DataSource::Battery | DataSource::RemainingTime
            ),
            active_page: Page::new(),
            page_actions: None,
        }
    }

    pub fn battery(&self) -> Option<BatteryState> {
        self.changed([DataSource::Battery, DataSource::Page])
            .then(|| battery::STATE.get())
    }

    pub fn remaining_time(&self) -> Option<RemainingTime> {
        self.changed([DataSource::RemainingTime, DataSource::Page])
            .then(|| keepalive::STATE.get())
    }

    fn changed<const N: usize>(&self, changes: [DataSource; N]) -> bool {
        changes
            .iter()
            .find(|data_source| self.changeset.contains(**data_source))
            .is_some()
    }
}

pub(crate) static BUTTON1_PRESSED_NOTIF: Notification = Notification::new();
pub(crate) static REMAINING_TIME_NOTIF: Notification = Notification::new();

static DRAW_REQUEST_NOTIF: Notification = Notification::new();

static STATE: Mutex<CriticalSectionRawMutex, RefCell<ScreenState>> =
    Mutex::new(RefCell::new(ScreenState::new()));

#[allow(clippy::too_many_arguments)]
pub async fn process() {
    loop {
        let (_future, index) =
            select_array([BUTTON1_PRESSED_NOTIF.wait(), REMAINING_TIME_NOTIF.wait()]).await;

        {
            STATE.lock(|screen_state| {
                let mut screen_state = screen_state.borrow_mut();

                match index {
                    0 => {
                        if let Some((actions, action)) = screen_state.page_actions {
                            screen_state.page_actions =
                                action.prev(&actions).map(|action| (actions, action));
                        } else {
                            screen_state.active_page = screen_state.active_page.prev();
                        }

                        screen_state.changeset.insert(DataSource::Page);
                    }
                    1 => {
                        if let Some((actions, action)) = screen_state.page_actions {
                            screen_state.page_actions =
                                action.next(&actions).map(|action| (actions, action));
                        } else {
                            screen_state.active_page = screen_state.active_page.next();
                        }

                        screen_state.changeset.insert(DataSource::Page);
                    }

                    _ => unreachable!(),
                }
            });
        }

        DRAW_REQUEST_NOTIF.notify();
    }
}

pub async fn unblock_run_draw<U, D>(unblocker: U, mut display: D)
where
    U: Unblocker,
    D: Flushable<Color = Color> + Send + 'static,
    D::Error: Debug + Send + 'static,
{
    loop {
        let screen_state = wait_change().await;

        display = unblocker
            .unblock(move || draw(display, screen_state))
            .await
            .unwrap();
    }
}

pub async fn run_draw<D>(mut display: D)
where
    D: Flushable<Color = Color>,
    D::Error: Debug,
{
    loop {
        let screen_state = wait_change().await;

        log::info!("run_draw: {:?}", screen_state);
        display = draw(display, screen_state).unwrap();
    }
}

async fn wait_change() -> ScreenState {
    DRAW_REQUEST_NOTIF.wait().await;

    STATE.lock(|screen_state| {
        let screen_state_prev = screen_state.borrow().clone();

        screen_state.borrow_mut().changeset = EnumSet::empty();

        screen_state_prev
    })
}

fn draw<D>(mut display: D, screen_state: ScreenState) -> Result<D, D::Error>
where
    D: Flushable<Color = Color>,
    D::Error: Debug,
{
    trace!("DRAWING: {:?}", screen_state);

    let page_changed = screen_state.changeset.contains(DataSource::Page);

    if page_changed {
        clear(&display.bounding_box(), &mut display)?;
    }

    match screen_state.active_page {
        Page::Summary => Summary::draw(
            &mut display,
            page_changed,
            // screen_state.valve().as_ref(),
            // screen_state.wm().as_ref(),
            screen_state.battery().as_ref(),
            screen_state.remaining_time().as_ref(),
        )?,
        Page::Battery => {
            Battery::draw(&mut display, page_changed, screen_state.battery().as_ref())?
        }
    }

    if let Some((actions, action)) = screen_state.page_actions {
        pages::actions::draw(&mut display, actions, action)?;
    }

    display.flush()?;

    Ok(display)
}
