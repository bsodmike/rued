use core::cell::RefCell;
use core::fmt::Debug;

use embedded_graphics::mono_font::ascii::FONT_9X15;
use embedded_graphics::mono_font::iso_8859_9::FONT_10X20;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::prelude::{DrawTarget, Point};
use serde::{Deserialize, Serialize};

use log::trace;

use enumset::{enum_set, EnumSet, EnumSetType};

use embassy_futures::select::select_array;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;

use gfx_xtra::draw_target::Flushable;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};
use super::screen::shapes::util::{self, clear};
use super::wifi::{self, WifiConnection};

pub use shapes::Color;

use self::pages::{Battery, Summary, Wifi};
use self::shapes::Action;

#[cfg(feature = "display-i2c")]
pub use i2c::*;
#[cfg(feature = "display-i2c")]
mod i2c {
    use embedded_graphics::pixelcolor::BinaryColor;

    pub type DisplayColor = BinaryColor;
    pub const DISPLAY_COLOR_YELLOW: DisplayColor = BinaryColor::On; // Color::Yellow
    pub const DISPLAY_COLOR_GREEN: DisplayColor = BinaryColor::On; // Color::Green
    pub const DISPLAY_COLOR_BLACK: DisplayColor = BinaryColor::On; // Color::Black
    pub const DISPLAY_COLOR_WHITE: DisplayColor = BinaryColor::On; // Color::White
    pub const DISPLAY_COLOR_GRAY: DisplayColor = BinaryColor::On; // Color::Gray
    pub const DISPLAY_COLOR_LIGHT_BLUE: DisplayColor = BinaryColor::On; // Color::LightBlue
}

#[cfg(not(feature = "display-i2c"))]
pub use color::*;
#[cfg(not(feature = "display-i2c"))]
mod color {
    use super::Color;

    pub type DisplayColor = Color;
    pub const DISPLAY_COLOR_YELLOW: DisplayColor = Color::Yellow;
    pub const DISPLAY_COLOR_GREEN: DisplayColor = Color::Green;
    pub const DISPLAY_COLOR_BLACK: DisplayColor = Color::Black;
    pub const DISPLAY_COLOR_WHITE: DisplayColor = Color::White;
    pub const DISPLAY_COLOR_GRAY: DisplayColor = Color::Gray;
    pub const DISPLAY_COLOR_LIGHT_BLUE: DisplayColor = Color::LightBlue;
}

mod pages;
mod shapes;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Page {
    Summary = 0,
    Battery = 1,
    Wifi = 2,
}

impl Page {
    pub const fn new() -> Self {
        Self::Summary
    }

    pub fn prev(&self) -> Self {
        match self {
            Self::Summary => Self::Wifi,
            Self::Wifi => Self::Battery,
            Self::Battery => Self::Summary,
        }
    }

    pub fn next(&self) -> Self {
        match self {
            Self::Summary => Self::Battery,
            Self::Battery => Self::Wifi,
            Self::Wifi => Self::Summary,
        }
    }

    pub fn actions(&self) -> EnumSet<Action> {
        let actions = match self {
            Self::Summary => Action::OpenValve | Action::CloseValve | Action::Arm | Action::Disarm,
            Self::Battery => EnumSet::empty(),
            Self::Wifi => EnumSet::empty(),
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
    Wifi,
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
                DataSource::Page
                    | DataSource::Battery
                    | DataSource::RemainingTime
                    | DataSource::Wifi
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

    pub fn wiif(&self) -> Option<Option<WifiConnection>> {
        self.changed([DataSource::Wifi, DataSource::Page])
            .then(|| wifi::STATE.get())
    }

    fn changed<const N: usize>(&self, changes: [DataSource; N]) -> bool {
        changes
            .iter()
            .find(|data_source| self.changeset.contains(**data_source))
            .is_some()
    }
}

pub(crate) static BUTTON1_PRESSED_NOTIF: Notification = Notification::new();
pub(crate) static BUTTON2_PRESSED_NOTIF: Notification = Notification::new();
pub(crate) static BUTTON3_PRESSED_NOTIF: Notification = Notification::new();
pub(crate) static WIFI_STATE_NOTIF: Notification = Notification::new();
pub(crate) static REMAINING_TIME_NOTIF: Notification = Notification::new();

static DRAW_REQUEST_NOTIF: Notification = Notification::new();

static STATE: Mutex<CriticalSectionRawMutex, RefCell<ScreenState>> =
    Mutex::new(RefCell::new(ScreenState::new()));

#[allow(clippy::too_many_arguments)]
pub async fn process() {
    loop {
        let (_future, index) = select_array([
            BUTTON1_PRESSED_NOTIF.wait(), // 0
            BUTTON2_PRESSED_NOTIF.wait(), // 1
            BUTTON3_PRESSED_NOTIF.wait(), // 2
            REMAINING_TIME_NOTIF.wait(),  // 3
            WIFI_STATE_NOTIF.wait(),      // 4
        ])
        .await;

        {
            STATE.lock(|screen_state| {
                let mut screen_state = screen_state.borrow_mut();

                match index {
                    0 => {
                        if let Some((actions, action)) = screen_state.page_actions {
                            screen_state.page_actions =
                                action.prev(&actions).map(|action| (actions, action));

                            log::info!("0: page_actions: {:?}", screen_state.page_actions);
                        } else {
                            screen_state.active_page = screen_state.active_page.prev();
                            log::info!("0: active_page: {:?}", screen_state.active_page);
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
                    2 => {
                        if let Some((_, action)) = screen_state.page_actions {
                            screen_state.page_actions = None;
                            action.trigger();
                        } else {
                            let actions = screen_state.active_page.actions();
                            screen_state.page_actions =
                                Action::first(&actions).map(|action| (actions, action));
                        }

                        screen_state.changeset.insert(DataSource::Page);
                    }
                    3 => {
                        screen_state.changeset.insert(DataSource::RemainingTime);
                    }
                    4 => {
                        screen_state.active_page = Page::Summary;
                        screen_state.changeset.insert(DataSource::Page);
                    }

                    _ => unreachable!(),
                }
            });
        }

        log::info!("screen: DRAW_REQUEST_NOTIF.notify()");
        DRAW_REQUEST_NOTIF.notify();
    }
}

pub async fn unblock_run_draw<U, D>(unblocker: U, mut display: D)
where
    U: Unblocker,
    D: Flushable<Color = crate::core::internal::screen::DisplayColor> + Send + 'static,
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
    D: Flushable<Color = crate::core::internal::screen::DisplayColor>,
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

// fn draw_text<T>(target: &mut T, text: &str, top: bool) -> Result<(), T::Error>
// where
//     T: DrawTarget<Color = crate::core::internal::screen::DisplayColor>
//         + Flushable<Color = crate::core::internal::screen::DisplayColor>,
//     // FIXME needed to call target.size()
//     // + OriginDimensions,
//     // + Flushable,
//     // T::Error: Debug,
// {
//     // Size { width, height } = target.size();
//     // let position = Point::new(width as i32 / 2, height as i32 / 2);
//     let mut position: Point;
//     if top {
//         position = Point::new(0, 0)
//     } else {
//         position = Point::new(0, 16)
//     }

//     #[cfg(feature = "display-i2c")]
//     target.clear(BinaryColor::Off)?;
//     #[cfg(not(feature = "display-i2c"))]
//     target.clear(Color::Black)?;

//     log::info!("DRAWING text: {}", &text);
//     util::text(
//         &FONT_9X15,
//         target,
//         position,
//         text,
//         DISPLAY_COLOR_WHITE,
//         None,
//     )?;

//     target.flush()?;

//     Ok(())
// }

fn draw<D>(mut display: D, screen_state: ScreenState) -> Result<D, D::Error>
where
    D: Flushable<Color = crate::core::internal::screen::DisplayColor>,
    D::Error: Debug,
{
    trace!("DRAWING: {:?}", screen_state);
    log::info!("DRAWING: {:?}", screen_state);

    // let text = "> B1 Pressed!";
    // draw_text(&mut display, text, true)?;

    // let page_changed = screen_state.changeset.contains(DataSource::RemainingTime);
    // if page_changed {
    //     match super::keepalive::STATE.get() {
    //         RemainingTime::Indefinite => (),
    //         RemainingTime::Duration(time_to_sleep) => {
    //             log::warn!("--> Entering Deep-sleep in {}!!", time_to_sleep);

    //             draw_text(
    //                 &mut display,
    //                 format!("Deep-sleep in:\n{}", time_to_sleep).as_str(),
    //                 true,
    //             )?;
    //         }
    //     }
    // }

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
            screen_state.wiif().as_ref(),
        )?,
        Page::Battery => {
            Battery::draw(&mut display, page_changed, screen_state.battery().as_ref())?
        }
        Page::Wifi => Wifi::draw(&mut display, page_changed, screen_state.wiif().as_ref())?,
    }

    // if let Some((actions, action)) = screen_state.page_actions {
    //     pages::actions::draw(&mut display, actions, action)?;
    // }

    display.flush()?;

    Ok(display)
}
