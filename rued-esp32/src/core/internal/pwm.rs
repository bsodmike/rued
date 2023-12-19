use core::cell::RefCell;
use core::fmt::Debug;

use channel_bridge::Receiver;
use embedded_hal_0_2::PwmPin;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::ledc::LedcDriver;
use serde::{Deserialize, Serialize};

use log::trace;

use embassy_futures::select::{select, select_array, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::utils::asyncify::Unblocker;

use channel_bridge::notification::Notification;

use crate::NvsDataState;

use super::battery::{self, BatteryState};
use super::keepalive::{self, RemainingTime};
use super::state::State;

#[derive(Copy, Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum PwmCommand {
    Initialised,
    SetDutyCycle(u32),
}

#[derive(Default, Clone, Debug, Eq, PartialEq)]
pub struct PwmState {
    duty_cycle: u32,
}

impl PwmState {
    pub const fn new() -> Self {
        Self {
            duty_cycle: DEFAULT_DUTY_CYCLE,
        }
    }
}

pub(crate) static BUTTON1_PRESSED_NOTIF: Notification = Notification::new();
static STATE_PERSIST_NOTIFY: Notification = Notification::new();
static STATE_FLASH_NOTIFY: Notification = Notification::new();

pub const FLASH_WRITE_CYCLE: usize = 20;

pub static STATE: State<PwmCommand> = State::new(
    "PWM",
    PwmCommand::Initialised,
    &[
        &super::screen::PWM_CHANGE_NOTIF,
        &STATE_PERSIST_NOTIFY,
        &STATE_FLASH_NOTIFY,
    ],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, PwmCommand> = Signal::new();

pub const DEFAULT_DUTY_CYCLE: u32 = 50;

pub async fn process<'a>(
    #[allow(unused_mut)] mut pwm: Option<(
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
    )>,
) {
    loop {
        let (future, index) = select_array([COMMAND.wait()]).await;

        #[cfg(feature = "pwm")]
        {
            match index {
                0 => match future {
                    PwmCommand::SetDutyCycle(percentage) => {
                        let (ref mut pwm0, ref mut pwm1, ref mut pwm2) =
                            pwm.as_mut().expect("Unwraps pwm channels");
                        log::info!("PwmCommand::SetDutyCycle: {}", percentage);

                        if percentage > 100 {
                            log::warn!("PWM duty-cycle cannot exceed 100%");
                            return;
                        } else {
                            set_duty(pwm0, percentage);
                            set_duty(pwm1, percentage);
                            set_duty(pwm2, percentage);

                            STATE.update(future);
                        }
                    }
                    PwmCommand::Initialised => {
                        let (ref mut pwm0, ref mut pwm1, ref mut pwm2) =
                            pwm.as_mut().expect("Unwraps pwm channels");

                        set_duty(pwm0, DEFAULT_DUTY_CYCLE);
                        set_duty(pwm1, DEFAULT_DUTY_CYCLE);
                        set_duty(pwm2, DEFAULT_DUTY_CYCLE);
                    }
                },
                _ => unreachable!(),
            }
        }
    }
}

fn set_duty<'a>(pwm: &mut (impl PwmPin<Duty = u32> + 'a), duty_cycle: u32) {
    pwm.disable();
    FreeRtos::delay_ms(500);

    let max_duty = pwm.get_max_duty();

    let ratio = duty_cycle as f32 / 100.00;
    let duty = max_duty as f32 * ratio;
    pwm.set_duty(duty.round() as u32);

    pwm.enable();
}

fn persistable(state: &mut PwmCommand) -> NvsDataState {
    match state {
        PwmCommand::Initialised => NvsDataState::default(),
        PwmCommand::SetDutyCycle(percentage) => NvsDataState {
            pwm_duty_cycle: *percentage,
            ..Default::default()
        },
    }
}

pub async fn persist(mut persister: impl FnMut(NvsDataState)) {
    loop {
        STATE_PERSIST_NOTIFY.wait().await;

        persister(persistable(&mut STATE.get()));
    }
}

/// Only performs flashing once every number of `FLASH_WRITE_CYCLE`s.
pub async fn flash(mut flasher: impl FnMut(NvsDataState)) {
    let mut cycle = 0;

    loop {
        STATE_FLASH_NOTIFY.wait().await;

        if cycle == 0 {
            let data = persistable(&mut STATE.get());

            log::info!("[PWM]: Flashing {:?}", &data);
            flasher(data);
        }

        cycle += 1;

        if cycle >= FLASH_WRITE_CYCLE {
            cycle = 0;
        }
    }
}
