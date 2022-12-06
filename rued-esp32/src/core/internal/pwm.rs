use core::cell::RefCell;
use core::fmt::Debug;

use channel_bridge::Receiver;
use embedded_hal_0_2::PwmPin;
use esp_idf_hal::delay::FreeRtos;
use serde::{Deserialize, Serialize};

use log::trace;

use embassy_futures::select::{select, select_array, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;

use embedded_svc::executor::asynch::Unblocker;

use channel_bridge::notification::Notification;

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

pub static STATE: State<PwmCommand> = State::new(
    "PWM COMMAND",
    PwmCommand::Initialised,
    &[&super::screen::PWM_CHANGE_NOTIF],
);

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, PwmCommand> = Signal::new();

pub const DEFAULT_DUTY_CYCLE: u32 = 50;

pub async fn process<'a>(
    pwm: (
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
    ),
) {
    let (mut pwm0, mut pwm1, mut pwm2) = pwm;

    set_duty(&mut pwm0, DEFAULT_DUTY_CYCLE);
    set_duty(&mut pwm1, DEFAULT_DUTY_CYCLE);
    set_duty(&mut pwm2, DEFAULT_DUTY_CYCLE);

    loop {
        let (future, index) = select_array([COMMAND.wait()]).await;

        {
            match index {
                0 => match future {
                    PwmCommand::SetDutyCycle(percentage) => {
                        log::info!("PwmCommand::SetDutyCycle: {}", percentage);

                        if percentage > 100 {
                            log::warn!("PWM duty-cycle cannot exceed 100%");
                            return;
                        }

                        set_duty(&mut pwm0, percentage);
                        set_duty(&mut pwm1, percentage);
                        set_duty(&mut pwm2, percentage);

                        STATE.update(future);
                    }
                    PwmCommand::Initialised => {}
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
