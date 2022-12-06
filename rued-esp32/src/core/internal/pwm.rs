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

#[derive(PartialEq, Debug, Serialize, Deserialize)]
pub enum PwmCommand {
    SetDutyCycle(u32),
}

#[derive(Default, Clone, Debug, Eq, PartialEq)]
pub struct PwmState {}

impl PwmState {
    pub const fn new() -> Self {
        Self {}
    }
}

pub(crate) static BUTTON1_PRESSED_NOTIF: Notification = Notification::new();

static STATE: Mutex<CriticalSectionRawMutex, RefCell<PwmState>> =
    Mutex::new(RefCell::new(PwmState::new()));

pub(crate) static COMMAND: Signal<CriticalSectionRawMutex, PwmCommand> = Signal::new();

pub async fn process<'a>(
    pwm: (
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
    ),
) {
    let (mut pwm0, mut pwm1, mut pwm2) = pwm;

    set_duty(&mut pwm0, 2);
    set_duty(&mut pwm1, 2);
    set_duty(&mut pwm2, 2);

    loop {
        let (future, index) = select_array([COMMAND.wait()]).await;

        {
            match index {
                0 => match future {
                    PwmCommand::SetDutyCycle(percentage) => {
                        log::info!("PwmCommand::SetDutyCycle: {}", percentage);

                        set_duty(&mut pwm0, percentage);
                        set_duty(&mut pwm1, percentage);
                        set_duty(&mut pwm2, percentage);
                    }
                },
                _ => unreachable!(),
            }
        }
    }
}

fn set_duty<'a>(pwm: &mut (impl PwmPin<Duty = u32> + 'a), divisor: u32) {
    pwm.disable();

    FreeRtos::delay_ms(500);

    let duty = pwm.get_max_duty();
    pwm.set_duty(duty / divisor);

    pwm.enable();
}
