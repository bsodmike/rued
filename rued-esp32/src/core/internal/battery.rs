use embassy_time::{Duration, Timer};

use core::fmt::Debug;
use embedded_hal_0_2::adc;
use embedded_hal_0_2::digital::v2::InputPin;

use super::state::State;

pub use super::dto::battery::*;

pub trait Adc {
    type Error: Debug;

    async fn read(&mut self) -> Result<u16, Self::Error>;
}

pub static STATE: State<BatteryState> = State::new(
    "BATTERY",
    BatteryState::new(),
    &[
        // &crate::keepalive::NOTIF,
        // &crate::emergency::BATTERY_STATE_NOTIF,
        &super::screen::BATTERY_STATE_NOTIF,
        // &super::mqtt::BATTERY_STATE_NOTIF,
        // &crate::web::BATTERY_STATE_NOTIF,
    ],
);

pub async fn process(mut battery_adc: impl Adc, power_pin: impl InputPin) {
    const ROUND_UP: u16 = 50; // TODO: Make it smaller once ADC is connected

    loop {
        Timer::after(Duration::from_secs(2)).await;

        let voltage = battery_adc
            .read()
            .await
            .ok()
            .map(|voltage| voltage / ROUND_UP * ROUND_UP);

        let powered = Some(power_pin.is_high().unwrap_or(false));

        STATE.update(BatteryState { voltage, powered });
    }
}
