use embassy_time::{Duration, Timer};

use embedded_hal_0_2::adc;
use embedded_hal_0_2::digital::v2::InputPin;

use super::state::State;

pub use super::dto::battery::*;

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

pub async fn process<ADC, BP>(
    mut one_shot: impl adc::OneShot<ADC, u16, BP>,
    mut battery_pin: BP,
    power_pin: impl InputPin,
) where
    BP: adc::Channel<ADC>,
{
    const ROUND_UP: u16 = 50; // TODO: Make it smaller once ADC is connected

    loop {
        Timer::after(Duration::from_secs(2)).await;

        let voltage = one_shot
            .read(&mut battery_pin)
            .ok()
            .map(|voltage| voltage / ROUND_UP * ROUND_UP);

        let powered = Some(power_pin.is_high().unwrap_or(false));

        STATE.update(BatteryState { voltage, powered });
    }
}
