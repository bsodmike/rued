use core::fmt::Debug;

use embedded_graphics::pixelcolor::BinaryColor;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_0_2::{adc, PwmPin};
use embedded_hal_async::digital::Wait;

use embedded_svc::mqtt::client::asynch::{Client, Connection, Publish};
use embedded_svc::ws::asynch::server::Acceptor;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::netif::IpEvent;
use esp_idf_svc::wifi::{EspWifi, WifiEvent};

use gfx_xtra::draw_target::Flushable;

use edge_executor::*;

use channel_bridge::asynch::*;

use crate::core::internal::mqtt::MqttCommand;
use crate::core::internal::screen;
use crate::models::rtc_external::RtcExternal;
use crate::services::httpd::LazyInitHttpServer;
use crate::MQTT_MAX_TOPIC_LEN;

use futures::task::SpawnError;

use super::button::{self, PressedLevel};
use super::screen::Color;
use super::web::{self, WebEvent, WebRequest};
use super::{battery, mqtt, wifi};

pub fn high_prio<'a, const C: usize>(
    executor: &LocalExecutor<'a, C>,
    button1_pin: impl InputPin<Error = impl Debug + 'a> + Wait + 'a,
    battery_voltage: impl crate::core::internal::battery::Adc + 'a,
    power_pin: impl InputPin + 'a,
    // display: D,
    httpd: &'a mut LazyInitHttpServer<'static>,
    pwm: Option<(
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
        impl PwmPin<Duty = u32> + 'a,
    )>,
    rtc: Option<impl RtcExternal + 'a>,
    pwm_flash: impl FnMut(crate::NvsDataState) + 'a,
    netif_notifier: impl Receiver<Data = IpEvent> + 'a,
    mqtt_topic_prefix: &'a str,
) {
    executor
        .spawn(button::button1_process(button1_pin, PressedLevel::Low))
        .detach();
    executor.spawn(super::inspector::process()).detach();
    executor.spawn(super::keepalive::process()).detach();
    // executor.spawn(screen::process()).detach();
    // executor.spawn(screen::run_draw(display)).detach();

    // FIXME
    executor.spawn(super::httpd::process(httpd)).detach();

    executor.spawn(super::pwm::process(pwm)).detach();
    executor.spawn(super::sntp::process()).detach();
    executor.spawn(super::pwm::flash(pwm_flash)).detach();
    executor
        .spawn(super::battery::process(battery_voltage, power_pin))
        .detach();
    // Netif State Change
    executor
        .spawn(crate::process_netif_state_change(netif_notifier))
        .detach();
    // OTA
    executor.spawn(super::ota::ota_task()).detach();

    if let Some(rtc) = rtc {
        executor.spawn(super::external_rtc::process(rtc)).detach();
    };
}

// FIXME
// pub fn high_prio_original<'a, ADC, BP, const C: usize, M>(
//     executor: &mut Executor<'a, C, M, Local>,
//     tasks: &mut heapless::Vec<Task<()>, C>,
//     battery_voltage: impl adc::OneShot<ADC, u16, BP> + 'a,
//     battery_pin: BP,
//     // used to indicate if powered by battery, only if this is enabled will Deep-sleep be enabled.
//     power_pin: impl InputPin + 'a,
//     roller: bool,
//     button1_pin: impl InputPin<Error = impl Debug + 'a> + 'a,
//     button2_pin: impl InputPin<Error = impl Debug + 'a> + 'a,
//     button3_pin: impl InputPin<Error = impl Debug + 'a> + 'a,
// ) -> Result<(), SpawnError>
// where
//     M: Monitor + Default,
//     ADC: 'a,
//     BP: adc::Channel<ADC> + 'a,
// {
//     executor
//         .spawn_local_collect(
//             battery::process(battery_voltage, battery_pin, power_pin),
//             tasks,
//         )?
//         .spawn_local_collect(
//             super::button::button3_process(button3_pin, super::button::PressedLevel::Low),
//             tasks,
//         )?
//         // .spawn_local_collect(emergency::process(), tasks)?
//         .spawn_local_collect(super::keepalive::process(), tasks)?;

//     if roller {
//         executor.spawn_local_collect(
//             super::button::button1_button2_roller_process(button1_pin, button2_pin),
//             tasks,
//         )?;
//     } else {
//         executor
//             .spawn_local_collect(
//                 super::button::button1_process(button1_pin, super::button::PressedLevel::Low),
//                 tasks,
//             )?
//             .spawn_local_collect(
//                 super::button::button2_process(button2_pin, super::button::PressedLevel::Low),
//                 tasks,
//             )?;
//     }

//     Ok(())
// }

pub fn mid_prio<'a, const C: usize, D>(
    executor: &LocalExecutor<'a, C>,
    display: D,
    // wm_flash: impl FnMut(WaterMeterState) + 'a,
) where
    D: Flushable<Color = crate::core::internal::screen::DisplayColor> + 'a,
    D::Error: Debug,
{
    executor.spawn(screen::process()).detach();
    executor.spawn(screen::run_draw(display)).detach();
    // executor.spawn(wm_stats::process()).detach();
    // executor.spawn(wm::flash(wm_flash)).detach();
}

pub fn wifi<'a, const C: usize, D>(
    executor: &LocalExecutor<'a, C>,
    wifi: impl embedded_svc::wifi::asynch::Wifi + 'a,
    wifi_notif: impl Receiver<Data = D> + 'a,
) -> Result<(), SpawnError>
where
    D: 'a,
    WifiEvent: From<D>,
{
    executor.spawn(wifi::process(wifi, wifi_notif)).detach();

    Ok(())
}

pub fn mqtt_send<'a, const L: usize, const C: usize>(
    executor: &LocalExecutor<'a, C>,
    mqtt_topic_prefix: &'a str,
    mqtt_client: impl Client + Publish + 'a,
) {
    executor
        .spawn(mqtt::send::<L>(mqtt_topic_prefix, mqtt_client))
        .detach();
}

pub fn mqtt_receive<const C: usize>(
    executor: &LocalExecutor<'_, C>,
    mqtt_conn: impl for<'a> Connection<Message<'a> = Option<MqttCommand>> + 'static,
) {
    executor.spawn(mqtt::receive(mqtt_conn)).detach();
}

pub fn ws<'a, const C: usize>(
    executor: &LocalExecutor<'a, C>,
    acceptor: impl Acceptor + 'a,
) -> Result<(), SpawnError> {
    executor
        .spawn(crate::core::internal::ws::process(acceptor))
        .detach();

    Ok(())
}

// pub fn run<const C: usize>(executor: &LocalExecutor<'_, C>)
// {
//     // FIXME: Simulate a button press
//     // let condition = move || false;

//     let condition = move || !super::quit::QUIT.triggered();
//     executor.run_tasks(condition, tasks);
//     // executor.spawn(ws::process(acceptor)).detach();
// }

// pub fn start<const C: usize, M, F>(executor: &LocalExecutor<'a, C>, finished: F)
// where
//     M: Monitor + Start + Default,
//     F: FnOnce() + 'static,
// {
//     executor.start(move || !super::quit::QUIT.triggered(), {
//         let executor = &*executor;

//         move || {
//             executor.drop_tasks(tasks);

//             finished();
//         }
//     });
// }
