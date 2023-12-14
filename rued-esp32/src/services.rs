use core::cell::RefCell;
use core::fmt::Debug;
use core::mem;
use std::sync::Arc;

extern crate alloc;

// use edge_frame::assets::serve::AssetMetadata;

use embassy_sync::blocking_mutex::Mutex;
use embassy_time::Duration;

use embedded_hal::digital::OutputPin as EHOutputPin;

use embedded_hal::spi::SpiDevice;
use esp_idf_svc::tls;

use embedded_svc::http::server::Method;
use embedded_svc::mqtt::client::asynch::{Client, Connection, Publish};
use embedded_svc::utils::asyncify::Asyncify;
use embedded_svc::wifi::asynch::Wifi;
use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
use embedded_svc::ws::asynch::server::Acceptor;

use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::adc::{Adc, AdcChannelDriver, AdcConfig, AdcDriver};
use esp_idf_svc::hal::delay;
use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::gpio::*;
use esp_idf_svc::hal::modem::WifiModemPeripheral;
use esp_idf_svc::hal::peripheral::Peripheral;
use esp_idf_svc::hal::prelude::*;
use esp_idf_svc::hal::reset::WakeupReason;
use esp_idf_svc::hal::spi::*;
use esp_idf_svc::hal::task::embassy_sync::EspRawMutex;

use esp_idf_svc::http::server::ws::EspHttpWsProcessor;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::mqtt::client::{EspMqttClient, MqttClientConfiguration};
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::timer::EspTaskTimerService;
use esp_idf_svc::wifi::{AsyncWifi, BlockingWifi, EspWifi};

use esp_idf_svc::sys::{adc_atten_t, EspError};

// use esp_idf_hal::gpio::*;
// use esp_idf_hal::i2c::{I2c, I2cConfig, I2cDriver};
// use esp_idf_hal::modem::WifiModemPeripheral;
// use esp_idf_hal::peripheral::Peripheral;
// use esp_idf_hal::prelude::*;
// use esp_idf_hal::reset::WakeupReason;
// use esp_idf_hal::spi::config::Duplex;
// use esp_idf_hal::spi::*;
// use esp_idf_hal::task::embassy_sync::EspRawMutex;
// use esp_idf_svc::hal::delay;
// use esp_idf_svc::hal::delay::FreeRtos;

use futures::task::SpawnError;
use futures::Future;

use gfx_xtra::draw_target::{Flushable, OwnedDrawTargetExt};

use edge_executor::*;
use shared_bus::BusManager;

use crate::core::internal::mqtt::{MessageParser, MqttCommand};

use crate::core::internal::pulse_counter::PulseCounter;
use crate::core::internal::pulse_counter::PulseWakeup;
use crate::core::internal::ws;

use crate::core::internal::screen::Color;
// use ruwm::button::PressedLevel;
// use ruwm::pulse_counter::PulseCounter;
// use ruwm::pulse_counter::PulseWakeup;
// use ruwm::screen::{Color, Flushable, OwnedDrawTargetExt};
// use ruwm::valve::{self, ValveState};
// use ruwm::wm::WaterMeterState;
// use ruwm::wm_stats::WaterMeterStatsState;
// use ruwm::ws;

use channel_bridge::{asynch::pubsub, asynch::*, notification::Notification};

use crate::peripherals::{
    I2c0Peripherals, PulseCounterPeripherals, SpiBusPeripherals, ValvePeripherals, SPI_BUS_FREQ,
};
use crate::{errors::*, peripherals};

// Display
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};

#[cfg(feature = "display-i2c")]
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

use self::httpd::LazyInitHttpServer;

pub mod httpd;

const WIFI_SSID: &str = env!("WIFI_SSID");
const WIFI_PSK: &str = env!("WIFI_PSK");
const WIFI_START_TIMEOUT: std::time::Duration = std::time::Duration::from_secs(20);
const MQTT_HOST: &str = env!("MQTT_HOST");

// const ASSETS: assets::serve::Assets = edge_frame::assets!("RUWM_WEB");

// #[derive(Default)]
// pub struct RtcMemory {
//     pub valve: Option<ValveState>,
//     pub wm: WaterMeterState,
//     pub wm_stats: WaterMeterStatsState,
// }

// impl RtcMemory {
//     pub const fn new() -> Self {
//         Self {
//             valve: None,
//             wm: WaterMeterState::new(),
//             wm_stats: WaterMeterStatsState::new(),
//         }
//     }
// }

// #[cfg_attr(feature = "rtc-mem", link_section = ".rtc.data.rtc_memory")]
// pub static mut RTC_MEMORY: RtcMemory = RtcMemory::new();

// pub fn valve_pins(
//     peripherals: ValvePeripherals,
//     wakeup_reason: WakeupReason,
// ) -> Result<
//     (
//         impl EHOutputPin<Error = impl Debug>,
//         impl EHOutputPin<Error = impl Debug>,
//         impl EHOutputPin<Error = impl Debug>,
//     ),
//     EspError,
// > {
//     let mut power = PinDriver::input_output(peripherals.power)?;
//     let mut open = PinDriver::input_output(peripherals.open)?;
//     let mut close = PinDriver::input_output(peripherals.close)?;

//     power.set_pull(Pull::Floating)?;
//     open.set_pull(Pull::Floating)?;
//     close.set_pull(Pull::Floating)?;

//     if wakeup_reason == WakeupReason::ULP {
//         // valve::emergency_close(&mut power, &mut open, &mut close, &mut FreeRtos);
//     }

//     Ok((power, open, close))
// }

#[cfg(feature = "nvs")]
pub fn storage(
    partition: EspDefaultNvsPartition,
) -> Result<
    &'static Mutex<
        impl embassy_sync::blocking_mutex::raw::RawMutex,
        RefCell<impl embedded_svc::storage::Storage>,
    >,
    InitError,
> {
    const POSTCARD_BUF_SIZE: usize = 500;

    struct PostcardSerDe;

    impl embedded_svc::storage::SerDe for PostcardSerDe {
        type Error = postcard::Error;

        fn serialize<'a, T>(&self, slice: &'a mut [u8], value: &T) -> Result<&'a [u8], Self::Error>
        where
            T: serde::Serialize,
        {
            postcard::to_slice(value, slice).map(|r| &*r)
        }

        fn deserialize<T>(&self, slice: &[u8]) -> Result<T, Self::Error>
        where
            T: serde::de::DeserializeOwned,
        {
            postcard::from_bytes(slice)
        }
    }

    static STORAGE: static_cell::StaticCell<
        Mutex<
            EspRawMutex,
            RefCell<
                embedded_svc::storage::StorageImpl<
                    { POSTCARD_BUF_SIZE },
                    esp_idf_svc::nvs::EspDefaultNvs,
                    PostcardSerDe,
                >,
            >,
        >,
    > = static_cell::StaticCell::new();

    let storage = &*STORAGE.init(Mutex::new(RefCell::new(
        embedded_svc::storage::StorageImpl::new(
            esp_idf_svc::nvs::EspNvs::new(partition, "DEFAULT", true)?,
            PostcardSerDe,
        ),
    )));

    Ok(storage)
}

#[cfg(not(feature = "ulp"))]
pub fn pulse(
    peripherals: PulseCounterPeripherals<impl RTCPin + InputPin + OutputPin>,
) -> Result<(impl PulseCounter, impl PulseWakeup), InitError> {
    static PULSE_SIGNAL: Notification = Notification::new();

    let pulse_counter = crate::core::internal::pulse_counter::CpuPulseCounter::new(
        subscribe_pin(peripherals.pulse, || PULSE_SIGNAL.notify())?,
        crate::core::internal::button::PressedLevel::Low,
        Some(Duration::from_millis(50)),
    );

    Ok((pulse_counter, ()))
}

#[cfg(feature = "ulp")]
pub fn pulse(
    peripherals: PulseCounterPeripherals<impl RTCPin + InputPin + OutputPin>,
    wakeup_reason: WakeupReason,
) -> Result<(impl PulseCounter, impl PulseWakeup), InitError> {
    let mut pulse_counter = ulp_pulse_counter::UlpPulseCounter::new(
        esp_idf_hal::ulp::UlpDriver::new(ulp)?,
        peripherals.pulse,
        wakeup_reason == WakeupReason::Unknown,
    )?;

    //let (pulse_counter, pulse_wakeup) = pulse_counter.split();

    Ok((pulse_counter, ()))
}

pub fn button<'d, P: InputPin + OutputPin>(
    pin: impl Peripheral<P = P> + 'd,
    notification: &'static Notification,
) -> Result<impl embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait + 'd, InitError>
{
    let resp = subscribe_pin(pin, move || notification.notify())?;

    Ok(resp)
}

#[cfg(feature = "display-i2c")]
pub fn display(
    i2c: impl embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_Write
        + embedded_hal_0_2::prelude::_embedded_hal_blocking_i2c_WriteRead
        + 'static,
) -> Result<impl Flushable<Color = BinaryColor, Error = impl Debug + 'static> + 'static, InitError>
{
    #[cfg(feature = "ssd1306")]
    let display = {
        let interface = I2CDisplayInterface::new_custom_address(i2c, 0x3C);

        let mut display = Ssd1306::new(interface, DisplaySize128x32, DisplayRotation::Rotate0)
            .into_buffered_graphics_mode();
        display.init().unwrap();
        display.clear();
        display.flush().unwrap();

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_9X15)
            .text_color(BinaryColor::On)
            .build();

        Text::with_baseline(
            "[OK] System\n     online.",
            Point::new(0, 0),
            text_style,
            Baseline::Top,
        )
        .draw(&mut display)
        .unwrap();

        display
    };

    let mut display = display
        .owned_flushing(|target| {
            target.flush().expect("Flushing the target display.");

            Ok(())
        })
        .owned_color_converted();

    display.flush().expect("Flushing the target display.");

    Ok(display)
}

#[cfg(not(feature = "display-i2c"))]
pub fn display<'a>(
    peripherals: peripherals::DisplayPeripherals,
    spi_bus_peripherals: SpiBusPeripherals,
) -> Result<impl Flushable<Color = Color, Error = impl Debug + 'static> + 'static, InitError> {
    if let Some(backlight) = peripherals.control.backlight {
        let mut backlight = PinDriver::output(backlight)?;

        // NOTE: use lowest drive strength as P-channel MOSFET used to drive
        // the backlight https://www.sparkfun.com/products/16653
        backlight.set_drive_strength(DriveStrength::I5mA)?;
        backlight.set_high()?;

        mem::forget(backlight); // TODO: For now
    }

    let spi = SpiDeviceDriver::new(
        spi_bus_peripherals.driver,
        spi_bus_peripherals.cs,
        &SpiConfig::default().baudrate(SPI_BUS_FREQ.MHz().into()),
    )?;

    let dc = PinDriver::output(peripherals.control.dc)?;

    #[cfg(any(feature = "ili9342", feature = "st7789"))]
    let display = {
        let rst = PinDriver::output(peripherals.control.rst)?;

        #[cfg(feature = "ili9342")]
        let builder = mipidsi::Builder::ili9342c_rgb666(
            display_interface_spi::SPIInterfaceNoCS::new(spi, dc),
        );

        #[cfg(feature = "st7789")]
        let builder =
            mipidsi::Builder::st7789(display_interface_spi::SPIInterfaceNoCS::new(spi, dc));

        builder
            .with_display_size(240, 320)
            .with_framebuffer_size(240, 320)
            .with_color_order(mipidsi::ColorOrder::Bgr)
            .with_orientation(mipidsi::Orientation::PortraitInverted(true))
            .init(&mut delay::Ets, Some(rst))
            .unwrap()
    };

    #[cfg(feature = "ssd1351")]
    let display = {
        use ssd1351::mode::displaymode::DisplayModeTrait;

        let mut display =
            ssd1351::mode::graphics::GraphicsMode::new(ssd1351::display::Display::new(
                ssd1351::interface::spi::SpiInterface::new(spi, dc),
                ssd1351::properties::DisplaySize::Display128x128,
                ssd1351::properties::DisplayRotation::Rotate0,
            ));

        display
            .reset(
                &mut PinDriver::output(peripherals.control.rst)?,
                &mut delay::Ets,
            )
            .unwrap();

        display
    };

    #[cfg(feature = "ttgo")]
    let mut display = {
        let rect = embedded_graphics::primitives::Rectangle::new(
            embedded_graphics::prelude::Point::new(52, 40),
            embedded_graphics::prelude::Size::new(135, 240),
        );

        display.owned_cropped(display, &rect)
    };

    let display = display.owned_noop_flushing().owned_color_converted();

    Ok(display)
}

// TODO: Make it async
pub fn wifi<'d>(
    modem: impl Peripheral<P = impl WifiModemPeripheral + 'd> + 'd,
    mut sysloop: EspSystemEventLoop,
    timer_service: EspTaskTimerService,
    partition: Option<EspDefaultNvsPartition>,
    auth_method: AuthMethod,
) -> Result<
    (
        impl embedded_svc::wifi::asynch::Wifi + 'd,
        impl Receiver<Data = esp_idf_svc::wifi::WifiEvent>,
    ),
    InitError,
> {
    let mut wifi = EspWifi::new(modem, sysloop.clone(), partition)?;

    if WIFI_PSK.is_empty() {
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: WIFI_SSID.into(),
            auth_method: AuthMethod::None,
            ..Default::default()
        }))?;
    } else {
        wifi.set_configuration(&Configuration::Client(ClientConfiguration {
            ssid: WIFI_SSID.into(),
            password: WIFI_PSK.into(),
            auth_method,
            ..Default::default()
        }))?;
    }

    let mut bwifi = BlockingWifi::wrap(&mut wifi, sysloop.clone())?;

    bwifi.start()?;

    bwifi.connect()?;

    bwifi.wait_netif_up()?;

    let wifi = AsyncWifi::wrap(wifi, sysloop.clone(), timer_service)?;

    Ok((
        wifi,
        pubsub::SvcReceiver::new(sysloop.as_async().subscribe()?),
    ))
}

pub fn httpd<'a>() -> Result<LazyInitHttpServer<'a>, InitError> {
    let server_certificate = tls::X509::pem_until_nul(include_bytes!(
        "/home/mike/esp/openssl-generate-rs/output/cert.pem"
    ));
    let server_private_key = tls::X509::pem_until_nul(include_bytes!(
        "/home/mike/esp/openssl-generate-rs/output/cert_key.pem"
    ));

    let mut config = esp_idf_svc::http::server::Configuration::default();
    config.server_certificate = Some(server_certificate);
    config.private_key = Some(server_private_key);

    let httpd = LazyInitHttpServer::new(config);

    Ok(httpd)
}

pub fn mqtt() -> Result<
    (
        &'static str,
        impl Client + Publish,
        impl for<'a> Connection<Message<'a> = Option<MqttCommand>> + 'static,
    ),
    InitError,
> {
    let client_id = "rued";
    let mut mqtt_parser = MessageParser::new();

    let (mqtt_client, mqtt_conn) = EspMqttClient::new_with_converting_async_conn(
        format!("mqtt://{}:1883", MQTT_HOST).as_str(),
        &MqttClientConfiguration {
            client_id: Some(client_id),
            ..Default::default()
        },
        move |event| mqtt_parser.convert(event),
    )?;

    let mqtt_client = mqtt_client.into_async();

    Ok((client_id, mqtt_client, mqtt_conn))
}

pub fn adc<'d, const A: adc_atten_t, ADC: Adc + 'd, P: ADCPin<Adc = ADC>>(
    adc: impl Peripheral<P = ADC> + 'd,
    pin: impl Peripheral<P = P> + 'd,
) -> Result<impl crate::core::internal::battery::Adc + 'd, InitError> {
    struct AdcImpl<'d, const A: adc_atten_t, ADC, V>
    where
        ADC: Adc,
        V: ADCPin<Adc = ADC>,
    {
        driver: AdcDriver<'d, ADC>,
        channel_driver: AdcChannelDriver<'d, A, V>,
    }

    impl<'d, const A: adc_atten_t, ADC, V> crate::core::internal::battery::Adc
        for AdcImpl<'d, A, ADC, V>
    where
        ADC: Adc,
        V: ADCPin<Adc = ADC>,
    {
        type Error = esp_idf_svc::sys::EspError;

        async fn read(&mut self) -> Result<u16, Self::Error> {
            self.driver.read(&mut self.channel_driver)
        }
    }

    Ok(AdcImpl {
        driver: AdcDriver::new(adc, &AdcConfig::new().calibration(true))?,
        channel_driver: AdcChannelDriver::<{ A }, _>::new(pin)?,
    })
}

fn subscribe_pin<'d, P: InputPin + OutputPin>(
    pin: impl Peripheral<P = P> + 'd,
    notify: impl Fn() + Send + 'static,
) -> Result<impl embedded_hal::digital::InputPin + embedded_hal_async::digital::Wait + 'd, InitError>
{
    let mut pin = PinDriver::input(pin)?;

    pin.set_interrupt_type(InterruptType::NegEdge)?;

    unsafe {
        pin.subscribe(notify)?;
    }

    Ok(pin)
}

pub fn schedule<'a, const C: usize>(
    stack_size: usize,
    run: impl Future + Send + 'static,
    spawner: impl FnOnce() -> LocalExecutor<'a, C> + Send + 'static,
) -> std::thread::JoinHandle<()> {
    std::thread::Builder::new()
        .stack_size(stack_size)
        .spawn(move || {
            let executor = spawner();

            // info!(
            //     "Tasks on thread {:?} scheduled, about to run the executor now",
            //     "TODO"
            // );

            block_on(executor.run(run));
        })
        .unwrap()
}
