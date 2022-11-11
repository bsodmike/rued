use anyhow::Result;
use esp_idf_hal::i2c;
use esp_idf_hal::units::FromValueType;

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

pub fn display_test<E, T, U>(i2c0: i2c::I2C0, scl: T, sda: U, ip: &str, dns: &str) -> Result<()>
where
    T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
    U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
    E: std::fmt::Debug,
{
    let i2c = i2c::Master::new(
        i2c0,
        i2c::MasterPins {
            scl, // O
            sda, // I+O
        },
        i2c::config::MasterConfig::new().baudrate(400.kHz().into()),
    )?;

    let interface = I2CDisplayInterface::new(i2c);

    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display
        .init()
        .map_err(|e| anyhow::anyhow!("Init error: {:?}", e))?;

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .map_err(|e| anyhow::anyhow!("Txt error: {:?}", e))?;

    Text::with_baseline(
        &format!("IP: {}", ip),
        Point::new(0, 16),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .map_err(|e| anyhow::anyhow!("Txt2 error: {:?}", e))?;

    Text::with_baseline(
        &format!("DNS: {}", dns),
        Point::new(0, 32),
        text_style,
        Baseline::Top,
    )
    .draw(&mut display)
    .map_err(|e| anyhow::anyhow!("Txt3 error: {:?}", e))?;

    display
        .flush()
        .map_err(|e| anyhow::anyhow!("Flush error: {:?}", e))?;

    Ok(())
}
