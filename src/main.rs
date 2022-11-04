use esp_idf_sys as _; // If using the `binstart` feature of `esp-idf-sys`, always keep this module imported
use std::time::Duration;

use embedded_hal::digital::v2::ToggleableOutputPin;
use esp_idf_hal::{
    peripherals::Peripherals,
    units::FromValueType,
    i2c::{self, MasterPins}
};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{
    prelude::*,
    I2CDisplayInterface,
    Ssd1306,
};

fn main() {
    esp_idf_sys::link_patches();

    //  configure peripherals
    let peripherals = Peripherals::take().unwrap();
    let mut led = peripherals.pins.gpio17.into_output().unwrap();
    let mut led_onboard = peripherals.pins.gpio18.into_output().unwrap();
    let sda = peripherals.pins.gpio4.into_input_output().unwrap();
    let scl = peripherals.pins.gpio5.into_output().unwrap();

    //  configure i2c
    let _cfg = i2c::config::MasterConfig::new()
        .baudrate(400.kHz().into());
    let i2c = i2c::Master::new(
        peripherals.i2c0,
        MasterPins { sda, scl },
        i2c::config::MasterConfig::new()
        .baudrate(400.kHz().into())
    ).unwrap();

    //  configure OLED display
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    Text::with_baseline("Hello world!", Point::zero(), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    Text::with_baseline("Hello Rust!", Point::new(0, 16), text_style, Baseline::Top)
        .draw(&mut display)
        .unwrap();

    display.flush().unwrap();

    // heart-beat sequence
    for i in 0..20 {
        println!("Toggling LED now: {}", i);
        led.toggle().unwrap();
        led_onboard.toggle().unwrap();

        std::thread::sleep(Duration::from_millis(500));
    }

    loop {}
}
