// use anyhow::Result;

// use embedded_graphics::{
//     mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
//     pixelcolor::BinaryColor,
//     prelude::*,
//     text::{Baseline, Text},
// };
// use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};

// pub fn display_test(driver: crate::I2cDriverType, ip: &str, dns: &str) -> Result<()> {
//     let i2c = driver;
//     let interface = I2CDisplayInterface::new(i2c);

//     let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
//         .into_buffered_graphics_mode();
//     display
//         .init()
//         .map_err(|e| anyhow::anyhow!("Init error: {:?}", e))?;

//     let text_style = MonoTextStyleBuilder::new()
//         .font(&FONT_6X10)
//         .text_color(BinaryColor::On)
//         .build();

//     Text::with_baseline("Hello Rust!", Point::zero(), text_style, Baseline::Top)
//         .draw(&mut display)
//         .map_err(|e| anyhow::anyhow!("Txt error: {:?}", e))?;

//     Text::with_baseline(
//         &format!("IP: {}", ip),
//         Point::new(0, 16),
//         text_style,
//         Baseline::Top,
//     )
//     .draw(&mut display)
//     .map_err(|e| anyhow::anyhow!("Txt2 error: {:?}", e))?;

//     Text::with_baseline(
//         &format!("DNS: {}", dns),
//         Point::new(0, 32),
//         text_style,
//         Baseline::Top,
//     )
//     .draw(&mut display)
//     .map_err(|e| anyhow::anyhow!("Txt3 error: {:?}", e))?;

//     display
//         .flush()
//         .map_err(|e| anyhow::anyhow!("Flush error: {:?}", e))?;

//     Ok(())
// }
