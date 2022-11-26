use crate::core::internal::screen::shapes::{self};
use crate::core::internal::wifi::WifiConnection;
use core::fmt::Write;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font;
use embedded_graphics::prelude::{DrawTargetExt, Size};
use embedded_graphics::primitives::Rectangle;

use super::with_title;

pub struct Wifi;

impl Wifi {
    pub fn draw<T>(
        target: &mut T,
        page_changed: bool,
        state: Option<&Option<WifiConnection>>,
    ) -> Result<(), T::Error>
    where
        T: DrawTarget<Color = super::super::super::screen::DisplayColor>,
    {
        let bbox = target.bounding_box();

        let Size { width, .. } = bbox.size;

        let mut target = with_title(target, page_changed, "Wifi")?;

        let (status_font, status_height, status_padding) = if width <= 128 {
            (mono_font::ascii::FONT_9X15, 12, 5)
        } else {
            (mono_font::ascii::FONT_9X18, 20, 2)
        };

        if let Some(state) = state {
            let mut status_rt = shapes::Textbox {
                text: "            ",
                color: super::super::super::screen::DISPLAY_COLOR_YELLOW, // Color::Yellow
                font: status_font,
                padding: 1,
                outline: 0,
                strikethrough: false,
                ..Default::default()
            };
            let status_rt_size = status_rt.preferred_size();

            let mut ip = String::default();
            if let Some(conn) = state {
                ip = conn.ip();
            };

            let mut text_buf = heapless::String::<14>::new();
            write!(&mut text_buf, "ip: {}", ip).unwrap();

            status_rt.text = &text_buf;

            status_rt.draw(&mut target.cropped(&Rectangle::new(
                bbox.top_left + Size::new(0, bbox.size.height - status_rt_size.height),
                status_rt_size,
            )))?;
        }

        Ok(())
    }
}
