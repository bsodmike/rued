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
            (profont::PROFONT_9_POINT, 12, 5)
        } else {
            (profont::PROFONT_14_POINT, 20, 2)
        };

        if let Some(state) = state {
            let mut status_rt = shapes::Textbox {
                text: "            ",
                color: super::super::super::screen::DISPLAY_COLOR_GREEN,
                font: status_font,
                padding: 1,
                outline: 0,
                strikethrough: false,
                ..Default::default()
            };

            let mut ip = String::default();
            let mut dns = String::default();
            if let Some(conn) = state {
                ip = conn.ip();
                dns = conn.dns();
            };

            let mut text_buf = heapless::String::<41>::new();
            write!(&mut text_buf, "ip: {}\ndns: {}", ip, dns).unwrap();
            status_rt.text = &text_buf;

            let status_rt_size = status_rt.preferred_size();

            status_rt.draw(&mut target.cropped(&Rectangle::new(
                bbox.top_left + Size::new(0, bbox.size.height - status_rt_size.height),
                status_rt_size,
            )))?;
        }

        Ok(())
    }
}
