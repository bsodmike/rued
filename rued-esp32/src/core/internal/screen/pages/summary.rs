use core::{cmp::min, fmt::Write};
use std::fmt::Display;

use embedded_graphics::mono_font;
use embedded_graphics::pixelcolor::BinaryColor;
use embedded_graphics::{
    draw_target::DrawTarget,
    prelude::{DrawTargetExt, Point, Size},
    primitives::Rectangle,
};

use gfx_xtra::draw_target::{DrawTargetExt2, RotateAngle};

use crate::core::internal::battery::BatteryState;
use crate::core::internal::external_rtc::RtcExternalState;
use crate::core::internal::keepalive::RemainingTime;
use crate::core::internal::screen::shapes::{self, BatteryChargedText, Color};
use crate::core::internal::wifi::WifiConnection;
// use crate::valve::ValveState;
// use crate::wm::WaterMeterState;

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
pub struct MeterState {
    pub edges_count: u64,
}

impl MeterState {
    pub const fn new() -> Self {
        Self { edges_count: 0 }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Default)]
pub struct CurrentTime {
    pub text: String,
}

impl CurrentTime {
    pub fn new() -> Self {
        Self {
            text: String::from(""),
        }
    }
}

impl Display for CurrentTime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.text)
    }
}

pub struct Summary;

impl Summary {
    pub fn draw<D>(
        target: &mut D,
        _page_changed: bool,
        // valve_state: Option<&Option<ValveState>>,
        // wm_state: Option<&WaterMeterState>,
        battery_state: Option<&BatteryState>,
        remaining_time_state: Option<&RemainingTime>,
        ip_addr: Option<&Option<WifiConnection>>,
        ext_rtc: Option<&RtcExternalState>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = super::super::super::screen::DisplayColor>,
    {
        let bbox = target.bounding_box();

        let top_height = Self::draw_top_status_line(target, battery_state)?;
        let bottom_height = Self::draw_bottom_status_line(target, remaining_time_state, ip_addr)?;

        let content_rect = Rectangle::new(
            bbox.top_left + Size::new(0, top_height + 5),
            bbox.size - Size::new(0, top_height + bottom_height + 5),
        );

        let meter_state = MeterState { edges_count: 88888 };
        let local_time = CurrentTime {
            text: String::from("local time"),
        };

        Self::draw_content(
            &mut target.cropped(&content_rect),
            Some(&meter_state),
            ext_rtc,
        )?;

        Ok(())
    }

    fn draw_top_status_line<D>(
        target: &mut D,
        battery_state: Option<&BatteryState>,
    ) -> Result<u32, D::Error>
    where
        D: DrawTarget<Color = super::super::super::screen::DisplayColor>,
    {
        let bbox = target.bounding_box();

        let Size { width, .. } = bbox.size;

        let (status_font, status_height, status_padding) = if width <= 128 {
            (profont::PROFONT_9_POINT, 12, 5)
        } else {
            (profont::PROFONT_14_POINT, 20, 2)
        };

        let mut x_offs = bbox.top_left.x;
        let mut x_right_offs = bbox.bottom_right().unwrap().x;
        let y_offs = bbox.top_left.y;

        let status_wifi_size = Size::new(status_height * 3 / 4, status_height);
        let status_wifi = shapes::Wifi {
            padding: 1,
            outline: 1,
            strength: None, //Some(60),
            ..Default::default()
        };

        status_wifi.draw(&mut target.cropped(&Rectangle::new(
            Point::new(x_offs, y_offs),
            status_wifi_size,
        )))?;

        x_offs += (status_wifi_size.width + status_padding) as i32;

        let status_mqtt = shapes::Textbox {
            text: "MQTT",
            font: status_font,
            padding: 1,
            outline: 0,
            strikethrough: false,
            ..Default::default()
        };

        // status_mqtt.draw(&mut target.cropped(&Rectangle::new(
        //     Point::new(x_offs, y_offs),
        //     status_mqtt.preferred_size(),
        // )))?;

        //x_offs += (status_mqtt.preferred_size().width + status_padding) as i32;

        let status_battery_size = Size::new(status_height * 2, status_height);
        let status_battery = shapes::Battery {
            charged_percentage: battery_state.and_then(|battery_state| battery_state.percentage()),
            text: BatteryChargedText::No,
            cathode: Size::new(status_height / 2, status_height / 4),
            padding: 1,
            outline: 1,
            distinct_outline: false,
            ..Default::default()
        };

        x_right_offs -= status_battery_size.width as i32;

        if battery_state.is_some() {
            status_battery.draw(
                &mut target
                    .cropped(&Rectangle::new(
                        Point::new(x_right_offs, y_offs),
                        status_battery_size,
                    ))
                    .rotated(RotateAngle::Degrees270),
            )?;
        }

        x_right_offs -= status_padding as i32;

        let status_power = shapes::Textbox {
            text: if battery_state
                .and_then(|battery_state| battery_state.powered)
                .unwrap_or(false)
            {
                "PWR"
            } else {
                "   "
            },
            color: super::super::super::screen::DISPLAY_COLOR_GREEN,
            font: status_font,
            padding: 1,
            outline: 0,
            strikethrough: false,
            ..Default::default()
        };

        x_right_offs -= status_power.preferred_size().width as i32;

        if battery_state.is_some() {
            status_power.draw(&mut target.cropped(&Rectangle::new(
                Point::new(x_right_offs, y_offs),
                status_power.preferred_size(),
            )))?;
        }

        Ok(status_height)
    }

    fn draw_content<D>(
        target: &mut D,
        meter_state: Option<&MeterState>,
        current_time: Option<&RtcExternalState>,
    ) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = Color>,
    {
        let bbox = target.bounding_box();

        let Size { width, .. } = bbox.size;

        let main_font = if width <= 128 {
            profont::PROFONT_18_POINT
        } else {
            profont::PROFONT_24_POINT
        };

        let mut y_offs = bbox.top_left.y;

        let meter_shape = shapes::MeterClassic::<8> {
            edges_count: meter_state.map(|m| m.edges_count),
            font: main_font,
            ..Default::default()
        };

        if meter_state.is_some() {
            meter_shape.draw(&mut target.cropped(&Rectangle::new(
                Point::new(
                    ((width - meter_shape.preferred_size().width) / 2) as i32,
                    y_offs,
                ),
                meter_shape.preferred_size(),
            )))?;
        }

        y_offs += (meter_shape.preferred_size().height + 5) as i32;

        let mut text_buf = heapless::String::<32>::new();
        write!(&mut text_buf, "{}", "").unwrap();

        if let Some(value) = current_time {
            match value {
                RtcExternalState::UpdateScreen(time_buffer) => {
                    let text = format!(
                        "{}-{:0>2}-{:0>2} {:0>2}:{:0>2}:{:0>2}",
                        time_buffer.year,
                        time_buffer.month,
                        time_buffer.date,
                        time_buffer.hours,
                        time_buffer.minutes,
                        time_buffer.seconds,
                    );

                    write!(&mut text_buf, "{}", text).unwrap();
                }
                _ => {
                    write!(&mut text_buf, "{}", "Updating...").unwrap();
                }
            }
        }

        let text_row = shapes::Textbox {
            text: &text_buf,
            color: super::super::super::screen::DISPLAY_COLOR_WHITE,
            font: profont::PROFONT_14_POINT,
            padding: 1,
            outline: 0,
            strikethrough: false,
            ..Default::default()
        };

        let text_row_size = text_row.preferred_size();
        text_row.draw(&mut target.cropped(&Rectangle::new(
            Point::new(((width - text_row_size.width) / 2) as i32, y_offs),
            text_row_size,
        )))?;

        Ok(())
    }

    fn draw_bottom_status_line<D>(
        target: &mut D,
        remaining_time: Option<&RemainingTime>,
        ip_address: Option<&Option<WifiConnection>>,
    ) -> Result<u32, D::Error>
    where
        D: DrawTarget<Color = super::super::super::screen::DisplayColor>,
    {
        let bbox = target.bounding_box();

        let Size { width, .. } = bbox.size;

        let (status_font, status_height, status_padding) = if width <= 128 {
            (profont::PROFONT_9_POINT, 12, 5)
        } else {
            (profont::PROFONT_14_POINT, 20, 2)
        };

        if let Some(remaining_time) = remaining_time {
            let mut status_rt = shapes::Textbox {
                text: "            ",
                color: super::super::super::screen::DISPLAY_COLOR_YELLOW,
                font: status_font,
                padding: 1,
                outline: 0,
                strikethrough: false,
                ..Default::default()
            };

            let mut ip = String::default();
            if let Some(inner) = ip_address {
                if let Some(conn) = inner {
                    ip = conn.ip();
                }
            };

            let mut text_buf = heapless::String::<14>::new();
            write!(&mut text_buf, "ip: {}", ip).unwrap();

            status_rt.text = &text_buf;

            let mut text_buf = heapless::String::<12>::new();
            status_rt.text = match remaining_time {
                RemainingTime::Indefinite => status_rt.text,
                RemainingTime::Duration(duration) => {
                    write!(&mut text_buf, "Sleep in {}s", min(duration.as_secs(), 99)).unwrap();

                    &text_buf
                }
            };

            let status_rt_size = status_rt.preferred_size();

            status_rt.draw(&mut target.cropped(&Rectangle::new(
                bbox.top_left + Size::new(0, bbox.size.height - status_rt_size.height),
                status_rt_size,
            )))?;
        }

        Ok(status_height)
    }
}
