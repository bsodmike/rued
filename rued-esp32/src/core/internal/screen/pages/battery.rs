use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::BinaryColor;

use crate::core::internal::battery::BatteryState;
use crate::core::internal::screen::shapes::{self, Color};

use super::with_title;

pub struct Battery;

impl Battery {
    pub fn draw<T>(
        target: &mut T,
        page_changed: bool,
        state: Option<&BatteryState>,
    ) -> Result<(), T::Error>
    where
        T: DrawTarget<Color = crate::core::internal::screen::DisplayColor>,
    {
        let mut target = with_title(target, page_changed, "Battery")?;

        shapes::Battery {
            charged_percentage: Some(50),
            ..Default::default()
        }
        .draw(&mut target)?;

        // if let Some(state) = state {
        //     shapes::Battery {
        //         charged_percentage: state.percentage(),
        //         ..Default::default()
        //     }
        //     .draw(&mut target)?;
        // }

        Ok(())
    }
}
