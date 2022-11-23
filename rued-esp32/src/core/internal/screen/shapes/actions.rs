use core::str;

use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::mono_font::*;
use embedded_graphics::prelude::{OriginDimensions, Point, Size};
use embedded_graphics::primitives::Rectangle;
use enumset::{EnumSet, EnumSetType};
// use valve::{ValveCommand, ValveState};

// use crate::dto::water_meter::WaterMeterCommand;
// use crate::{valve, wm};

use super::util::{clear_cropped, fill, text};
use super::Color;

#[derive(Debug, EnumSetType)]
pub enum Action {
    Dismiss,
    OpenValve,
    CloseValve,
    Arm,
    Disarm,
    CheckForUpdate,
    Update,
    Pair,
    Provision,
    Reprovision,
}

impl Action {
    pub fn text(&self) -> &'static str {
        match self {
            Self::Dismiss => "Dismiss",
            Self::OpenValve => "Open Valve",
            Self::CloseValve => "Close Valve",
            Self::Arm => "Arm",
            Self::Disarm => "Disarm",
            Self::CheckForUpdate => "Check for Update",
            Self::Update => "Update",
            Self::Pair => "Pair",
            Self::Provision => "Provision",
            Self::Reprovision => "Reprovision",
        }
    }

    pub fn first(actions: &EnumSet<Action>) -> Option<Self> {
        actions.into_iter().next()
    }

    pub fn prev(&self, actions: &EnumSet<Action>) -> Option<Self> {
        self.switch(actions, false)
    }

    pub fn next(&self, actions: &EnumSet<Action>) -> Option<Self> {
        self.switch(actions, true)
    }

    fn switch(&self, actions: &EnumSet<Action>, next: bool) -> Option<Self> {
        let mut cindex = actions
            .iter()
            .enumerate()
            .find_map(|(index, action)| (action == *self).then_some(index as i32))
            .unwrap_or(0);

        if next {
            cindex += 1;
        } else {
            cindex -= 1;
        }

        if cindex >= actions.len() as i32 {
            cindex = 0;
        }

        if cindex < 0 {
            cindex = actions.len() as i32 - 1;
        }

        actions
            .iter()
            .enumerate()
            .find_map(|(index, action)| (index as i32 == cindex).then_some(action))
    }

    pub fn active() -> EnumSet<Self> {
        let mut actions = EnumSet::empty();

        // let valve_state = valve::STATE.get();

        // if !matches!(
        //     valve_state,
        //     Some(ValveState::Open) | Some(ValveState::Opening(_))
        // ) {
        //     actions |= Action::OpenValve;
        // }

        // if !matches!(
        //     valve_state,
        //     Some(ValveState::Closed) | Some(ValveState::Closing(_))
        // ) {
        //     actions |= Action::CloseValve;
        // }

        // let wm_state = wm::STATE.get();

        // if !wm_state.armed {
        //     actions |= Action::Arm;
        // } else {
        //     actions |= Action::Disarm;
        // }

        actions
    }

    pub fn trigger(&self) {
        match self {
            // NOTE: disabled for now.
            // Self::OpenValve => valve::COMMAND.signal(ValveCommand::Open),
            // Self::CloseValve => valve::COMMAND.signal(ValveCommand::Close),
            // Self::Arm => wm::COMMAND.signal(WaterMeterCommand::Arm),
            // Self::Disarm => wm::COMMAND.signal(WaterMeterCommand::Disarm),

            // Disabled in ruwm
            // Self::CheckForUpdate => "Check for Update",
            // Self::Update => "Update",
            // Self::Pair => "Pair",
            // Self::Provision => "Provision",
            // Self::Reprovision => "Reprovision",
            _ => {}
        }
    }
}

pub struct Actions<'a> {
    pub enabled: EnumSet<Action>,
    pub selected: Action,
    pub divider: u32,
    pub padding: u32,
    pub outline: u32,
    pub font: MonoFont<'a>,
}

impl<'a> Actions<'a> {
    pub fn new() -> Self {
        Self {
            enabled: EnumSet::all(),
            selected: Action::Dismiss,
            divider: 1,
            padding: 2,
            outline: 2,
            font: profont::PROFONT_18_POINT,
        }
    }

    pub fn preferred_size(&self) -> Size {
        let width = self
            .enabled
            .iter()
            .map(|action| action.text().len())
            .max()
            .unwrap_or(0) as u32;
        let height = self.font.character_size.height * self.enabled.len() as u32;

        Size::new(width, height)
            + Size::new(self.padding, self.padding) * 2
            + Size::new(self.outline, self.outline) * 2
    }

    pub fn draw<T>(&self, target: &mut T) -> Result<(), T::Error>
    where
        T: DrawTarget<Color = Color>,
    {
        self.draw_shape(&mut clear_cropped(target, self.padding)?)
    }

    fn draw_shape<T>(&self, target: &mut T) -> Result<(), T::Error>
    where
        T: DrawTarget<Color = Color> + OriginDimensions,
    {
        let bbox = target.bounding_box();

        fill(&bbox, Color::LightBlue, target)?;

        for (line, action) in self.enabled.iter().enumerate() {
            if self.selected == action {
                fill(
                    &Rectangle::new(
                        Point::new(
                            bbox.top_left.x + self.outline as i32,
                            bbox.top_left.y
                                + self.outline as i32
                                + self.font.character_size.height as i32 * line as i32,
                        ),
                        Size::new(
                            bbox.size.width - self.outline * 2,
                            self.font.character_size.height,
                        ),
                    ),
                    Color::Black,
                    target,
                )?;
            }

            text(
                &self.font,
                target,
                Point::new(
                    bbox.top_left.x + self.outline as i32,
                    bbox.top_left.y
                        + self.outline as i32
                        + self.font.character_size.height as i32 * line as i32,
                ),
                action.text(),
                Color::White,
                None,
            )?;
        }

        Ok(())
    }
}

impl<'a> Default for Actions<'a> {
    fn default() -> Self {
        Self::new()
    }
}
