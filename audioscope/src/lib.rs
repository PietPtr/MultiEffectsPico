#![no_std]

use core::f32::consts::PI;

use defmt::info;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use micromath::F32Ext;

/// Settings that are expected to be adjusted by a user
pub struct AudioScopeSettings {
    // TODO: set which edge to trigger on? or keep rising by default
    trigger_value: i32,
    view_x: i32,
    zoom_x: i32,
    trigger_signal_index: usize,
}

impl Default for AudioScopeSettings {
    fn default() -> Self {
        Self {
            // TODO: make these make sense
            trigger_value: 0x1000,
            view_x: AudioScope::SIGNAL_LENGTH as i32 / 2
                - AudioScope::OLED_DISPLAY_WIDTH as i32 / 2,
            trigger_signal_index: AudioScope::SIGNAL_LENGTH / 2,
        }
    }
}

pub struct AudioScope {
    signal: [i32; Self::SIGNAL_LENGTH],
    pub settings: AudioScopeSettings,
}

impl AudioScope {
    pub const SIGNAL_LENGTH: usize = 128 * 8;
    const OLED_DISPLAY_WIDTH: i32 = 128;
    const OLED_DISPLAY_HEIGHT: i32 = 64;

    pub fn new() -> Self {
        Self {
            signal: [0; Self::SIGNAL_LENGTH],
            settings: AudioScopeSettings::default(),
        }
    }

    const STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    pub fn update_signal(&mut self, new_signal: &[i32; Self::SIGNAL_LENGTH]) {
        // Given the new signal, search for a trigger event starting at the trigger signal index
        // when found, the found index will be at index trigger_signal_index, and the rest scales accordingly
        // any index that does not have a mapping this way is for now set to zero TODO: find a nicer way to deal with that
        self.signal.copy_from_slice(new_signal);

        let mut triggered_index = None;
        for (index, points) in new_signal
            .windows(2)
            .skip(self.settings.trigger_signal_index)
            .enumerate()
        {
            let [point1, point2] = points else {
                unreachable!()
            };

            if *point1 <= self.settings.trigger_value && *point2 > self.settings.trigger_value {
                triggered_index = Some(index);
            }
        }

        let Some(index) = triggered_index else {
            info!("No triggered index found, not refreshing scope");
            return;
        };

        let diff_trigger_to_view = index as i32 - self.settings.trigger_signal_index as i32;

        // TODO: probably has bugs
        for (index, point) in self.signal.iter_mut().enumerate() {
            *point = new_signal
                .get((index as i32 + diff_trigger_to_view).clamp(0, i32::MAX) as usize)
                .copied()
                .unwrap_or_default()
        }
    }

    pub fn draw<D: DrawTarget>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        Ok(())
    }
}
