#![no_std]

pub mod configurator;

use defmt::info;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle, Rectangle, StyledDrawable},
};

/// Settings that are expected to be adjusted by a user
#[derive(Debug, defmt::Format, Copy, Clone)]
pub struct AudioScopeSettings {
    // TODO: set which edge to trigger on? or keep rising by default
    pub trigger: Trigger,
    // difference from the center of the captured signal that the view is horizontally centered on
    pub view_x: i32,
    // difference from zero that the view is vertically centered on
    pub view_y: i32,
    // scale to multiply horizontal point position with
    pub zoom_x: usize,
    // bitshift the sample value by this many places
    pub zoom_y: i32,
    pub trigger_signal_index: usize,
}

impl Default for AudioScopeSettings {
    fn default() -> Self {
        Self {
            // TODO: make these make sense
            trigger: Trigger::NoTrigger,
            view_x: AudioScope::SIGNAL_LENGTH as i32 / 2
                - AudioScope::OLED_DISPLAY_WIDTH as i32 / 2,
            view_y: 0,
            zoom_x: 1,
            zoom_y: 26,
            trigger_signal_index: AudioScope::SIGNAL_LENGTH / 2,
        }
    }
}

#[derive(Debug, defmt::Format, Copy, Clone)]
pub enum Trigger {
    NoTrigger,
    RisingEdge { threshold: i32 },
}

pub struct AudioScope {
    signal: [i32; Self::SIGNAL_LENGTH],
    pub settings: AudioScopeSettings,
}

impl AudioScope {
    pub const MAX_X_ZOOM: usize = 8;
    pub const SIGNAL_LENGTH: usize = 128 * Self::MAX_X_ZOOM;
    const OLED_DISPLAY_WIDTH: i32 = 128;
    const OLED_DISPLAY_HEIGHT: i32 = 64;

    pub fn new() -> Self {
        Self {
            signal: [0; Self::SIGNAL_LENGTH],
            settings: AudioScopeSettings::default(),
        }
    }

    pub fn update_signal(&mut self, new_signal: &[i32; Self::SIGNAL_LENGTH]) {
        match self.settings.trigger {
            Trigger::NoTrigger => {
                self.signal.copy_from_slice(new_signal);
            }
            Trigger::RisingEdge { threshold } => {
                // Given the new signal, search for a trigger event starting at the trigger signal index
                // when found, the found index will be at index trigger_signal_index, and the rest moves accordingly
                // any index that does not have a mapping this way is for now set to zero TODO: find a nicer way to deal with that
                let mut triggered_index = None;
                for (index, points) in new_signal
                    .windows(2)
                    .skip(self.settings.trigger_signal_index)
                    .enumerate()
                {
                    let [point1, point2] = points else {
                        unreachable!()
                    };

                    if *point1 <= threshold && *point2 > threshold {
                        triggered_index = Some(index);
                    }
                }

                let Some(index) = triggered_index else {
                    return;
                };

                let diff_trigger_to_view = index as i32 - self.settings.trigger_signal_index as i32;

                self.signal.copy_from_slice(new_signal);
                for (index, point) in self.signal.iter_mut().enumerate() {
                    *point = new_signal
                        .get((index as i32 + diff_trigger_to_view).clamp(0, i32::MAX) as usize)
                        .copied()
                        .unwrap_or_default()
                }
            }
        }
    }

    const STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    pub fn draw<D: DrawTarget>(&self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        let zoom_x = self.settings.zoom_x.clamp(1, Self::MAX_X_ZOOM);

        let x_start = self.settings.view_x - (Self::OLED_DISPLAY_WIDTH / 2) * zoom_x as i32;
        let x_end = self.settings.view_x + (Self::OLED_DISPLAY_WIDTH / 2 - 1) * zoom_x as i32;
        info!("{}..{}", x_start, x_end);
        let mut last = None;
        for x in (x_start..x_end).step_by(zoom_x) {
            let Some(&measurement_point) = self.signal.get(x as usize) else {
                continue; // Skip out of bound points
            };

            let y = measurement_point >> self.settings.zoom_y;

            let Some((x_last, y_last)) = last else {
                last = Some((x, y));
                continue;
            };

            let line_start_x = x_last - x_start;
            let line_end_x = x - x_start;

            Line::new(
                Point::new(
                    line_start_x / zoom_x as i32,
                    y_last + Self::OLED_DISPLAY_HEIGHT / 2,
                ),
                Point::new(
                    line_end_x / zoom_x as i32,
                    y + Self::OLED_DISPLAY_HEIGHT / 2,
                ),
            )
            .draw_styled(&Self::STYLE, target)?;
            last = Some((x, y));
        }

        // draw trigger line as dotted horizontal line
        match self.settings.trigger {
            Trigger::NoTrigger => (),
            Trigger::RisingEdge { threshold } => {
                let trigger_y = (threshold >> self.settings.zoom_y) + Self::OLED_DISPLAY_HEIGHT / 2;
                for x in (0..(Self::OLED_DISPLAY_WIDTH)).step_by(3) {
                    Pixel(Point::new(x, trigger_y), BinaryColor::On).draw(target)?;
                }
            }
        }

        Ok(())
    }
}
