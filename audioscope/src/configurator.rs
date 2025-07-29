use crate::{AudioScopeSettings, Trigger};
use core::iter;
use core::u32;
use defmt_rtt as _;
use embedded_graphics::mono_font::iso_8859_13::FONT_5X7;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::{
    mono_font::MonoTextStyleBuilder,
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use panic_probe as _;
use strum::IntoEnumIterator;
use strum_macros::EnumIter;

pub struct EncoderScopeConfigurator {
    current_setting: EncoderScopeConfiguratorSetting,
    setting_iterator: iter::Cycle<EncoderScopeConfiguratorSettingIter>,
    settings: AudioScopeSettings,
    initial_draw_setting_countdown: u32,
    draw_setting_countdown: u32,
    encoder_offset: i32,
}

#[derive(Debug, EnumIter)]
pub enum EncoderScopeConfiguratorSetting {
    TriggerLevel,
    ZoomY,
    ZoomX,
}

impl EncoderScopeConfiguratorSetting {
    fn str(&self) -> &'static str {
        match self {
            EncoderScopeConfiguratorSetting::ZoomY => "zoom y",
            EncoderScopeConfiguratorSetting::ZoomX => "zoom x",
            EncoderScopeConfiguratorSetting::TriggerLevel => "trigger level",
        }
    }
}

impl EncoderScopeConfigurator {
    pub fn new(draw_setting_countdown: u32, settings_now: AudioScopeSettings) -> Self {
        Self {
            current_setting: EncoderScopeConfiguratorSetting::ZoomY,
            setting_iterator: EncoderScopeConfiguratorSetting::iter().cycle(),
            settings: settings_now,
            initial_draw_setting_countdown: draw_setting_countdown,
            draw_setting_countdown,
            encoder_offset: 0,
        }
    }
    pub fn next_setting(&mut self, new_settings: AudioScopeSettings, encoder_offset: i32) {
        self.settings = new_settings;
        self.draw_setting_countdown = self.initial_draw_setting_countdown;
        self.encoder_offset = encoder_offset;
        self.current_setting = self.setting_iterator.next().unwrap()
    }

    // TODO: this updating stuff is very buggy
    pub fn update_current_setting(&mut self, raw_encoder_value: i32) -> AudioScopeSettings {
        let offset_encoder_value = raw_encoder_value - self.encoder_offset;
        let mut new_settings = self.settings;
        match self.current_setting {
            EncoderScopeConfiguratorSetting::ZoomY => {
                new_settings.zoom_y = self.settings.zoom_y.saturating_add(offset_encoder_value);
            }
            EncoderScopeConfiguratorSetting::ZoomX => {
                new_settings.zoom_x = self
                    .settings
                    .zoom_x
                    .saturating_add(offset_encoder_value as usize);
            }
            EncoderScopeConfiguratorSetting::TriggerLevel => {
                new_settings.trigger = Trigger::RisingEdge {
                    threshold: offset_encoder_value << 27,
                };
            }
        }

        new_settings
    }

    const TEXT_STYLE: MonoTextStyle<'_, BinaryColor> = MonoTextStyleBuilder::new()
        .font(&FONT_5X7)
        .text_color(BinaryColor::On)
        .build();

    pub fn draw<D: DrawTarget>(&mut self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        if self.draw_setting_countdown == 0 {
            return Ok(());
        }

        self.draw_setting_countdown -= 1;

        Text::with_baseline(
            self.current_setting.str(),
            Point::zero(),
            Self::TEXT_STYLE,
            Baseline::Top,
        )
        .draw(target)?;

        Ok(())
    }
}
