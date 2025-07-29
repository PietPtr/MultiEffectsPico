use audioscope::{configurator::EncoderScopeConfigurator, AudioScopeSettings, Trigger};

#[test]
fn test_single_encoder_configurator() {
    let mut settings = AudioScopeSettings {
        trigger: Trigger::RisingEdge { threshold: 100 },
        view_x: 0,
        view_y: 0,
        zoom_x: 1,
        zoom_y: 1,
        trigger_signal_index: 500,
    };
    let mut configurator = EncoderScopeConfigurator::new(10, settings);

    let mut encoder = 0i32;

    // configurator.next_setting(settings, encoder);
    settings = dbg!(configurator.update_current_setting(encoder));
    encoder += 1;
    settings = dbg!(configurator.update_current_setting(encoder));
    encoder += 1;
    settings = dbg!(configurator.update_current_setting(encoder));
    configurator.next_setting(settings, encoder);
    settings = dbg!(configurator.update_current_setting(encoder));
    encoder += 1;
    settings = dbg!(configurator.update_current_setting(encoder));
}
