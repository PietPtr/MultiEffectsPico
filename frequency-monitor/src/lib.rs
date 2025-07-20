#![no_std]

use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};

const FFT_SIZE: usize = 128; // Same as OLED display width
const SAMPLE_BIT_WIDTH: u32 = 16;
const MAXIMUM_SAMPLE_SIZE: i32 = 2i32.pow(SAMPLE_BIT_WIDTH);

/// Given a time domain signal, computes the FFT (with a library), and displays it on an embedded graphics target
pub struct FrequencyMonitor {
    frequency_domain: [f32; FFT_SIZE],
    spectrum: [u32; FFT_SIZE],
    pub inverse_scale: u32,
}

impl FrequencyMonitor {
    pub fn new(scale: u32) -> Self {
        Self {
            frequency_domain: [0.; FFT_SIZE],
            spectrum: [0; FFT_SIZE],
            inverse_scale: scale,
        }
    }

    const BASE_SCALE: u32 = 256;

    pub fn recompute(&mut self, time_domain_signal: [u32; FFT_SIZE]) {
        // TODO: cheat at the u32 -> f32 conversion and do it with bit magic?
        // https://www.h-schmidt.net/FloatConverter/IEEE754.html
        // TODO: set up real hw benchmark for this function (turn lib into binary crate)

        self.frequency_domain
            .copy_from_slice(&time_domain_signal.map(|v| v as f32 / MAXIMUM_SAMPLE_SIZE as f32));

        let spectrum = microfft::real::rfft_128(&mut self.frequency_domain);
        spectrum[0].im = 0.0;

        // self.frequency_domain = spectrum.iter().map(|c| c.norm_sqr());
        for i in 0..FFT_SIZE {
            self.spectrum[i] =
                (spectrum[i].norm_sqr() as u32 * Self::BASE_SCALE) / self.inverse_scale
        }
    }

    const LINE_STYLE: PrimitiveStyle<BinaryColor> = PrimitiveStyle::with_stroke(BinaryColor::On, 1);

    pub fn draw<D: DrawTarget>(&mut self, target: &mut D) -> Result<(), D::Error>
    where
        D: DrawTarget<Color = BinaryColor>,
    {
        for (x, points) in self.spectrum.windows(2).enumerate() {
            let &[point1, point2] = points else {
                unreachable!()
            };
            let x = x as i32;
            Line::new(Point::new(x, point1 as i32), Point::new(x, point2 as i32))
                .into_styled(Self::LINE_STYLE)
                .draw(target)?;
        }

        Ok(())
    }
}
