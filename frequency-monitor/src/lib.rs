#![no_std]

use core::f32::consts::PI;

use defmt::info;
use embedded_graphics::{
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{Line, PrimitiveStyle},
};
use micromath::F32Ext;

/// Given a time domain signal, computes the FFT (with a library), and displays it on an embedded graphics target
pub struct FrequencyMonitor {
    frequency_domain: [f32; Self::FFT_SIZE],
    spectrum: [u32; Self::FFT_SIZE / 2],
    /// The smaller this number, the higher the displayed peaks
    pub inverse_scale: u32,
}

impl FrequencyMonitor {
    // if this constant is changed, the call to microfft::real::rfft_* must be changed as well
    pub const FFT_SIZE: usize = 256;
    pub const SAMPLE_BIT_WIDTH: u32 = 16;
    const MAXIMUM_SAMPLE_SIZE: i32 = 2i32.pow(Self::SAMPLE_BIT_WIDTH);
    const OLED_DISPLAY_WIDTH: i32 = 128;
    const OLED_DISPLAY_HEIGHT: i32 = 64;

    pub fn new(inverse_scale: u32) -> Self {
        Self {
            frequency_domain: [0.; Self::FFT_SIZE],
            spectrum: [0; Self::FFT_SIZE / 2],
            inverse_scale,
        }
    }

    const BASE_SCALE: u32 = 64;

    pub fn stats(sample_rate: usize) -> FftStats {
        FftStats {
            sample_rate,
            fft_size: Self::FFT_SIZE,
        }
    }

    pub fn hamming_window(signal: &mut [f32]) {
        let n = signal.len();
        for (i, val) in signal.iter_mut().enumerate() {
            let multiplier = 0.54 - 0.46 * (2.0 * PI * i as f32 / (n - 1) as f32).cos();
            *val *= multiplier;
        }
    }

    pub fn recompute(&mut self, time_domain_signal: [u32; Self::FFT_SIZE]) {
        // TODO: cheat at the u32 -> f32 conversion and do it with bit magic?
        // https://www.h-schmidt.net/FloatConverter/IEEE754.html

        self.frequency_domain.copy_from_slice(
            &time_domain_signal.map(|v| v as f32 / Self::MAXIMUM_SAMPLE_SIZE as f32),
        );

        FrequencyMonitor::hamming_window(&mut self.frequency_domain);

        let spectrum = microfft::real::rfft_256(&mut self.frequency_domain);
        spectrum[0].im = 0.0;

        // self.frequency_domain = spectrum.iter().map(|c| c.norm_sqr());
        for (i, spectrum_datapoint) in spectrum.iter().enumerate() {
            self.spectrum[i] =
                ((spectrum_datapoint.norm_sqr() as u64) / self.inverse_scale as u64) as u32
            // self.spectrum[i] = spectrum_datapoint.re as u32 / self.inverse_scale;
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
            let point1 = point1.clamp(0, Self::OLED_DISPLAY_HEIGHT as u32);
            let point2 = point2.clamp(0, Self::OLED_DISPLAY_HEIGHT as u32);

            let x = x as i32;
            Line::new(
                Point::new(x, Self::OLED_DISPLAY_HEIGHT - 1),
                Point::new(x, Self::OLED_DISPLAY_HEIGHT - 1 - point1 as i32),
            )
            .into_styled(Self::LINE_STYLE)
            .draw(target)?;

            if x > Self::OLED_DISPLAY_WIDTH {
                break;
            }
        }

        Ok(())
    }
}

/// Expensive to compute debugging functions for configuring FFT's
pub struct FftStats {
    pub sample_rate: usize,
    pub fft_size: usize,
}

impl FftStats {
    pub fn bin_width(&self) -> f32 {
        self.sample_rate as f32 / self.fft_size as f32
    }

    pub fn frequency_at_bin(&self, bin_index: usize) -> Result<f32, &str> {
        if bin_index > self.fft_size {
            return Err("bin_index larger than fft_size");
        }

        Ok(bin_index as f32 * self.bin_width())
    }

    pub fn print(&self) {
        info!(
            "\nsample rate: {}Hz\nFFT size:{}\nbin width: {}Hz",
            self.sample_rate,
            self.fft_size,
            self.bin_width()
        );
    }
}
