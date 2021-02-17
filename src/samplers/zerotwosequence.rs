use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::{Float, round_up_pow2_64, is_power_of2, Options};
use crate::core::rng::RNG;
use log::warn;
use crate::*;
use crate::pixel_sampler_new;
use crate::core::sampler::{Sampler, Samplers};
use crate::core::camera::CameraSample;
use crate::core::lowdiscrepancy::{vander_corput, sobol_2d};
use crate::core::paramset::ParamSet;

#[derive(Default, Clone)]
pub struct ZeroTwoSequenceSampler {
    // Sampler data
    current_pixel               : Point2i,
    current_pixel_sample_index  : u64,
    samples_1d_array_sizes      : Vec<usize>,
    samples_2d_array_sizes      : Vec<usize>,
    sample_array_1d             : Vec<Vec<Float>>,
    sample_array_2d             : Vec<Vec<Point2f>>,
    array_1d_offset             : usize,
    array_2d_offset             : usize,
    samples_per_pixel           : u64,
    // PixelSampler data
    samples_1d                  : Vec<Vec<Float>>,
    samples_2d                  : Vec<Vec<Point2f>>,
    current_1d_dimension        : usize,
    current_2d_dimension        : usize,
    rng                         : RNG
}

impl ZeroTwoSequenceSampler {
    pub fn new(samples_per_pixel: u64, nsampled_dimensions: usize) -> Self {
        let mut zts = Self::default();
        let spp = round_up_pow2_64(samples_per_pixel as i64);
        pixel_sampler_new!(spp as u64, nsampled_dimensions, zts);

        if !is_power_of2(samples_per_pixel) {
            warn!(
                "Pixel samples being rounded up to power of 2 \
                (from {} to {}).", samples_per_pixel, spp);
        }

        zts
    }
}

impl Sampler for ZeroTwoSequenceSampler {
    get_sampler_data!();

    fn start_pixel(&mut self, p: &Point2i) {
        // TODO: ProfilePhase
        // Generate 1D and 2D pixel sample components using (0, 2)-sequence
        for samples in &mut self.samples_1d {
            vander_corput(1, self.samples_per_pixel as usize, samples, &mut self.rng);
        }
        for samples in &mut self.samples_2d {
            sobol_2d(1, self.samples_per_pixel as usize, samples, &mut self.rng);
        }

        // Generate 1D and 2D array samples using (0, 2)-sequence
        for i in 0..self.samples_1d_array_sizes.len() {
            let samples = self.sample_array_1d[i].as_mut_slice();
            let nspps = self.samples_1d_array_sizes[i];
            vander_corput(nspps, self.samples_per_pixel as usize, samples, &mut self.rng);
        }
        for i in 0..self.samples_2d_array_sizes.len() {
            let samples = self.sample_array_2d[i].as_mut_slice();
            let nspps = self.samples_2d_array_sizes[i];
            sobol_2d(nspps, self.samples_per_pixel as usize, samples, &mut self.rng);
        }

        start_pixel_default!(self, *p);
    }

    pixel_get_1d!();

    pixel_get_2d!();

    get_camera_sample_default!();

    request_1d_array_default!();

    request_2d_array_default!();

    fn round_count(&self, n: usize) -> usize {
        round_up_pow2_64(n as i64) as usize
    }

    get_1d_array_default!();

    get_2d_array_default!();

    pixel_start_next_sample!();

    pixel_set_sample_number!();

    current_sample_number_default!();

    fn clone(&self, seed: isize) -> Samplers {
        let mut lds = Clone::clone(self);
        lds.rng.set_sequence(seed as u64);

        lds.into()
    }
}

pub fn create_zerotwo_sequence_sampler(params: &ParamSet, opts: &Options) -> Option<Box<Samplers>> {
    let mut nsamp = params.find_one_int("pixelsamples", 16);
    let sd = params.find_one_int("dimensions", 4);

    if opts.quick_render { nsamp = 1; }

    Some(Box::new(ZeroTwoSequenceSampler::new(nsamp as u64, sd as usize).into()))
}