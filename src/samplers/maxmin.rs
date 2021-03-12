use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::{Float, log2_int64, is_power_of2, round_up_pow2_64, Options};
use crate::core::rng::RNG;
use log::warn;
use crate::core::lowdiscrepancy::{CMAX_MIN_DIST, sample_generator_matrix, vander_corput, sobol_2d};
use crate::*;
use crate::core::sampler::{Sampler, Samplers};
use crate::core::camera::CameraSample;
use crate::core::sampling::shuffle;
use crate::core::paramset::ParamSet;

#[derive(Clone)]
pub struct MaxMinDistSampler {
    cpixel                      : &'static [u32],
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

impl MaxMinDistSampler {
    pub fn new(samples_per_pixel: u64, nsampled_dimensions: usize) -> Self {
        let mut spp = samples_per_pixel;
        let cindex = log2_int64(spp as i64) as u64;

        if cindex >= 17 {
            panic!(
                "No more than {} samples per pixel are supported with \
                MaxMinDistSampler.", cindex);
        }

        if !is_power_of2(spp) {
            spp = round_up_pow2_64(spp as i64) as u64;
            warn!(
                "Non power-of-two sample count rounded up to {} \
                for MaxMinDistSampler", spp);
        }

        let cindex = log2_int64(spp as i64);
        assert!(cindex >= 0 && cindex < 17);

        let mut maxmin = MaxMinDistSampler {
            cpixel: &CMAX_MIN_DIST[cindex as usize],
            current_pixel: Default::default(),
            current_pixel_sample_index: 0,
            samples_1d_array_sizes: vec![],
            samples_2d_array_sizes: vec![],
            sample_array_1d: vec![],
            sample_array_2d: vec![],
            array_1d_offset: 0,
            array_2d_offset: 0,
            samples_per_pixel: spp,
            samples_1d: vec![],
            samples_2d: vec![],
            current_1d_dimension: 0,
            current_2d_dimension: 0,
            rng: Default::default()
        };

        pixel_sampler_new!(spp, nsampled_dimensions, maxmin);

        maxmin
    }
}

impl Sampler for MaxMinDistSampler {
    get_sampler_data!();

    fn start_pixel(&mut self, p: &Point2i) {
        // TODO: ProfilePhase
        let inv_spp = 1.0 / self.samples_per_pixel as Float;

        for i in 0..self.samples_per_pixel as usize {
            let pix = Point2f::new(
                i as Float * inv_spp,
                sample_generator_matrix(&self.cpixel, i as u32, 0));


            self.samples_2d[0][i] = pix
        }

        let samples = self.samples_2d[0].as_mut_slice();
        shuffle(samples, self.samples_per_pixel as usize, 1, &mut self.rng);
        // Generate remaining samples for MaxMinDistSampler
        for samples in &mut self.samples_1d {
            vander_corput(1, self.samples_per_pixel as usize, samples, &mut self.rng);
        }



        for samples in self.samples_2d.iter_mut().skip(1) {
            sobol_2d(1, self.samples_per_pixel as usize, samples, &mut self.rng);
        }


        for i in 0..self.samples_1d_array_sizes.len() {
            let count = self.samples_1d_array_sizes[i];
            let samples = self.sample_array_1d[i].as_mut_slice();
            vander_corput(count, self.samples_per_pixel as usize, samples, &mut self.rng);
        }
        for i in 0..self.samples_2d_array_sizes.len() {
            let count = self.samples_2d_array_sizes[i];
            let samples = self.sample_array_2d[i].as_mut_slice();
            sobol_2d(count, self.samples_per_pixel as usize, samples, &mut self.rng);
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
        let mut mmds = Clone::clone(self);
        mmds.rng.set_sequence(seed as u64);

        mmds.into()
    }
}

pub fn create_maxmin_dist_sampler(params: &ParamSet, opts: &Options) -> Option<Box<Samplers>> {
    let mut nsamp = params.find_one_int("pixelsamplers", 16);
    let sd = params.find_one_int("dimensions", 4);
    if opts.quick_render { nsamp = 1; }

    Some(Box::new(MaxMinDistSampler::new(nsamp as u64, sd as usize).into()))
}