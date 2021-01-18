use crate::core::sampler::*;
use crate::*;
use crate::core::sampling::{stratified_sample_1d, stratified_sample_2d, shuffle, latin_hypercube};
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::Float;
use crate::core::rng::RNG;
use crate::core::camera::CameraSample;
use crate::core::paramset::ParamSet;

#[derive(Debug, Default, Clone)]
pub struct StratifiedSampler {
    xpixel_samples              : usize,
    ypixel_samples              : usize,
    jitter_samples              : bool,
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

impl StratifiedSampler {
    fn new(
        xpixel_samples: usize, ypixel_samples: usize,
        jitter_samples: bool, n_sampled_dimensions: usize) -> Self {
        let mut ss = Self {
            xpixel_samples,
            ypixel_samples,
            jitter_samples,
            ..Default::default()
        };

        pixel_sampler_new!((xpixel_samples * ypixel_samples) as u64,  n_sampled_dimensions, ss);

        ss
    }
}

impl Sampler for StratifiedSampler {
    fn start_pixel(&mut self, p: &Point2i) {
        // Generate single statified samples for the pixel
        let count = (self.xpixel_samples * self.ypixel_samples) as usize;

        for i in 0..self.samples_1d.len() {
            let samples = &mut self.samples_1d[i][0..];

            stratified_sample_1d(samples, count, &mut self.rng, self.jitter_samples);
            shuffle(samples, count, 1, &mut self.rng);
        }

        for i in 0..self.samples_2d.len() {
            let samples = &mut self.samples_2d[i][0..];
            stratified_sample_2d(samples, self.xpixel_samples as usize, self.ypixel_samples as usize, &mut self.rng, self.jitter_samples);
            shuffle(samples, count, 1, &mut self.rng);
        }

        // Generate arrays of stratified samples for the pixel
        for i in 0..self.samples_1d_array_sizes.len() {
            for j in 0..self.samples_per_pixel {
                let count = self.samples_1d_array_sizes[i] as usize;
                let samples = &mut self.sample_array_1d[i][(j as usize * count as usize)..];
                stratified_sample_1d(samples, count, &mut self.rng, self.jitter_samples);
                shuffle(samples, count , 1, &mut self.rng)
            }
        }

        for  i in 0..self.samples_2d_array_sizes.len() {
            for  j in 0..self.samples_per_pixel {
                let count = self.samples_2d_array_sizes[i];
                let samples = &mut self.sample_array_2d[i][(j as usize * count as usize)..];
                latin_hypercube(samples, count as usize, 2, &mut self.rng)
            }
        }

        start_pixel_default!(self, *p);
    }

    pixel_get_1d!();

    pixel_get_2d!();

    get_camera_sample_default!();

    request_1d_array_default!();

    request_2d_array_default!();

    get_1d_array_default!();

    get_2d_array_default!();

    pixel_start_next_sample!();

    pixel_set_sample_number!();

    current_sample_number_default!();

    fn clone(&self, seed: isize) -> Samplers {
        let mut ss = Clone::clone(self);
        ss.rng.set_sequence(seed as u64);

        ss.into()
    }
}

pub fn create_stratified_sampler(params: &ParamSet, quick_render: bool) -> Samplers {
    let jitter = params.find_one_bool("jitter", true);
    let mut xsamp = params.find_one_int("xsample", 4);
    let mut ysamp = params.find_one_int("ysamples", 4);
    let sd = params.find_one_int("dimensions", 4);

    if quick_render {
        xsamp = 1;
        ysamp = 1;
    }

    StratifiedSampler::new(xsamp as usize, ysamp as usize, jitter, sd as usize).into()
}