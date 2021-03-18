use crate::core::rng::RNG;
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::Float;
use crate::*;
use crate::core::sampler::{Sampler, Samplers};
use crate::core::camera::CameraSample;
use crate::core::paramset::ParamSet;

#[derive(Default, Clone)]
pub struct RandomSampler {
    rng                         : RNG,
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
}

impl RandomSampler {
    pub fn new(ns: u64, seed: u64) -> Self {
        let mut rs = RandomSampler {
            rng: RNG::new(seed),
            ..Default::default()
        };

        sampler_new!(ns, rs);

        rs
    }
}

impl Sampler for RandomSampler {
    get_sampler_data!();

    fn start_pixel(&mut self, p: &Point2i) {
        for i in 0..self.sample_array_1d.len() {
            for j in 0..self.sample_array_1d[i].len() {
                self.sample_array_1d[i][j] = self.rng.uniform_float();
            }
        }

        for i in 0..self.sample_array_2d.len() {
            for j in 0..self.sample_array_2d[i].len() {
                let x = self.rng.uniform_float();
                let y = self.rng.uniform_float();
                self.sample_array_2d[i][j] = Point2f::new(x, y)
            }
        }

        start_pixel_default!(self, *p);
    }

    fn get_1d(&mut self) -> f32 {
        // TODO: ProfilePhase
        assert!(self.current_pixel_sample_index < self.samples_per_pixel);

        self.rng.uniform_float()
    }

    fn get_2d(&mut self) -> Point2f {
        // TODO: ProfilePhase
        assert!(self.current_pixel_sample_index < self.samples_per_pixel);
        let x = self.rng.uniform_float();
        let y = self.rng.uniform_float();

        Point2f::new(x, y)
    }

    get_camera_sample_default!();

    request_1d_array_default!();

    request_2d_array_default!();

    get_1d_array_default!();

    get_2d_array_default!();

    fn start_next_sample(&mut self) -> bool {
        start_next_sample_default!(self)
    }

    fn set_sample_number(&mut self, sample_num: u64) -> bool {
        set_sample_number_default!(self, sample_num)
    }

    current_sample_number_default!();

    fn clone(&self, seed: isize) -> Samplers {
        let mut rs = Clone::clone(self);
        rs.rng.set_sequence(seed as u64);

        rs.into()
    }
}

pub fn create_random_sampler(params: &ParamSet) -> Option<Box<Samplers>> {
    let ns = params.find_one_int("pixelsamples", 4) as u64;

    Some(Box::new(RandomSampler::new(ns, 0).into()))
}