use enum_dispatch::enum_dispatch;
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::Float;
use crate::samplers::stratified::StratifiedSampler;
use crate::samplers::halton::HaltonSampler;
use crate::core::camera::CameraSample;
use crate::samplers::zerotwosequence::ZeroTwoSequenceSampler;
use crate::samplers::maxmin::MaxMinDistSampler;
use crate::samplers::sobol::SobolSampler;
use crate::samplers::random::RandomSampler;

pub const ARRAY_START_DIM: usize = 5;

#[enum_dispatch(Sampler)]
pub enum Samplers {
    SobolSampler,
    RandomSampler,
    StratifiedSampler,
    HaltonSampler,
    MaxMinDistSampler,
    ZeroTwoSequenceSampler
}

#[enum_dispatch]
pub trait Sampler {
    fn samples_per_pixel(&self) -> u64;

    fn start_pixel(&mut self, p: &Point2i);
    fn get_1d(&mut self) -> Float;
    fn get_2d(&mut self) -> Point2f;
    fn get_camera_sample(&mut self, p_raster: &Point2i) -> CameraSample;
    fn request_1d_array(&mut self, n: usize);
    fn request_2d_array(&mut self, n: usize);
    fn round_count(&self, n: usize) -> usize { n }
    fn get_1d_array(&mut self, n: usize) -> Option<&[Float]>;
    fn get_2d_array(&mut self, n: usize) -> Option<&[Point2f]>;
    fn start_next_sample(&mut self) -> bool;
    fn set_sample_number(&mut self, sample_num: u64) -> bool;
    fn current_sample_number(&self) -> u64;

    fn clone(&self, seed: isize) -> Samplers;
}

pub trait GlobalSampler: Sampler {
    fn get_index_for_sample(&self, sample_num: u64) -> u64;
    fn sample_dimension(&self, index: u64, dimension: usize) -> Float;
}

#[macro_export]
macro_rules! sampler_new {
    ($samples_per_pixel:expr, $s:ident) => {{
        $s.samples_per_pixel = $samples_per_pixel
    }}
}

#[macro_export]
macro_rules! get_sampler_data {
    () => {
        fn samples_per_pixel(&self) -> u64 { self.samples_per_pixel }
    }
}

#[macro_export]
macro_rules! start_pixel_default {
    ($t:ident, $p:expr) => {{
        $t.current_pixel = $p;
        $t.current_pixel_sample_index = 0;
        $t.array_1d_offset = 0;
        $t.array_2d_offset = 0;
    }}
}

#[macro_export]
macro_rules! set_sample_number_default {
    ($t:ident, $sample_number:expr) => {{
        $t.array_1d_offset = 0;
        $t.array_2d_offset = 0;
        $t.current_pixel_sample_index = $sample_number;

        $t.current_pixel_sample_index < $t.samples_per_pixel
    }}
}

#[macro_export]
macro_rules! start_next_sample_default {
    ($t:ident) => {{
        $t.array_1d_offset = 0;
        $t.array_2d_offset = 0;
        $t.current_pixel_sample_index += 1;

        $t.current_pixel_sample_index < $t.samples_per_pixel
    }}
}

#[macro_export]
macro_rules! current_sample_number_default {
    () => {
        fn current_sample_number(&self) -> u64 {
            self.current_pixel_sample_index
        }
    }
}


#[macro_export]
macro_rules! request_1d_array_default {
    () => {
        fn request_1d_array(&mut self, n: usize) {
            assert!(self.round_count(n) > n);
            self.samples_1d_array_sizes.push(n);
            self.sample_array_1d.push(vec![0.0; n * self.samples_per_pixel as usize]);
        }
    }
}

#[macro_export]
macro_rules! request_2d_array_default {
    () => {
        fn request_2d_array(&mut self, n: usize) {
            assert!(self.round_count(n) > n);
            self.samples_2d_array_sizes.push(n);
            self.sample_array_2d.push(vec![Default::default(); n * self.samples_per_pixel as usize]);
        }
    }
}

#[macro_export]
macro_rules! get_1d_array_default {
    () => {
        fn get_1d_array(&mut self, n: usize) -> Option<&[Float]> {
            if self.array_1d_offset == self.sample_array_1d.len() {
                return None;
            }

            assert_eq!(self.samples_1d_array_sizes[self.array_1d_offset], n);
            assert!(self.current_pixel_sample_index < self.samples_per_pixel);

            let start = self.current_pixel_sample_index * n as u64;
            let end = start + n as u64;
            let samples = &self.sample_array_1d[self.array_1d_offset][start as usize..end as usize];
            self.array_1d_offset += 1;

            Some(samples)
        }
    }
}

#[macro_export]
macro_rules! get_2d_array_default {
    () => {
        fn get_2d_array(&mut self, n: usize) -> Option<&[Point2f]> {
            if self.array_2d_offset == self.sample_array_2d.len() {
                return None;
            }

            assert_eq!(self.samples_2d_array_sizes[self.array_2d_offset], n);
            assert!(self.current_pixel_sample_index < self.samples_per_pixel);

            let start = self.current_pixel_sample_index * n as u64;
            let end = start + n as u64;
            let samples = &self.sample_array_2d[self.array_2d_offset][start as usize..end as usize];
            self.array_2d_offset += 1;

            Some(samples)
        }
    }
}

#[macro_export]
macro_rules! get_camera_sample_default {
    () => {
        fn get_camera_sample(&mut self, p_raster: &Point2i) -> CameraSample {
            CameraSample {
                pfilm: Point2f::from(*p_raster),
                time: self.get_1d(),
                plens: self.get_2d()
            }
        }
    }
}

// PixelSampler

#[macro_export]
macro_rules! pixel_sampler_new {
    ($samples_per_pixel:expr, $sample_dims:ident, $p:ident) =>{{
        crate::sampler_new!($samples_per_pixel, $p);

        for _ in 0..$sample_dims {
            $p.samples_1d.push(vec![0.0; $samples_per_pixel as usize]);
            $p.samples_2d.push(vec![Default::default(); $samples_per_pixel as usize]);
        }
    }}
}

#[macro_export]
macro_rules! pixel_start_next_sample {
    () => {
        fn start_next_sample(&mut self) -> bool {
            self.current_1d_dimension = 0;
            self.current_2d_dimension = 0;
            crate::start_next_sample_default!(self)
        }
    }
}

#[macro_export]
macro_rules! pixel_set_sample_number {
    () => {
        fn set_sample_number(&mut self, sample_num: u64) -> bool {
            self.current_1d_dimension = 0;
            self.current_2d_dimension = 0;

            crate::set_sample_number_default!(self, sample_num)
        }
    }
}

#[macro_export]
macro_rules! pixel_get_1d {
    () => {
        fn get_1d(&mut self) -> Float {
            assert!(self.current_pixel_sample_index < self.samples_per_pixel);

            if self.current_1d_dimension < self.samples_1d.len() {
                let sample = self.samples_1d[self.current_1d_dimension][self.current_pixel_sample_index as usize];
                self.current_1d_dimension += 1;
                sample
            } else {
                self.rng.uniform_float()
            }
        }
    }
}

#[macro_export]
macro_rules! pixel_get_2d {
    () => {
        fn get_2d(&mut self) -> Point2f {
            assert!(self.current_pixel_sample_index < self.samples_per_pixel);

            if self.current_2d_dimension < self.samples_2d.len() {
                let sample = self.samples_2d[self.current_2d_dimension][self.current_pixel_sample_index as usize];
                self.current_2d_dimension += 1;
                sample
            } else {
                Point2f::new(self.rng.uniform_float(), self.rng.uniform_float())
            }
        }
    }
}

// Global Sampler
#[macro_export]
macro_rules! global_start_next_sample {
    () => {
        fn start_next_sample(&mut self) -> bool {
            self.dimension = 0;
            self.interval_sample_index = self.get_index_for_sample(self.current_pixel_sample_index + 1);
            start_next_sample_default!(self)
        }
    }
}

#[macro_export]
macro_rules! global_start_pixel {
    () => {
        fn start_pixel(&mut self, p: &Point2i) {
            start_pixel_default!(self, *p);
            self.dimension = 0;
            self.interval_sample_index = self.get_index_for_sample(0);

            // Compute arrayEndDim for dimensions used for array samples
            self.array_end_dim =
                crate::core::sampler::ARRAY_START_DIM +
                self.sample_array_1d.len() + 2 * self.sample_array_2d.len();

            // Compute 1D array samples for GlobalSampler
            for i in 0..self.samples_1d_array_sizes.len() {
                let nsamples = self.samples_1d_array_sizes[i] as u64 * self.samples_per_pixel;

                for j in 0..nsamples {
                    let index = self.get_index_for_sample(j);
                    self.sample_array_1d[i][j as usize] =
                        self.sample_dimension(index, crate::core::sampler::ARRAY_START_DIM + i);
                }
            }

            // Compute 2D array samples for GlobalSampler
            let mut dim = crate::core::sampler::ARRAY_START_DIM + self.samples_1d_array_sizes.len();

            for i in 0..self.samples_2d_array_sizes.len() {
                let nsamples = self.samples_2d_array_sizes[i] as u64 * self.samples_per_pixel;
                for j in 0..nsamples {
                    let idx = self.get_index_for_sample(j);
                    self.sample_array_2d[i][j as usize].y = self.sample_dimension(idx, dim + 1);
                    self.sample_array_2d[i][j as usize].x = self.sample_dimension(idx, dim);
                }

                dim += 2;
            }

            assert_eq!(self.array_end_dim, dim);
        }
    }
}

#[macro_export]
macro_rules! global_set_sample_number {
    () => {
        fn set_sample_number(&mut self, sample_num: u64) -> bool {
            self.dimension = 0;
            self.interval_sample_index = self.get_index_for_sample(sample_num);
            crate::set_sample_number_default!(self, sample_num)
        }
    }
}

#[macro_export]
macro_rules! global_get_1d {
    () => {
        fn get_1d(&mut self) -> Float {
            if self.dimension >= crate::core::sampler::ARRAY_START_DIM && self.dimension < self.array_end_dim {
                self.dimension = self.array_end_dim;
            }

            let res = self.sample_dimension(self.interval_sample_index, self.dimension);
            self.dimension += 1;

            res
        }
    }
}

#[macro_export]
macro_rules! global_get_2d {
    () => {
        fn get_2d(&mut self) -> Point2f {
            if self.dimension + 1 >= crate::core::sampler::ARRAY_START_DIM && self.dimension < self.array_end_dim {
                self.dimension = self.array_end_dim;
            }

            let y = self.sample_dimension(self.interval_sample_index, self.dimension + 1);
            let x = self.sample_dimension(self.interval_sample_index, self.dimension);

            let p = Point2f::new(x, y);

            self.dimension += 2;

            p
        }
    }
}
