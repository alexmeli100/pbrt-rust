use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::pbrt::{Float, is_power_of2, clamp, Options, round_up_pow2_32, log2_int};
use crate::core::geometry::bounds::Bounds2i;
use log::warn;
use crate::*;
use crate::core::sampler::{Sampler, Samplers, GlobalSampler};
use crate::core::camera::CameraSample;
use crate::core::lowdiscrepancy::{sobol_interval_to_index, sobol_sample};
use crate::core::sobolmatrices::NUM_SOBOL_DIMENSIONS;
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::paramset::ParamSet;

#[derive(Default, Clone)]
pub struct SobolSampler {
    sample_bounds               : Bounds2i,
    resolution                  : i32,
    log2_resolution             : i32,
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
    // GlobalSampler data
    dimension                   : usize,
    interval_sample_index       : u64,
    array_end_dim               : usize,
}

impl SobolSampler {
    pub fn new(samples_per_pixel: u64, sbounds: &Bounds2i) -> Self {
        if !is_power_of2(samples_per_pixel) {
            warn!(
                "Non power-of-two sample count rounded up to {} \
                for SobolSampler.", samples_per_pixel);
        }

        let diag = sbounds.diagonal();
        let resolution = round_up_pow2_32(std::cmp::max(diag.x as i32, diag.y as i32));
        let log2_resolution = log2_int(resolution);
        if resolution > 0 { assert_eq!(1 << log2_resolution, resolution); }

        let mut ss = SobolSampler {
            resolution,
            log2_resolution,
            sample_bounds: *sbounds,
            ..Default::default()
        };

        sampler_new!(samples_per_pixel, ss);

        ss
    }
}

impl GlobalSampler for SobolSampler {
    fn get_index_for_sample(&self, sample_num: u64) -> u64 {
        sobol_interval_to_index(
            self.log2_resolution as u32,
            sample_num,
            &Point2i::from(self.current_pixel - self.sample_bounds.p_min))
    }

    fn sample_dimension(&self, index: u64, dim: usize) -> Float {
        if dim >= NUM_SOBOL_DIMENSIONS {
            panic!(
                "SobolSampler can only sample up to {} \
                dimensions! Exiting.", NUM_SOBOL_DIMENSIONS);
        }

        let mut s = sobol_sample(index, dim, 0);

        // Remap sobol' dimensions used for pixel samples
        if dim == 0 || dim == 1 {
            s = s * self.resolution as Float + self.sample_bounds.p_min[dim] as Float;
            s = clamp(s - self.current_pixel[dim] as Float, 0.0, ONE_MINUS_EPSILON)
        }


        
        s
    }
}

impl Sampler for SobolSampler {
    get_sampler_data!();

    global_start_pixel!();

    global_get_1d!();

    global_get_2d!();

    get_camera_sample_default!();

    request_1d_array_default!();

    request_2d_array_default!();

    get_1d_array_default!();

    get_2d_array_default!();

    global_start_next_sample!();

    global_set_sample_number!();

    current_sample_number_default!();

    fn clone(&self, _seed: isize) -> Samplers {
        Clone::clone(self).into()
    }
}

pub fn create_sobol_sampler(params: &ParamSet, sbounds: &Bounds2i, opts: &Options) -> Option<Box<Samplers>> {
    let  mut nsamp = params.find_one_int("pixelsamples", 16);
    if opts.quick_render { nsamp = 1; }
    
    Some(Box::new(SobolSampler::new(nsamp as u64, sbounds).into()))
}