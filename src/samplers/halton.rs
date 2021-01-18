use lazy_static::lazy_static;
use crate::core::sampler::*;
use crate::*;
use crate::core::lowdiscrepancy::*;
use crate::core::geometry::point::{Point2i, Point2f};
use std::cell::Cell;
use crate::core::geometry::bounds::Bounds2i;
use crate::core::pbrt::{Float, mod_};
use crate::core::camera::CameraSample;
use crate::core::paramset::ParamSet;

lazy_static! {
    static ref RadicalInversePermutations: Vec<u16> = compute_radical_inverse_permutations(&mut Default::default());
}
const ARRAY_START_DIM: usize = 5;
const K_MAX_RESOLUTION: usize = 128;

pub fn extended_gcd(a: i64, b: i64, x: i64, y: i64) -> (i64, i64) {
    if b == 0 {
        return (1, 0)
    }

    let d = a / b;
    let xp = 0;
    let yp = 0;
    let (r1, r2) = extended_gcd(b, a % b, xp, yp);

    (r2, r1 - (d * r2))
}

pub fn multiplicative_inverse(a: i64, n: i64) -> i64 {
    let (x, _) = extended_gcd(a, n, 0, 0);

    mod_(x, n)
}

#[derive(Default, Clone)]
pub struct HaltonSampler {
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
    // HaltonSampler data
    base_scales                 : Point2i,
    base_exponents              : Point2i,
    sample_stride               : isize,
    mult_inverse                : [i64; 2],
    offset_for_current_pixel    : Cell<usize>,
    pixel_for_offset            : Cell<Point2i>,
    sample_at_pixelcenter       : bool
}

impl HaltonSampler {
    fn new(samples_per_pixel: usize, sample_bounds: &Bounds2i, sample_at_pixelcenter: bool) -> Self {
        let res = sample_bounds.p_max - sample_bounds.p_min;
        let mut base_scales = Point2i::default();
        let mut base_exponents = Point2i::default();

        // Find radical inverse base scales and exponents that cover sampling area
        for i in 0..2 {
            let base = if i == 0 { 2 } else { 3 };

            let mut scale = 1;
            let mut exp = 0;

            while scale < res[i].min(K_MAX_RESOLUTION as isize) {
                scale *= base;
                exp += 1;
            }
            base_scales[i] = scale;
            base_exponents[i] = exp;
        }

        // Compute stride in samples for visiting each pixel area
        let sample_stride = base_scales[0] * base_scales[1];

        // Compute multiplicative inverses for base_scales
        let mult_inverse: [i64; 2] = [
            multiplicative_inverse(base_scales[1] as i64, base_scales[0] as i64),
            multiplicative_inverse(base_scales[0] as i64, base_scales[1] as i64)
        ];

        let mut h = Self {
            base_scales,
            base_exponents,
            sample_at_pixelcenter,
            sample_stride,
            mult_inverse,
            ..Default::default()
        };

        sampler_new!(samples_per_pixel as u64, h);

        h
    }
}

impl GlobalSampler for HaltonSampler {
    fn get_index_for_sample(&self, sample_num: u64) -> u64 {
        unimplemented!()
    }

    fn sample_dimension(&self, index: u64, dimention: usize) -> f32 {
        unimplemented!()
    }
}

impl Sampler for HaltonSampler {
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

pub fn create_halton_sampler(params: &ParamSet, bounds: &Bounds2i, quick_render: bool) -> Samplers {
    let mut nsamp = params.find_one_int("pixelsamples", 16);

    if quick_render {
        nsamp = 1;
    }

    let samp_at_center = params.find_one_bool("samplepixelcenter", false);

    HaltonSampler::new(nsamp as usize, bounds, samp_at_center).into()
}
