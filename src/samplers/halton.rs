use lazy_static::lazy_static;
use crate::core::sampler::*;
use crate::*;
use crate::core::lowdiscrepancy::*;
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::geometry::bounds::Bounds2i;
use crate::core::pbrt::{Float, mod_};
use crate::core::camera::CameraSample;
use crate::core::paramset::ParamSet;
use static_assertions::_core::sync::atomic::{AtomicU64, AtomicIsize};
use std::sync::atomic::Ordering;

lazy_static! {
    static ref RadicalInversePermutations: Vec<u16> = compute_radical_inverse_permutations(&mut Default::default());
}

const K_MAX_RESOLUTION: usize = 128;

pub fn extended_gcd(a: i64, b: i64) -> (i64, i64) {
    if b == 0 {
        return (1, 0)
    }

    let d = a / b;
    let (r1, r2) = extended_gcd(b, a % b);

    (r2, r1 - (d * r2))
}

pub fn multiplicative_inverse(a: i64, n: i64) -> i64 {
    let (x, _) = extended_gcd(a, n);

    mod_(x, n)
}

#[derive(Default)]
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
    sample_stride               : u64,
    mult_inverse                : [i64; 2],
    offset_for_current_pixel    : AtomicU64,
    pixel_for_offsetx           : AtomicIsize,
    pixel_for_offsety           : AtomicIsize,
    sample_at_pixelcenter       : bool
}

impl HaltonSampler {
    pub fn new(samples_per_pixel: usize, sample_bounds: &Bounds2i, sample_at_pixelcenter: bool) -> Self {
        // Generate random digit permutations for Halton sampler
        lazy_static::initialize(&RadicalInversePermutations);

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
        let sample_stride = (base_scales[0] * base_scales[1]) as u64;

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

    fn permutation_for_dimension(&self, dim: usize) -> &[u16] {
        if dim >= PRIME_TABLE_SIZE {
            panic!("HaltonSampler can only sample {} dimensions", PRIME_TABLE_SIZE);
        }

        &RadicalInversePermutations[PRIME_SUMS[dim]..]
    }
}

impl GlobalSampler for HaltonSampler {
    fn get_index_for_sample(&self, sample_num: u64) -> u64 {
        let pox = self.pixel_for_offsetx.load(Ordering::Relaxed);
        let poy = self.pixel_for_offsety.load(Ordering::Relaxed);
        let pfo = Point2i::new(pox as isize, poy as isize);
        if self.current_pixel != pfo {
            // Compute Halton sample offset for currentPixel
            self.offset_for_current_pixel.store(0, Ordering::Relaxed);

            if self.sample_stride > 1 {
                let pm = Point2i::new(
                    mod_(self.current_pixel[0], K_MAX_RESOLUTION as isize),
                    mod_(self.current_pixel[1], K_MAX_RESOLUTION as isize));

                for i in 0..2 {
                    let dimoffset = if i == 0 {
                        inverse_radical_inverse::<2>(pm[i] as u64, self.base_exponents[i] as u64)
                    } else {
                        inverse_radical_inverse::<3>(pm[i] as u64, self.base_exponents[i] as u64)
                    };

                    let val = dimoffset * (self.sample_stride / self.base_scales[i] as u64) * self.mult_inverse[i] as u64;
                    self.offset_for_current_pixel.fetch_add(val, Ordering::Relaxed);
                }

                let ofcp = self.offset_for_current_pixel.load(Ordering::Relaxed);
                self.offset_for_current_pixel.store(ofcp % self.sample_stride as u64, Ordering::Relaxed);
            }

            self.pixel_for_offsetx.store(self.current_pixel[0], Ordering::Relaxed);
            self.pixel_for_offsety.store(self.current_pixel[1], Ordering::Relaxed);
        }

        let ofcp = self.offset_for_current_pixel.load(Ordering::Relaxed);

        ofcp as u64 + sample_num * self.sample_stride as u64
    }

    fn sample_dimension(&self, index: u64, dim: usize) -> Float {
        if self.sample_at_pixelcenter && (dim == 0 || dim == 1) { return 0.5 }

        match dim {
            0 => radical_inverse(dim, index >> self.base_exponents[0] as u64),
            1 => radical_inverse(dim, index / self.base_scales[1] as u64),
            _ => scrambled_radical_inverse(dim, index, self.permutation_for_dimension(dim))
        }
    }
}

impl Sampler for HaltonSampler {
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
        let pixel_for_offsetx: isize = self.pixel_for_offsetx.load(Ordering::Relaxed);
        let pixel_for_offsety: isize = self.pixel_for_offsety.load(Ordering::Relaxed);
        let offset_for_current_pixel: u64 = self.offset_for_current_pixel.load(Ordering::Relaxed);

        let h = HaltonSampler {
            current_pixel               : self.current_pixel,
            current_pixel_sample_index  : self.current_pixel_sample_index,
            samples_1d_array_sizes      : self.samples_1d_array_sizes.to_vec(),
            samples_2d_array_sizes      : self.samples_2d_array_sizes.to_vec(),
            sample_array_1d             : self.sample_array_1d.to_vec(),
            sample_array_2d             : self.sample_array_2d.to_vec(),
            array_1d_offset             : self.array_1d_offset,
            array_2d_offset             : self.array_2d_offset,
            samples_per_pixel           : self.samples_per_pixel,
            dimension                   : self.dimension,
            interval_sample_index       : self.interval_sample_index,
            array_end_dim               : self.array_end_dim,
            base_scales                 : self.base_scales,
            base_exponents              : self.base_exponents,
            sample_stride               : self.sample_stride,
            pixel_for_offsetx           : AtomicIsize::new(pixel_for_offsetx),
            pixel_for_offsety           : AtomicIsize::new(pixel_for_offsety),
            offset_for_current_pixel    : AtomicU64::new(offset_for_current_pixel),

            mult_inverse                : self.mult_inverse,
            sample_at_pixelcenter       : self.sample_at_pixelcenter
        };

        h.into()
    }
}

pub fn create_halton_sampler(params: &ParamSet, bounds: &Bounds2i, quick_render: bool) -> Option<Box<Samplers>> {
    let mut nsamp = params.find_one_int("pixelsamples", 16);

    if quick_render {
        nsamp = 1;
    }

    let samp_at_center = params.find_one_bool("samplepixelcenter", false);

    Some(Box::new(HaltonSampler::new(nsamp as usize, bounds, samp_at_center).into()))
}
