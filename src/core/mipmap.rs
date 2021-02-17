use crate::core::memory::BlockedArray;
use log::info;
use lazy_static::lazy_static;
use crate::core::pbrt::{Float, is_power_of2, round_up_pow2_32, mod_, clamp, INFINITY, lerp};
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::geometry::vector::Vector2f;
use ndarray::prelude::*;
use ndarray::parallel::prelude::*;
use crate::core::texture::lanczos;
use std::ops::{Mul, AddAssign, Div};
use crate::core::spectrum::{RGBSpectrum, SampledSpectrum};
use ndarray::Zip;

stat_counter!("Texture/EWA lookups", newa_lookups);
stat_counter!("Texture/Trilinear lookups", ntrilerp_lookups);
stat_memory_counter!("Memory/Texture MIP maps", mipmap_memory);

#[macro_export]
macro_rules! mipmap {
    ($res:expr, $data:expr) => {
        MIPMap::new(
            $res, $data, false, 8.0,
            crate::core::mipmap::ImageWrap::Repeat)
    }
}

pub fn init_stats() {
    newa_lookups::init();
    ntrilerp_lookups::init();
    mipmap_memory::init();
}

const WEIGHT_LUT_SIZE: usize = 128;

lazy_static! {
    static ref WEIGHT_LUT: [Float; WEIGHT_LUT_SIZE] = initialize_weightlut();
}

fn initialize_weightlut() -> [Float; WEIGHT_LUT_SIZE] {
    let mut wl= [0 as Float; WEIGHT_LUT_SIZE];

    for (i, w) in wl.iter_mut().enumerate() {
        let alpha = 2.0;
        let r2 = i as Float / (WEIGHT_LUT_SIZE - 1) as Float;
        *w = (-alpha * r2).exp() - (-alpha).exp();
    }

    wl
}

#[derive(Eq, PartialEq, Ord, PartialOrd, Copy, Clone)]
pub enum ImageWrap { Repeat, Black, Clamp }

#[derive(Default, Copy, Clone)]
pub struct ResampleWeight {
    first_texel: isize,
    weight: [Float; 4]
}

pub struct MIPMap<T>
where T: num::Zero + Clone + Copy + Send + Sync
{
    pyramid         : Vec<BlockedArray<T>>,
    do_trilinear    : bool,
    max_anisotropy  : Float,
    wrap            : ImageWrap,
    resolution      : Point2i,
    black           : T
}



impl<T> MIPMap<T>
where T: num::Zero + Clone + Copy + Send + Sync + Mul<Float, Output=T> + AddAssign<T> + Div<Float, Output=T> + Clampable
{
    pub fn new(
        res: &Point2i, data: Vec<T>, do_trilinear: bool,
        max_anisotropy: Float, wrap: ImageWrap) -> Self {
        // TODO: ProfilePhase
        let (r, img) = if !is_power_of2(res[0]) || !is_power_of2(res[1]) {
            // Resample image to power-of-two resolution
            let res_pow2 = Point2i::new(
                round_up_pow2_32(res[0] as i32) as isize,
                round_up_pow2_32(res[1] as i32) as isize);

            info!("Resampling MIPMap from {} to {}. Ratio= {}",
                  res, res_pow2, (res_pow2.x * res_pow2.y) as Float / (res.x * res.y) as Float);

            // Resample image in s direction
            let sweights = MIPMap::<T>::resample_weights(res[0] as usize, res_pow2[0] as usize);
            let mut resampled = Array2::<T>::zeros((res_pow2.y as usize, res_pow2.x as usize));

            resampled
                .axis_iter_mut(Axis(0))
                .into_par_iter()
                .take(res.y as usize)
                .enumerate()
                .for_each(|(t, mut row)| {
                    for s in 0..res_pow2[0] as usize {
                        // Compute texel (s, t) in s-zoomed image
                        for j in 0..4 {
                            let mut origs = sweights[s].first_texel as isize + j as isize;
                            origs = match wrap {
                                ImageWrap::Repeat => mod_(origs, res[0]),
                                ImageWrap::Clamp  => clamp(origs, 0, res[0] - 1),
                                _ => origs
                            };

                            if origs >= 0 && origs < res[0] {
                                row[s] += data[t * res[0] as usize + origs as usize] * sweights[s].weight[j];
                            }
                        }
                    }
                });

            // Resample image in t direction
            let tweights = MIPMap::<T>::resample_weights(res[1] as usize, res_pow2[1] as usize);

            resampled
                .axis_iter_mut(Axis(1))
                .into_par_iter()
                .for_each(|mut col| {
                    let mut work_data = vec![T::zero(); res_pow2[1] as usize];

                    for t in 0..res_pow2[1] as usize {
                        for j in 0..4 {
                            let mut offset = tweights[t].first_texel + j;
                            offset = match wrap {
                                ImageWrap::Repeat => mod_(offset, res[1]),
                                ImageWrap::Clamp  => clamp(offset, 0, res[1] - 1),
                                _ => offset
                            };

                            if offset >= 0 && offset < res[1] {
                                work_data[t] += col[offset as usize] * tweights[t].weight[j as usize]
                            }
                        }
                    }

                    for t in 0..res_pow2[1] as usize {
                        col[t] = Clampable::clamp(work_data[t], 0.0, INFINITY);
                    }
                });

            (res_pow2, resampled)
        } else {
            (
                *res,
                ArrayView2::from_shape((res.x as usize, res.y as usize), &data).unwrap().to_owned()
            )
        };

        // Initialize levels of MIPMap from image
        let nlevels = 1 + (std::cmp::max(r.x, r.y) as Float).log2() as usize;
        let mut pyramid = Vec::with_capacity(nlevels);
        pyramid.push(BlockedArray::new(
            r.x as usize,
            r.y as usize,
            Some(img.view().as_slice().unwrap())));

        let mut mipmap = Self {
            do_trilinear,
            max_anisotropy,
            wrap,
            resolution: r,
            black: T::zero(),
            pyramid
        };

        for i in 1..nlevels {
            // Initialize ith MIPMap level from i-1st level
            let sres = std::cmp::max(1, mipmap.pyramid[i - 1].ures() / 2);
            let tres = std::cmp::max(1, mipmap.pyramid[i - 1].vres() / 2);
            let mut d = Array2::zeros((tres, sres));

            // Filter four texels from finer level of pyramid
            Zip::indexed(&mut d).par_apply(|(t, s), p| {
                let (si, ti) = (s as isize, t as isize);
                *p =
                    (*mipmap.texel(i - 1, 2 * si, 2 * ti) +
                     *mipmap.texel(i - 1, 2 * si + 1, 2 * ti) +
                     *mipmap.texel(i - 1, 2 * si, 2 * ti + 1) +
                     *mipmap.texel(i - 1, 2 * si + 1, 2 * ti + 1 + 1)) *
                    0.25;
            });

            mipmap.pyramid.push(BlockedArray::new(
                sres,
                tres,
                Some(d.view().as_slice().unwrap())));
        }

        // Add memory stats for mipmap_memory
        mipmap_memory::add((4 * r.x as u64 * r.y as u64 * std::mem::size_of::<T>() as u64) / 3);

        mipmap
    }
    
    pub fn width(&self) -> usize { self.resolution[0] as usize }
    pub fn height(&self) -> usize { self.resolution[1] as usize }

    pub fn lookup(&self, st: &Point2f, width: Float) -> T {
        ntrilerp_lookups::inc();
        // TODO: ProfilePhase
        // Compute MIPMap level for trilinear filtering
        let level = (self.levels() - 1) as Float + width.max(1.0e-8);

        // Perform trilinear interpolation at appropriate MIPMap level
        if level < 0.0 {
            self.triangle(0, st)
        } else if level >= (self.levels() - 1) as Float {
            *self.texel(self.levels() - 1, 0, 0)
        } else {
            let ilevel = level.floor();
            let delta = level - ilevel;

            lerp(
                delta,
                self.triangle(ilevel as usize, st),
                self.triangle(ilevel as usize + 1, st)
            )
        }
    }

    pub fn lookup2(&self, st: &Point2f, mut dst0: Vector2f, mut dst1: Vector2f) -> T {
        if self.do_trilinear {
            let (x, y) = (
                dst0.x.abs().max(dst0.y.abs()),
                dst1.x.abs().max(dst1.y.abs())
                );
            let width = x.max(y);

            return self.lookup(st, width)
        }

        newa_lookups::inc();
        // TODO: ProfilePhase
        // Compute ellipse minor and major axis
        if dst0.length_squared() < dst1.length_squared() {
            std::mem::swap(&mut dst0, &mut dst1);
        }

        let majorl = dst0.length();
        let mut minorl = dst1.length();

        // Clamp ellipse eccentricity if too large
        if minorl * self.max_anisotropy < majorl && minorl > 0.0 {
            let scale = majorl / (minorl * self.max_anisotropy);
            dst1 *= scale;
            minorl *= scale;
        }

        if minorl == 0.0 { return self.triangle(0, st); }

        // Choose level of detail for EWA lookup and perform EWA filtering
        let lod = (self.levels() as Float - 1.0 + minorl.log2()).max(0.0);
        let ilod = lod.floor() as usize;

        lerp(
            lod - ilod as Float,
            self.ewa(ilod, *st, dst0, dst1),
            self.ewa(ilod + 1, *st, dst0, dst1))
    }

    pub fn levels(&self) -> usize {
        self.pyramid.len()
    }

    pub fn resample_weights(oldres: usize, newres: usize) -> Vec<ResampleWeight> {
        assert!(newres >= oldres);

        let mut wt: Vec<ResampleWeight> = vec![Default::default(); newres];
        let filter_width = 2.0;

        for i in 0..newres {
            // Compute resampling weights for ith texel
            let center = (i as Float + 0.5) * oldres as Float / newres as Float;
            wt[i].first_texel = ((center - filter_width) + 0.5).floor() as isize;

            for i in 0..4 {
                for j in 0..4 {
                    let pos = wt[i].first_texel as Float + j as Float + 0.5;
                    wt[i].weight[j] = lanczos((pos - center)/ filter_width, 2.0);
                }
            }

            // Normalize filter weights for texel sampling
            let inv_sum_wts = 1.0 / (wt[i].weight[0] + wt[i].weight[1] +
                                     wt[i].weight[2] + wt[i].weight[3]);

            for j in 0..4 { wt[i].weight[j] *= inv_sum_wts }
        }

        wt
    }

    fn texel(&self, level: usize, s: isize, t: isize) -> &T {
        assert!(level < self.pyramid.len());

        let l = &self.pyramid[level];
        let (u, v)  = (l.ures() as isize, l.vres() as isize);

        // Compute texel (s, t) accounting for boundary conditions
        let (si, ti) = match self.wrap {
            ImageWrap::Repeat => (mod_(s, u), mod_(t, v)),
            ImageWrap::Clamp  => (clamp(s, 0, u), clamp(t, 0, v)),
            _ => {
                if s < 0 || s >= u || t < 0 || t >= v { return &self.black; }

                (s, t)
            }
        };

        &l[(si as usize, ti as usize)]
    }

    fn triangle(&self, mut level: usize, st: &Point2f) -> T {
        level = clamp(level, 0, self.levels() - 1);
        let s = st.x as Float * self.pyramid[level].ures() as Float - 0.5;
        let t = st.y as Float * self.pyramid[level].vres() as Float - 0.5;
        let (s0, t0) = (s.floor() as isize, t.floor() as isize);
        let (ds, dt) = (s - s0 as Float, t - t0 as Float);

        *self.texel(level, s0, t0) * (1.0 - dt) * (1.0 - ds) +
        *self.texel(level, s0, t0 + 1) * dt * (1.0 - ds) +
        *self.texel(level, s0 + 1, t0) * (1.0 - dt) * ds +
        *self.texel(level, s0 + 1, t0 + 1) * dt * ds
    }

    fn ewa(&self, level: usize, mut st: Point2f,
           mut dst0: Vector2f, mut dst1: Vector2f) -> T {
        // Convert ellipse coefficients to bound EWA filter region
        st.x = st.x * self.pyramid[level].ures() as Float - 0.5;
        st.y = st.y * self.pyramid[level].vres() as Float - 0.5;
        dst0.x *= self.pyramid[level].ures() as Float;
        dst0.y *= self.pyramid[level].vres() as Float;
        dst1.x *= self.pyramid[level].ures() as Float;
        dst1.y *= self.pyramid[level].vres() as Float;

        // Compute ellipse coefficients to bound EWA filter region
        let mut A = dst0[1] * dst0[1] + dst1[1] * dst1[1] + 1.0;
        let mut B = -2.0 * (dst0[0] * dst0[1] + dst1[0] * dst1[1]);
        let mut C = dst0[0] * dst0[0] + dst1[0] * dst1[0] + 1.0;
        let invf = 1.0 / (A * C - B * B * 0.25);
        A *= invf;
        B *= invf;
        C *= invf;

        // Compute ellipse's (s, t) bounding box in texture space
        let det = -B * B + 4.0 * A * C;
        let idet = 1.0 / det;
        let usqrt = (det * C).sqrt();
        let vsqrt = (det * A).sqrt();
        let s0 = (st[0] - 2.0 * idet * usqrt).ceil() as isize;
        let s1 = (st[0] + 2.0 * idet * usqrt).floor() as isize;
        let t0 = (st[1] - 2.0 * idet * vsqrt).ceil() as isize;
        let t1 = (st[1] + 2.0 * idet * vsqrt).floor() as isize;

        // Scan over ellipse bound and compute quadratic equation
        let mut sum = T::zero();
        let mut sum_wts = 0.0;

        for it in t0..=t1 {
            let tt = it as Float - st.y;

            for is in s0..=s1 {
                let ss = is as Float - st.x;

                // Compute squared radius and filter texel if inside ellipse
                let r2 = A * ss * ss + B * ss * tt + C * tt * tt;

                if r2 < 1.0 {
                    let index =
                        std::cmp::min((r2 * WEIGHT_LUT_SIZE as Float) as usize, WEIGHT_LUT_SIZE - 1);
                    let weight = WEIGHT_LUT[index];
                    sum += *self.texel(level, is, it) * weight;
                    sum_wts += weight
                }
            }
        }

        sum / sum_wts
    }
}

pub trait Clampable {
    fn clamp(self, low: Float, high: Float) -> Self;
}

impl Clampable for Float {
    fn clamp(self, low: f32, high: f32) -> Self {
        crate::core::pbrt::clamp(self, low, high)
    }
}

impl Clampable for RGBSpectrum {
    fn clamp(self, low: f32, high: f32) -> Self {
        RGBSpectrum::clamps(&self, low, high)
    }
}

impl Clampable for SampledSpectrum {
    fn clamp(self, low: f32, high: f32) -> Self {
        SampledSpectrum::clamps(&self, low, high)
    }
}