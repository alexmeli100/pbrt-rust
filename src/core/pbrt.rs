use std::ops::{Sub, Add, Mul, BitAnd};

use num::{One, Zero};
use num::traits::Pow;
use std::sync::{Arc, Weak};
use std::path::PathBuf;
use lazy_static::lazy_static;
use indicatif::ProgressBar;
use parking_lot::RwLock;

lazy_static! {
    static ref PB: RwLock<Option<Weak<ProgressBar>>> = RwLock::new(None);
}

pub fn set_progress_bar(pb: Option<Weak<ProgressBar>>) {
    *PB.write() = pb;
}

pub fn get_progress_bar() -> Option<Arc<ProgressBar>> {
    PB.read().as_ref()?.upgrade()
}

pub type Float = f32;

pub const PI                : Float = 3.14159265358979323846;
pub const PI_OVER2          : Float = 1.57079632679489661923;
pub const PI_OVER4          : Float = 0.78539816339744830961;
pub const INV_PI            : Float = 0.31830988618379067154;
pub const INV2_PI           : Float = 0.15915494309189533577;
pub const INV4_PI           : Float = 0.07957747154594766788;
pub const INFINITY          : Float = std::f32::INFINITY;
pub const SHADOW_EPSILON    : Float = 0.0001;
pub const MACHINE_EPSILON   : Float = std::f32::EPSILON * 0.5;
pub const SQRT2             : Float = 1.41421356237309504880;

#[derive(Default, Clone)]
pub struct Options {
    pub quick_render    : bool,
    pub quiet           : bool,
    pub cat             : bool,
    pub to_ply          : bool,
    pub image_file      : PathBuf,
    pub integrator_name : String,
    pub crop_window     : [[Float; 2]; 2]
}

impl Options {
    pub fn new() -> Self {
        Self {
            crop_window: [[0.0, 1.0], [0.0, 1.0]],
            ..Default::default()
        }
    }
}

#[inline(always)]
pub fn float_to_bits(f: f32) -> u32 {
    let ui: u32;

    unsafe {
        let res: u32 = std::mem::transmute_copy(&f);
        ui = res;
    }

    ui
}

#[inline(always)]
pub fn bits_to_float(ui: u32) -> f32 {
    let f: f32;

    unsafe {
        let res: f32 = std::mem::transmute_copy(&ui);
        f = res;
    }

    f
}

pub fn next_float_up(v: f32) -> f32 {
    if v.is_infinite() && v > 0.0 { return v; }

    let mut i = v;
    if i == -0.0 { i = 0.0; }

    let mut ui = float_to_bits(i);

    if i >= 0.0 {
        ui += 1;
    } else {
        ui -= 1;
    }

    bits_to_float(ui)
}

pub fn next_float_down(v: f32) -> f32 {
    if v.is_infinite() && v < 0.0 { return v; }

    let mut i = v;
    if i == 0.0 { i = -0.0; }

    let mut ui = float_to_bits(i);

    if i > 0.0 {
        ui -= 1;
    } else {
        ui += 1;
    }

    bits_to_float(ui)
}

pub fn log2(x: Float) -> Float {
    let inv_log2 = 1.442695040888963387004650940071;

    x.ln() * inv_log2
}

pub fn log2_uint(v: u32) -> i32 {
    31_i32 - v.leading_zeros() as i32
}

pub fn log2_int(v: i32) -> i32 {
    log2_uint(v as u32)
}

pub fn log2_uint64(v: u64) -> i64 {
    63 - v.leading_zeros() as i64
}

pub fn log2_int64(v: i64) -> i64 {
    log2_uint64(v as u64)
}

pub fn lerp<T, S>(t: S, x: T, y: T) -> T
    where
        S: Copy + num::One + Sub<S, Output=S>,
        T: Add<T, Output=T> + Mul<S, Output=T>
{
    let one: S = One::one();

    x * (one - t) + y * t
}

#[inline]
pub fn quadratic(a: Float, b: Float, c: Float, t0: &mut Float, t1: &mut Float) -> bool {
    // Find quadratic discriminant
    let discrim = b as f64 * b as f64 - 4.0 * a as f64 * c as f64;
    if discrim < 0.0 { return false; }
    let root_discrim = discrim.sqrt();

    // Compute quadratic t values
    let q = if b < 0.0 {
        -0.5 * (b as f64 - root_discrim)
    } else {
        -0.5 * (b as f64 + root_discrim)
    };

    *t0 = (q / a as f64) as Float;
    *t1 = (c as f64 / q) as Float;
    if *t0 > *t1 { std::mem::swap(t0, t1); }

    true
}

#[inline(always)]
pub fn radians(deg: Float) -> Float {
    (PI / 180.0) as Float * deg
}

pub fn clamp<T>(val: T, low: T, high: T) -> T
where T: PartialOrd
{
    if val < low {
        low
    } else if val > high {
        high
    } else {
        val
    }
}

pub fn find_interval<F>(size: i32, pred: F) -> i32
where F: Fn(i32) -> bool
{
    let mut first = 0;
    let mut len = size;

    while len > 0 {
        let half = len >> 1;
        let middle = first + half;

        // Bisect range based on value of pred at middle
        if pred(middle) {
            first = middle + 1;
            len -= half + 1;
        } else {
            len = half;
        }
    }

    clamp(first - 1, 0, size - 2)
}

pub fn gamma(n: isize) -> Float {
    (n as Float * MACHINE_EPSILON) / (1.0 - n as Float * MACHINE_EPSILON)
}

pub fn gamma_correct(value: Float) -> Float {
    if value <= 0.0031308 {
        return 12.92 * value;
    }

    1.055 * value.pow(1.0 / 2.4) - 0.055
}

pub fn inverse_gamma_correct(value: Float) -> Float {
    if value <= 0.04045 { return value * 1.0 / 12.92 }

    ((value + 0.055) * 1.0 / 1.055).pow(2.4)
}



pub fn mod_<T>(a: T, b: T) -> T
where
    T: Copy + Zero + PartialOrd + num::Num
{
    let result = a - (a / b) * b;

    match result < Zero::zero() {
        true => result + b,
        _ => result
    }
}

pub fn is_power_of2<T>(v: T) -> bool
where T: Copy + num::One + num::Zero + PartialOrd + BitAnd<T, Output=T> + Sub<T, Output=T>
{
    (v > T::zero()) && !((v & (v - T::one())) > T::zero())
}

pub fn round_up_pow2_32(mut v: i32) -> i32 {
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;

    v + 1
}

pub fn round_up_pow2_64(mut v: i64) -> i64 {
    v -= 1;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v |= v >> 32;

    v + 1
}

pub fn erf(mut x: Float) -> Float {
    // constants
    let a1 = 0.254829592;
    let a2 = -0.284496736;
    let a3 = 1.421413741;
    let a4 = -1.453152027;
    let a5 = 1.061405429;
    let p = 0.3275911;

    // Save the sign of x
    let sign = if x < 0.0 { -1 } else { 1 };
    x = x.abs();

    // A&S formula 7.1.26
    let t = 1.0 / (1.0 + p * x);
    let y = 1.0 - (((((a5 * t + a4) * t) + a3) * t + a2) * t + a1) * t * (-x * x).exp();

    sign as Float * y
}

pub fn erf_inv(mut x: Float) -> Float {
    let mut p: Float;
    x = clamp(x, -0.99999, 0.99999);
    let mut w = -((1.0 - x) * (1.0 + x)).ln();

    if w < 5.0 {
        w = w - 2.5;
        p = 2.81022636e-08;
        p = 3.43273939e-07 + p * w;
        p = -3.5233877e-06 + p * w;
        p = -4.39150654e-06 + p * w;
        p = 0.00021858087 + p * w;
        p = -0.00125372503 + p * w;
        p = -0.00417768164 + p * w;
        p = 0.246640727 + p * w;
        p = 1.50140941 + p * w;
    } else {
        w = w.sqrt() - 3.0;
        p = -0.000200214257;
        p = 0.000100950558 + p * w;
        p = 0.00134934322 + p * w;
        p = -0.00367342844 + p * w;
        p = 0.00573950773 + p * w;
        p = -0.0076224613 + p * w;
        p = 0.00943887047 + p * w;
        p = 1.00167406 + p * w;
        p = 2.83297682 + p * w;
    }

    p * x
}