use std::ops::{Sub, Add, Mul};

use crate::core::efloat::EFloat;
use num::One;

pub type Float = f32;

pub const PI: Float = 3.14159265358979323846;
pub const INFINITY: f32 = std::f32::INFINITY;
pub const SHADOW_EPSILON: f32 = 0.0001;
pub const MACHINE_EPSILON: f32 = std::f32::EPSILON * 0.5;

#[inline(always)]
fn float_to_bits(f: f32) -> u32 {
    let ui: u32;

    unsafe {
        let res: u32 = std::mem::transmute_copy(&f);
        ui = res;
    }

    ui
}

#[inline(always)]
fn bits_to_float(ui: u32) -> f32 {
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

pub fn lerp<T, S>(t: S, x: T, y: T) -> T
    where
        S: Copy + num::One + Sub<S, Output=S>,
        T: Add<T, Output=T> + Mul<S, Output=T>
{
    let one: S = One::one();

    x * (one - t) + y * t
}

#[inline(always)]
pub fn radians(deg: Float) -> Float {
    (std::f32::consts::PI / 180.0) as Float * deg
}

pub fn clamp<T>(val: T, high: T, low: T) -> T
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

pub fn gamma(n: isize) -> Float {
    (n as Float * MACHINE_EPSILON) / (1.0 - n as Float * MACHINE_EPSILON)
}

pub fn quadratic(a: EFloat, b: EFloat, c: EFloat, t0: &mut EFloat, t1: &mut EFloat) -> bool {
    unimplemented!();
}