use std::ops::{Sub, Add, Mul};

pub type Float = f32;


pub fn lerp<T, S>(t: S, x: T, y: T) -> T
    where
        S: Copy + num::One + Sub<S, Output=S>,
        T: Add<T, Output=T> + Mul<S, Output=T>
{
    let one: S = num::One::one();

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