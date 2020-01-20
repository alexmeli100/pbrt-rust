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