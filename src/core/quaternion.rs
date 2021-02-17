use crate::core::geometry::vector::Vector3f;
use crate::core::pbrt::{Float, clamp};
use super::transform::Transform;
use std::ops::{Add, AddAssign, Sub, SubAssign, Mul, MulAssign, Neg, Div, DivAssign};

#[derive(Debug, Default, Copy, Clone)]
pub struct Quaternion {
    pub v: Vector3f,
    pub w: Float
}

impl Quaternion {
    pub fn new(v: &Vector3f) -> Quaternion {
        Self {
            v: *v,
            w: 1.0,
        }
    }

    pub fn dot(&self, q1: &Self) -> Float {
        self.v.dot(&q1.v) + self.w * q1.w
    }

    pub fn normalize(&self) -> Self {
        *self / self.dot(self).sqrt()
    }
}

impl Add for Quaternion {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        Self {
            v: self.v + other.v,
            w: self.w + other.w
        }
    }
}

impl AddAssign for Quaternion {
    fn add_assign(&mut self, other: Self) {
        self.v += other.v;
        self.w += other.w;
    }
}

impl Sub for Quaternion {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        Self {
            v: self.v - other.v,
            w: self.w - other.w
        }
    }
}

impl SubAssign for Quaternion {
    fn sub_assign(&mut self, other: Self) {
        self.v -= other.v;
        self.w -= other.w;
    }
}

impl Neg for Quaternion {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self {
            v: -self.v,
            w: -self.w
        }
    }
}

impl Mul<Float> for Quaternion {
    type Output = Self;

    fn mul(self, f: Float) -> Self::Output {
        Self {
            v: self.v * f,
            w: self.w * f
        }
    }
}

impl MulAssign<Float> for Quaternion {
    fn mul_assign(&mut self, f: Float) {
        self.v *= f;
        self.w *= f;
    }
}

impl Div<Float> for Quaternion {
    type Output = Self;

    fn div(self, f: Float) -> Self::Output {
        Self {
            v: self.v / f,
            w: self.w / f
        }
    }
}

impl DivAssign<Float> for Quaternion {
    fn div_assign(&mut self, f: Float) {
        self.v /= f;
        self.w /= f;
    }
}

impl From<Transform> for Quaternion {
    fn from(t: Transform) -> Self {
        let m= t.m;
        let trace: Float = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];

        if trace > 0.0 {

            let mut s: Float = (trace + 1.0).sqrt();
            let w: Float = s / 2.0;
            s = 0.5 / s;

            Self {
                v: Vector3f::new(
                    (m[(2, 1)] - m[(1, 2)]) * s,
                    (m[(0, 2)] - m[(2, 0)]) * s,
                    (m[(1, 0)] - m[(0, 1)]) * s),
                w,
            }
        } else {
            // compute largest of $x$, $y$, or $z$, then remaining components
            let nxt: [usize; 3] = [1, 2, 0];
            let mut q: [Float; 3] = [0.0; 3];
            let mut i = if m[(1, 1)] > m[(0, 0)] { 1 } else { 0 };
            if m[(2, 2)] > m[(i, i)] {
                i = 2;
            }
            let j = nxt[i];
            let k = nxt[j];
            let mut s: Float = ((m[(i, i)] - (m[(j ,j)] + m[(k, k)])) + 1.0).sqrt();
            q[i] = s * 0.5;
            if s != 0.0 {
                s = 0.5 / s;
            }
            let w: Float = (m[(k, j)] - m[(j, k)]) * s;
            q[j] = (m[(j, i)] + m[(i, j)]) * s;
            q[k] = (m[(k, i)] + m[(i, k)]) * s;

            Self { v: Vector3f::new(q[0], q[1], q[2]), w, }
        }
    }
}

pub fn slerp(q1: &Quaternion, q2: &Quaternion, t: Float) -> Quaternion {
    let cos_theta = q1.dot(q2);

    if cos_theta > 0.9995 {
        (*q1 * (1.0 - t) + *q2 * t).normalize()
    } else {
        let theta = clamp(cos_theta, -1.0, 1.0).acos();
        let thetap = theta * t;
        let qperp = (*q2 - *q1 * cos_theta).normalize();

        *q1 * thetap.cos() + qperp * thetap.sin()
    }
}