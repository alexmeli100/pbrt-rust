use crate::core::pbrt::{Float, next_float_down, next_float_up, INFINITY};
use std::ops::{Add, Sub, Mul, Div, Neg};

#[derive(Debug, Default, Copy, Clone)]
pub struct EFloat {
    pub v   : Float,
    pub low : Float,
    pub high: Float
}

impl EFloat {
    pub fn new(v: Float, err: Float) -> Self {
        let low;
        let high;

        if err == 0.0 {
            low = v;
            high = v;
        } else {
            low = next_float_down(v - err);
            high = next_float_up(v + err);
        }

        Self {v, low, high }
    }

    pub fn lower_bound(&self) -> Float {
        self.low
    }

    pub fn upper_bound(&self) -> Float {
        self.high
    }

    pub fn get_absolute_error(&self) -> Float { self.high - self.low }

    pub fn sqrt(&self) -> Self {
        let mut r = EFloat::default();

        r.v = self.v.sqrt();

        r.low = next_float_down(self.low.sqrt());
        r.high = next_float_up(self.high.sqrt());

        r
    }

    pub fn abs(&self) -> Self {
        if self.low >= 0.0 {
            *self
        } else if self.high <= 0.0 {
            let mut r = EFloat::default();
            r.v = -self.v;

            r.low = -self.high;
            r.high = -self.low;

            r
        } else {
            let mut r = EFloat::default();
            r.v = self.v.abs();

            r.low = 0.0;
            r.high = (-self.low).max(self.high);

            r
        }
    }
}

impl From<Float> for EFloat {
    fn from(f: Float) -> Self {
        EFloat::new(f, 0.0)
    }
}

impl From<EFloat> for Float {
    fn from(ef: EFloat) -> Self {
        ef.v
    }
}

impl Add for EFloat {
    type Output = Self;

    fn add(self, other: Self) -> Self::Output {
        let mut r = EFloat::default();

        r.v = self.v + other.v;
        r.low = next_float_down(self.lower_bound()+ other.lower_bound());
        r.high = next_float_up(self.upper_bound() + other.upper_bound());

        r

    }
}

impl Sub for EFloat {
    type Output = Self;

    fn sub(self, other: Self) -> Self::Output {
        let mut r = EFloat::default();

        r.v = self.v - other.v;
        r.low = next_float_down(self.lower_bound()- other.lower_bound());
        r.high = next_float_up(self.upper_bound() - other.upper_bound());

        r
    }
}

impl Mul for EFloat {
    type Output =Self;

    fn mul(self, other: Self) -> Self::Output {
        let mut r = EFloat::default();
        r.v = self.v * other.v;

        let prod = [
            self.lower_bound() * other.lower_bound(),
            self.upper_bound() * other.lower_bound(),
            self.lower_bound() * other.upper_bound(),
            self.upper_bound() * other.upper_bound()
        ];

        r.low = next_float_down((prod[0].min(prod[1])).min(prod[2].min(prod[3])));
        r.high = next_float_up((prod[0].max(prod[1])).max(prod[2].max(prod[3])));

        r
    }
}

impl Div for EFloat {
    type Output = Self;

    fn div(self, other: Self) -> Self::Output {
        let mut r = EFloat::default();
        r.v = self.v / other.v;

        if self.low < 0.0 && self.high > 0.0 {
            r.low = -INFINITY;
            r.high = INFINITY;
        } else {
            let div = [
                self.lower_bound() / other.lower_bound(),
                self.upper_bound() / other.lower_bound(),
                self.lower_bound() / other.upper_bound(),
                self.upper_bound() / other.upper_bound()
            ];

            r.low = next_float_down((div[0].min(div[1])).min(div[2].min(div[3])));
            r.high = next_float_up((div[0].max(div[1])).max(div[2].max(div[3])));
        }

        r
    }
}

impl Neg for EFloat {
    type Output = Self;

    fn neg(self) -> Self::Output {
        let mut r = EFloat::default();
        r.v = -self.v;

        r.high = -self.high;
        r.low = -self.low;

        r
    }
}

impl Add<Float> for EFloat {
    type Output = Self;

    fn add(self, f: Float) -> Self::Output {
        EFloat::from(f) + self
    }
}

impl Sub<Float> for EFloat {
    type Output = Self;

    fn sub(self, f: Float) -> Self::Output {
        EFloat::from(f) - self
    }
}

impl Div<Float> for EFloat {
    type Output =Self;

    fn div(self, f: Float) -> Self::Output {
        EFloat::from(f) / self
    }
}

impl Mul<Float> for EFloat {
    type Output = Self;

    fn mul(self, f: Float) -> Self::Output {
        EFloat::from(f) * self
    }
}

impl PartialEq for EFloat {
    fn eq(&self, other: &Self) -> bool {
        self.v == other.v
    }
}