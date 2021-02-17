use std::ops::{Add, Mul, AddAssign, Sub, SubAssign, Neg, MulAssign, Div, DivAssign, Index, IndexMut};
use crate::core::geometry::vector::{Vector3, Vector3f};
use crate::core::pbrt::Float;
use num::{Signed};
use std::fmt::{Display, Formatter};
use std::fmt;

pub type Normal3f = Normal3<Float>;
pub type Normal3i = Normal3<isize>;

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Normal3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Normal3<T> {
    pub fn new(x: T, y: T, z: T) -> Normal3<T> {
        Normal3::<T> {x, y, z}
    }

    pub fn has_nan(&self) -> bool
        where T: num::Float
    {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    pub fn length_squared(&self) -> T
        where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * self.x + self.y * self.y + self.z * self.z
    }

    pub fn length(&self) -> T
        where T: num::Float
    {
        self.length_squared().sqrt()
    }

    pub fn abs(&self) -> Self
        where T: num::Signed
    {
        Normal3::<T> {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs()
        }
    }

    pub fn dot(&self, n: &Self) -> T
        where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * n.x + self.y * n.y + self.z * n.z
    }

    pub fn dot_vec(&self, v: &Vector3<T>) -> T
        where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * v.x + self.y * v.y + self.z * v.z
    }

    pub fn abs_dot_vec(&self, v: &Vector3<T>) -> T
        where T: Copy + Signed
    {
        self.dot_vec(v).abs()
    }

    pub fn abs_dot(&self, n: &Self) -> T
        where T: Copy + Signed
    {
        self.dot(n).abs()
    }

    pub fn permutate(&self, x: usize, y: usize, z: usize) -> Normal3<T>
        where T: Copy
    {
        Normal3::new(self[x], self[y], self[z])
    }

    pub fn min_component(&self) -> T
        where T: num::Float
    {
        self.x.min(self.y.min(self.z))
    }



    pub fn max_component(&self) -> T
        where T: num::Float
    {
        self.x.max(self.y.max(self.z))
    }

    pub fn face_foward(&self, n: &Self) -> Self
        where T: Copy + PartialOrd + Add<T, Output=T> + Mul<T, Output=T> + Neg<Output=T>,
              Float: From<T>
    {
        if Float::from(self.dot(n)) < 0 as Float {
            -(*self)
        } else {
            *self
        }
    }

    pub fn face_foward_vec(&self, v: &Vector3<T>) -> Self
        where T: Copy + PartialOrd + Add<T, Output=T> + Mul<T, Output=T> + Neg<Output=T>,
              Float: From<T>
    {
        if Float::from(self.dot_vec(v)) < 0 as Float {
            -(*self)
        } else {
            *self
        }
    }

}

impl Normal3f {
    pub fn normalize(&self) -> Self {
        *self / self.length()
    }

    pub fn cross_vec(&self, v: &Vector3f) -> Vector3f {
        assert!(!v.has_nan() && !self.has_nan());

        let v1x = self.x as f64;
        let v1y = self.y as f64;
        let v1z = self.z as f64;
        let v2x = v.x as f64;
        let v2y = v.y as f64;
        let v2z = v.z as f64;

        Vector3f::new(
            ((v1y * v2z) - (v1z * v2y)) as Float,
            ((v1z * v2x) - (v1x * v2z)) as Float,
            ((v1x * v2y) - (v1y * v2x)) as Float
        )
    }
}

impl<T> Add for Normal3<T>
    where T: Copy + Add<T, Output=T>
{
    type Output = Normal3<T>;

    fn add(self, rhs: Self) -> Self {
        Normal3::<T> {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z
        }
    }
}

impl<T> AddAssign for Normal3<T>
    where T: AddAssign
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<T> Sub for Normal3<T>
    where T: Copy + Sub<T, Output=T>
{
    type Output = Normal3<T>;

    fn sub(self, rhs: Self) -> Self {
        Normal3::<T> {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z
        }
    }
}

impl<T> SubAssign for Normal3<T>
    where T: SubAssign
{
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl<T> Neg for Normal3<T>
    where T: Neg<Output=T>
{
    type Output = Normal3<T>;

    fn neg(self) -> Self {
        Normal3::<T> {
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Mul for Normal3<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Normal3<T>;

    fn mul(self, rhs: Self) -> Self {
        Normal3::<T> {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z
        }
    }
}

impl<T> MulAssign for Normal3<T>
    where T: MulAssign
{
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
        self.z *= rhs.z;
    }
}

impl<T> Mul<T> for Normal3<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Normal3<T>;

    fn mul(self, rhs: T) -> Normal3<T> {
        Normal3::<T> {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs
        }
    }
}

impl<T> MulAssign<T> for Normal3<T>
    where T: Copy + MulAssign
{
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<Float> for Normal3f {
    type Output = Normal3f;

    fn div(self, rhs: Float) -> Normal3f {
        assert_ne!(0.0 as Float, rhs);
        let d = 1.0 / rhs;

        Normal3f {
            x: self.x * d,
            y: self.y * d,
            z: self.z * d
        }
    }
}

impl DivAssign<Float> for Normal3<Float> {
    fn div_assign(&mut self, rhs: Float) {
        let d = 1.0 / rhs;

        self.x *= d;
        self.y *= d;
        self.z *= d;
    }
}

impl<T> Index<usize> for Normal3<T> {
    type Output = T;

    fn index(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            2 => &self.z,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> IndexMut<usize> for Normal3<T> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> From<Vector3<T>> for Normal3<T> {
    fn from(v: Vector3<T>) -> Self { Normal3::new(v.x, v.y, v.z) }
}

impl<T> Display for Normal3<T>
    where T: Display {
    fn fmt(&self, f: &mut Formatter<'_>) ->fmt::Result {
        write!(f, "[ {}, {}, {} ]", self.x, self.y, self.z)
    }
}

