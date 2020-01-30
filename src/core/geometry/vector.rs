use num;
use super::point::{Point2, Point3};
use super::normal::Normal3;
use crate::core::pbrt::{Float};
use std::ops::{Add, Mul, AddAssign, MulAssign, SubAssign, Sub, Div, DivAssign, Index, IndexMut, Neg};
use num::{Signed, Zero, One};

pub type Vector2i = Vector2<isize>;
pub type Vector2f = Vector2<Float>;
pub type Vector3i = Vector3<isize>;
pub type Vector3f = Vector3<Float>;

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Vector2<T> {
    pub x: T,
    pub y: T
}

impl<T> Vector2<T> {
    pub fn new(x: T, y: T) -> Vector2<T> {
        Vector2::<T> {x, y}
    }

    pub fn has_nan(&self) -> bool
    where T: num::Float
    {
        self.x.is_nan() || self.y.is_nan()
    }

    pub fn length_squared(&self) -> T
    where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * self.x + self.y * self.y
    }

    pub fn length(&self) -> T
    where T: num::Float
    {
        self.length_squared().sqrt()
    }

    pub fn abs(&self) -> Self
    where T: num::Float
    {
        Vector2::new(self.x.abs(), self.y.abs())
    }

    pub fn dot(&self, v: &Self) -> T
        where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * v.y + self.y * v.y
    }

    pub fn abs_dot(&self, v: &Self) -> T
        where T: Copy + Signed
    {
        self.dot(v).abs()
    }

    pub fn min_component(&self) -> T
        where T: num::Float
    {
        self.x.min(self.y)
    }

    pub fn max_component(&self) -> T
        where T: num::Float
    {
        self.x.max(self.y)
    }
}

impl Vector2<Float> {
    pub fn normalize(&self) -> Self {
        *self / self.length()
    }
}

impl<T> Add for Vector2<T>
where T: Copy + Add<T, Output=T>
{
    type Output = Vector2<T>;

    fn add(self, rhs: Self) -> Self {
        Vector2::<T> {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T> AddAssign for Vector2<T>
where T: AddAssign
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y
    }
}

impl<T> Sub for Vector2<T>
where T: Copy + Sub<T, Output=T>
{
    type Output = Vector2<T>;

    fn sub(self, rhs: Self) -> Self {
        Vector2::<T> {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl<T> SubAssign for Vector2<T>
where T: SubAssign
{
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y
    }
}

impl<T> Neg for Vector2<T>
where T: Neg<Output=T>
{
    type Output = Vector2<T>;

    fn neg(self) -> Self {
        Vector2::new(-self.x, -self.y)
    }
}

impl<T> Mul for Vector2<T>
where T: Copy + Mul<T, Output=T>
{
    type Output = Vector2<T>;

    fn mul(self, rhs: Self) -> Self {
        Vector2::<T> {
            x: self.x * rhs.x,
            y: self.y * rhs.y
        }
    }
}

impl<T> MulAssign for Vector2<T>
where T: MulAssign
{
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl<T> Mul<T> for Vector2<T>
where T: Copy + Mul<T, Output=T>
{
    type Output = Vector2<T>;

    fn mul(self, rhs: T) -> Vector2<T> {
        Vector2::<T> {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl<T> Div<T> for Vector2<T>
    where T: Copy + Div<T, Output=T> + Zero + One
{
    type Output = Vector2<T>;

    fn div(self, rhs: T) -> Vector2<T> {
        assert_ne!(T::zero(), rhs);
        let d = T::one() / rhs;

        Vector2::<T> {
            x: self.x * d,
            y: self.y * d
        }
    }
}

impl<T> DivAssign<T> for Vector2<T>
    where T: One + MulAssign + Div<T, Output=T>
{
    fn div_assign(&mut self, rhs: T) {
        let d = T::one() / rhs;

        self.x *= d;
        self.y *= d;
    }
}

impl<T> Index<usize> for Vector2<T> {
    type Output = T;

    fn index(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> IndexMut<usize> for Vector2<T> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> From<Point2<T>> for Vector2<T> {
   fn from(p: Point2<T>) -> Self {
       Vector2::new(p.x, p.y)
   }
}

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Vector3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Vector3<T> {
    pub fn new(x: T, y: T, z: T) -> Vector3<T> {
        Vector3::<T> {x, y, z}
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
    where T: num::Float
    {
        Vector3::<T> {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs()
        }
    }

    pub fn dot(&self, v: &Self) -> T
    where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
    {
        self.x * v.y + self.y * v.y + self.z * v.z
    }

    pub fn dot_norm(&self, n: &Normal3<T>) -> T
    where T: Copy + Add<T, Output=T> + Mul<T, Output=T> { self.x * n.x + self.y * n.y + self.z * n.z }

    pub fn abs_dot(&self, v: &Self) -> T
    where T: Copy + Signed
    {
        self.dot(v).abs()
    }

    pub fn abs_dot_norm(&self, n: &Normal3<T>) -> T
    where T: Copy + Signed { self.dot_norm(n).abs() }

    pub fn permutate(&self, x: usize, y: usize, z: usize) -> Vector3<T>
        where T: Copy
    {
        Vector3::new(self[x], self[y], self[z])
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

    pub fn face_foward(&self, v: &Self) -> Self
        where T: Copy + PartialOrd + Add<T, Output=T> + Mul<T, Output=T> + Neg<Output=T>,
              Float: From<T>
    {
        if Float::from(self.dot(v)) < 0 as Float {
            -(*self)
        } else {
            *self
        }
    }

    pub fn face_foward_vec(&self, n: &Normal3<T>) -> Self
        where T: Copy + PartialOrd + Add<T, Output=T> + Mul<T, Output=T> + Neg<Output=T>,
              Float: From<T>
    {
        if Float::from(self.dot_norm(n)) < 0 as Float {
            -(*self)
        } else {
            *self
        }
    }
}

impl Vector3f {
    pub fn normalize(&self) -> Self {
        *self / self.length()
    }

    pub fn cross(&self, v2: &Vector3f) -> Vector3f {
        let v1x = self.x as f64;
        let v1y = self.y as f64;
        let v1z = self.z as f64;
        let v2x = v2.x as f64;
        let v2y = v2.y as f64;
        let v2z = v2.z as f64;

        Vector3f {
            x: ((v1y * v2z) - (v1z * v2y)) as Float,
            y: ((v1z * v2x) - (v1x * v2z)) as Float,
            z: ((v1x * v2y) - (v1y * v2x)) as Float
        }
    }
}

impl<T> Add for Vector3<T>
where T: Copy + Add<T, Output=T>
{
    type Output = Vector3<T>;

    fn add(self, rhs: Self) -> Self {
        Vector3::<T> {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + self.z
        }
    }
}

impl<T> AddAssign for Vector3<T>
where T: AddAssign
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<T> Sub for Vector3<T>
where T: Copy + Sub<T, Output=T>
{
    type Output = Vector3<T>;

    fn sub(self, rhs: Self) -> Self {
        Vector3::<T> {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z
        }
    }
}

impl<T> SubAssign for Vector3<T>
where T: SubAssign
{
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl<T> Neg for Vector3<T>
where T: Neg<Output=T>
{
    type Output = Vector3<T>;

    fn neg(self) -> Self {
        Vector3::<T> {
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Mul for Vector3<T>
where T: Copy + Mul<T, Output=T>
{
    type Output = Vector3<T>;

    fn mul(self, rhs: Self) -> Self {
        Vector3::<T> {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z
        }
    }
}

impl<T> MulAssign for Vector3<T>
where T: MulAssign
{
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
        self.z *= rhs.z;
    }
}

impl<T> Mul<T> for Vector3<T>
where T: Copy + Mul<T, Output=T>
{
    type Output = Vector3<T>;

    fn mul(self, rhs: T) -> Vector3<T> {
        Vector3::<T> {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs
        }
    }
}

impl<T> MulAssign<T> for Vector3<T>
    where T: Copy + MulAssign
{
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl<T> Div<T> for Vector3<T>
    where T: Copy + Div<T, Output=T> + Zero + One
{
    type Output = Vector3<T>;

    fn div(self, rhs: T) -> Vector3<T> {
        assert_ne!(T::zero(), rhs);
        let d = T::one() / rhs;

        Vector3::<T> {
            x: self.x * d,
            y: self.y * d,
            z: self.z * d
        }
    }
}

impl<T> DivAssign<T> for Vector3<T>
    where T: One + MulAssign + Div<T, Output=T>
{
    fn div_assign(&mut self, rhs: T) {
        let d = T::one() / rhs;

        self.x *= d;
        self.y *= d;
        self.z *= d;
    }
}

impl<T> Index<usize> for Vector3<T> {
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

impl<T> IndexMut<usize> for Vector3<T> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> From<Point3<T>> for Vector3<T> {
    fn from(p: Point3<T>) -> Self {
        Vector3::new(p.x, p.y, p.z)
    }
}

pub fn vec3_coordinate_system(v1: &Vector3f, v2: &mut Vector3f, v3: &mut Vector3f) {
    if v1.x.abs() > v1.y.abs() {
        *v2 = Vector3f::new(-v1.z, 0.0, v1.x) / (v1.x*v1.x + v1.z*v1.z).sqrt()
    } else {
        *v2 = Vector3f::new(0.0, v1.z, -v1.y) / (v1.y*v1.y + v1.z*v1.z).sqrt()
    }


    *v3 = v1.cross(&*v2);
}