use num;
use crate::core::pbrt::*;
use super::vector::{Vector3, Vector2};
use std::ops::{Add, Mul, AddAssign, MulAssign, SubAssign, Sub, Div, DivAssign, Index, IndexMut, Neg};


pub type Point2f = Point2<Float>;
pub type Point2i = Point2<isize>;
pub type Point3f = Point3<Float>;
pub type Point3i = Point3<isize>;

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Point2<T> {
    pub x: T,
    pub y: T
}

impl<T> Point2<T> {
    pub fn new(x: T, y: T) -> Point2<T> {
        Point2::<T> {x, y}
    }

    pub fn has_nan(&self) -> bool
        where T: num::Float
    {
        self.x.is_nan() || self.y.is_nan()
    }

    pub fn max(&self, p2: &Self) -> Self
        where T: Ord + Copy
    {
        Point2::new(
            std::cmp::max(self.x, p2.x),
            std::cmp::max(self.y, p2.y),
        )
    }

    pub fn min(&self, p2: &Self) -> Self
        where T: Ord + Copy
    {
        Point2::new(
            std::cmp::min(self.x, p2.x),
            std::cmp::min(self.y, p2.y),
        )
    }

    fn length_squared(&self) -> T
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
        Point2::new(self.x.abs(), self.y.abs())
    }

    pub fn distance(&self, p2: &Point2<T>) -> T
        where T: num::Float
    {
        (*self - *p2).length()
    }

    pub fn distance_squared(&self, p2: &Point2<T>) -> T
        where T: num::Float
    {
        (*self - *p2).length_squared()
    }
}

impl Point2f {
    pub fn lerp(&self, t: Float, p: &Self) -> Self {
        lerp(t, *self, *p)
    }
}

impl<T> Add for Point2<T>
    where T: Copy + Add<T, Output=T>
{
    type Output = Point2<T>;

    fn add(self, rhs: Self) -> Self {
        Point2::<T> {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<T, U> Add<Vector2<U>> for Point2<T>
where T: Copy + Add<U, Output=T> {
    type Output = Point2<T>;

    fn add(self, rhs: Vector2<U>) -> Point2<T> {
        Point2::new(self.x+rhs.x, self.y+rhs.y)
    }
}

impl<T> AddAssign for Point2<T>
where T: AddAssign
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y
    }
}

impl<T> AddAssign<Vector2<T>> for Point2<T>
where T: AddAssign
{
    fn add_assign(&mut self, rhs: Vector2<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
    }
}

impl<T> Sub for Point2<T>
where T: Copy + Sub<T, Output=T>
{
    type Output = Vector2<T>;

    fn sub(self, rhs: Self) -> Vector2<T> {
        Vector2::new(self.x-rhs.x, self.y-rhs.y)
    }
}

impl<T, U> Sub<Vector2<U>> for Point2<T>
where T: Copy + Sub<U, Output=T>
{
    type Output = Point2<T>;

    fn sub(self, rhs: Vector2<U>) -> Point2<T> {
        Point2::new(self.x-rhs.x, self.y-rhs.y)
    }
}

impl<T> SubAssign<Vector2<T>> for Point2<T>
    where T: SubAssign
{
    fn sub_assign(&mut self, rhs: Vector2<T>) {
        self.x -= rhs.x;
        self.y -= rhs.y
    }
}

impl<T> Neg for Point2<T>
    where T: Neg<Output=T>
{
    type Output = Point2<T>;

    fn neg(self) -> Self {
        Point2::new(-self.x, -self.y)
    }
}

impl<T> Mul for Point2<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Point2<T>;

    fn mul(self, rhs: Self) -> Self {
        Point2::<T> {
            x: self.x * rhs.x,
            y: self.y * rhs.y
        }
    }
}

impl<T> MulAssign for Point2<T>
    where T: MulAssign
{
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
    }
}

impl<T> Mul<T> for Point2<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Point2<T>;

    fn mul(self, rhs: T) -> Point2<T> {
        Point2::<T> {
            x: self.x * rhs,
            y: self.y * rhs,
        }
    }
}

impl Div<Float> for Point2<Float> {
    type Output = Point2<Float>;

    fn div(self, rhs: Float) -> Point2<Float> {
        assert_ne!(0.0 as Float, rhs);
        let d = 1.0 as Float / rhs;

        Point2::<Float> {
            x: self.x * d,
            y: self.y * d
        }
    }
}

impl DivAssign<Float> for Point2<Float> {
    fn div_assign(&mut self, rhs: Float) {
        let d = 1.0 as Float / rhs;

        self.x *= d;
        self.y *= d;
    }
}

impl<T> Index<usize> for Point2<T> {
    type Output = T;

    fn index(&self, i: usize) -> &T {
        match i {
            0 => &self.x,
            1 => &self.y,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> IndexMut<usize> for Point2<T> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> From<Point3<T>> for Point2<T> {
    fn from(p: Point3<T>) -> Self {
        Point2::new(p.x, p.y)
    }
}

impl<T, U> From<Vector2<U>> for Point2<T>
where T: From<U>
{
    fn from(v: Vector2<U>) -> Self {
        Point2::new(T::from(v.x), T::from(v.y))
    }
}

#[derive(Debug, Default, Copy, Clone, PartialEq)]
pub struct Point3<T> {
    pub x: T,
    pub y: T,
    pub z: T
}

impl<T> Point3<T> {
    pub fn new(x: T, y: T, z: T) -> Point3<T> {
        Point3::<T> {x, y, z}
    }

    pub fn max(&self, p2: &Self) -> Self
        where T: Ord + Copy
    {
        Point3::new(
            std::cmp::max(self.x, p2.x),
            std::cmp::max(self.y, p2.y),
            std::cmp::max(self.z, p2.z)
        )
    }

    pub fn min(&self, p2: &Self) -> Self
        where T: Ord + Copy
    {
        Point3::new(
            std::cmp::min(self.x, p2.x),
            std::cmp::min(self.y, p2.y),
            std::cmp::min(self.z, p2.z)
        )
    }

    pub fn has_nan(&self) -> bool
        where T: num::Float
    {
        self.x.is_nan() || self.y.is_nan() || self.z.is_nan()
    }

    fn length_squared(&self) -> T
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
        Point3::<T> {
            x: self.x.abs(),
            y: self.y.abs(),
            z: self.z.abs()
        }
    }

    pub fn distance(&self, p2: &Point3<T>) -> T
        where T: num::Float
    {
        (*self - *p2).length()
    }

    pub fn distance_squared(&self, p2: &Point3<T>) -> T
        where T: num::Float
    {
        (*self - *p2).length_squared()
    }

    pub fn floor(&self) -> Self
        where T: num::Float
    {
        Point3::new(self.x.floor(), self.y.floor(), self.z.floor())
    }

    pub fn ceil(&self) -> Self
        where T: num::Float
    {
        Point3::new(self.x.ceil(), self.y.ceil(), self.z.ceil())

    }

    pub fn permutate(&self, x: usize, y: usize, z: usize) -> Self
        where T: Copy
    {
        Point3::new(self[x], self[y], self[z])
    }

}

impl Point3f {
    pub fn lerp(&self, t: Float, p: &Self) -> Self {
        lerp(t, *self, *p)
    }
}

impl<T> Add for Point3<T>
    where T: Copy + Add<T, Output=T>
{
    type Output = Point3<T>;

    fn add(self, rhs: Self) -> Self {
        Point3::<T> {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + self.z
        }
    }
}

impl<T, U> Add<Vector3<U>> for Point3<T>
    where T: Copy + Add<U, Output=T> {
    type Output = Point3<T>;

    fn add(self, rhs: Vector3<U>) -> Self {
        Point3::new(self.x+rhs.x, self.y+rhs.y, self.z+rhs.z)
    }
}

impl<T> AddAssign for Point3<T>
    where T: AddAssign
{
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl<T> AddAssign<Vector3<T>> for Point3<T>
    where T: AddAssign
{
    fn add_assign(&mut self, rhs: Vector3<T>) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}


impl<T> Sub for Point3<T>
    where T: Copy + Sub<T, Output=T>
{
    type Output = Vector3<T>;

    fn sub(self, rhs: Self) -> Vector3<T> {
        Vector3::<T> {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z
        }
    }
}

impl<T, U> Sub<Vector3<U>> for Point3<T>
    where T: Copy + Sub<U, Output=T>
{
    type Output = Point3<T>;

    fn sub(self, rhs: Vector3<U>) -> Point3<T> {
        Point3::new(self.x-rhs.x, self.y-rhs.y, self.z-rhs.z)
    }
}

impl<T> SubAssign<Vector3<T>> for Point3<T>
    where T: SubAssign
{
    fn sub_assign(&mut self, rhs: Vector3<T>) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl<T> Neg for Point3<T>
    where T: Neg<Output=T>
{
    type Output = Point3<T>;

    fn neg(self) -> Self {
        Point3::<T> {
            x: -self.x,
            y: -self.y,
            z: -self.z
        }
    }
}

impl<T> Mul for Point3<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Point3<T>;

    fn mul(self, rhs: Self) -> Self {
        Point3::<T> {
            x: self.x * rhs.x,
            y: self.y * rhs.y,
            z: self.z * rhs.z
        }
    }
}

impl<T> MulAssign for Point3<T>
    where T: MulAssign
{
    fn mul_assign(&mut self, rhs: Self) {
        self.x *= rhs.x;
        self.y *= rhs.y;
        self.z *= rhs.z;
    }
}

impl<T> Mul<T> for Point3<T>
    where T: Copy + Mul<T, Output=T>
{
    type Output = Point3<T>;

    fn mul(self, rhs: T) -> Point3<T> {
        Point3::<T> {
            x: self.x * rhs,
            y: self.y * rhs,
            z: self.z * rhs
        }
    }
}

impl<T> MulAssign<T> for Point3<T>
    where T: Copy + MulAssign
{
    fn mul_assign(&mut self, rhs: T) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<Float> for Point3<Float> {
    type Output = Point3<Float>;

    fn div(self, rhs: Float) -> Point3<Float> {
        assert_ne!(0.0 as Float, rhs);
        let d = 1.0 as Float / rhs;

        Point3::<Float> {
            x: self.x * d,
            y: self.y * d,
            z: self.z * d
        }
    }
}

impl DivAssign<Float> for Point3<Float> {
    fn div_assign(&mut self, rhs: Float) {
        let d = 1.0 as Float / rhs;

        self.x *= d;
        self.y *= d;
        self.z *= d;
    }
}

impl<T> Index<usize> for Point3<T> {
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

impl<T> IndexMut<usize> for Point3<T> {
    fn index_mut(&mut self, i: usize) -> &mut T {
        match i {
            0 => &mut self.x,
            1 => &mut self.y,
            2 => &mut self.z,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}



impl<T, U> From<Vector3<U>> for Point3<T>
    where T: From<U>
{
    fn from(v: Vector3<U>) -> Self {
        Point3::new(T::from(v.x), T::from(v.y), T::from(v.z))
    }
}
