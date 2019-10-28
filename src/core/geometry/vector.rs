use num;
use super::point::{Point2, Point3};
use std::ops::{Add, Mul, AddAssign, MulAssign, SubAssign, Sub, Div, DivAssign, Index, IndexMut, Neg};
use num::Signed;

pub type Float = f32;
type Vector3f = Vector3<Float>;

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

impl Div<Float> for Vector2<Float> {
    type Output = Vector2<Float>;

    fn div(self, rhs: Float) -> Vector2<Float> {
        assert_ne!(0.0 as Float, rhs);
        let d = 1.0 as Float / rhs;

        Vector2::<Float> {
            x: self.x * d,
            y: self.y * d
        }
    }
}

impl DivAssign<Float> for Vector2<Float> {
    fn div_assign(&mut self, rhs: Float) {
        let d = 1.0 as Float / rhs;

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

    pub fn abs_dot(&self, v: &Self) -> T
    where T: Copy + Signed
    {
        self.dot(v).abs()
    }

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

impl Div<Float> for Vector3<Float> {
    type Output = Vector3<Float>;

    fn div(self, rhs: Float) -> Vector3<Float> {
        assert_ne!(0.0 as Float, rhs);
        let d = 1.0 as Float / rhs;

        Vector3::<Float> {
            x: self.x * d,
            y: self.y * d,
            z: self.z * d
        }
    }
}

impl DivAssign<Float> for Vector3<Float> {
    fn div_assign(&mut self, rhs: Float) {
        let d = 1.0 as Float / rhs;

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

pub fn vec3_coordinate_system(v1: &Vector3f, v2: &mut Vector3f, v3: &mut Vector3f) {
    if v1.x.abs() > v1.y.abs() {
        *v2 = Vector3f::new(-v1.z, 0.0, v1.x) / (v1.x*v1.x + v1.z*v1.z).sqrt()
    } else {
        *v2 = Vector3f::new(0.0, v1.z, -v1.y) / (v1.y*v1.y + v1.z*v1.z).sqrt()
    }

    *v3 = v1.cross(&*v2);
}