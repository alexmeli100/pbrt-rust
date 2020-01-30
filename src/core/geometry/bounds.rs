use super::point::{Point2, Point3, Point2f, Point2i, Point3f};
use super::vector::{Vector2, Vector3};
use crate::core::pbrt;
use crate::core::pbrt::Float;
use num::{Num, Bounded};
use std::ops::{Index, IndexMut, Mul, Add, DivAssign, Sub};

pub type Bounds2f = Bounds2<Float>;
pub type Bounds2i = Bounds2<isize>;
pub type Bounds3f = Bounds3<Float>;
pub type Bounds3i = Bounds3<isize>;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds2<T>
where T: Num
{
    pub p_min: Point2<T>,
    pub p_max: Point2<T>
}

impl<T> Default for Bounds2<T>
where T: Bounded + Num + Copy
{
    fn default() -> Self {
        let min_num = T::min_value();
        let max_num = T::max_value();

        Self {
            p_min: Point2::new(max_num, max_num),
            p_max: Point2::new(min_num, min_num)
        }
    }
}

impl<T> Bounds2<T>
where T: Num + Copy
{
    pub fn from_point(p: &Point2<T>) -> Self {
        Self {
            p_min: *p,
            p_max: *p
        }
    }

    pub fn from_points(p1: &Point2<T>, p2: &Point2<T>) -> Self
    where T: Ord
    {
        Self {
            p_min: Point2::new(std::cmp::min(p1.x, p2.x), std::cmp::min(p1.y, p2.y)),
            p_max: Point2::new(std::cmp::max(p1.x, p2.x), std::cmp::max(p1.y, p2.y))
        }
    }

    pub fn diagonal(&self) -> Vector2<T> {
        self.p_max - self.p_min
    }

    pub fn area(&self) -> T {
        let d = self.diagonal();

        d.x * d.y
    }

    pub fn maximum_extent(&self) -> i32
    where T: PartialOrd
    {
        let d = self.diagonal();

        match d.x > d.y {
            true => 0,
            _ => 1
        }
    }

    pub fn lerp(&self, t: &Point2f) -> Point2<T>
    where T: Mul<Float, Output=T> + Add<T, Output=T>
    {
        Point2::new(
            pbrt::lerp(t.x, self.p_min.x, self.p_max.x),
            pbrt::lerp(t.y, self.p_min.y, self.p_max.y)
        )
    }

    pub fn offset(&self, p: &Point2<T>) -> Vector2<T>
    where T: DivAssign + PartialOrd
    {
        let mut o = *p - self.p_min;

        if self.p_max.x > self.p_min.x {
            o.x /= self.p_max.x - self.p_min.x
        }

        if self.p_max.y > self.p_min.y {
            o.y /= self.p_max.y - self.p_min.y
        }

        o
    }

    pub fn union_point(&self, p: &Point2<T>) -> Self
    where T: Ord
    {
        Self {
            p_min: self.p_min.min(p),
            p_max: self.p_max.max(p)
        }
    }

    pub fn union_bounds(&self, b: &Self) -> Self
        where T: Ord
    {
        Self {
            p_min: self.p_min.min(&b.p_min),
            p_max: self.p_max.max(&b.p_max)
        }
    }

    pub fn intersect(&self, b: &Self) -> Self
    where T: Ord
    {
        Self {
            p_min: self.p_min.max(&b.p_min),
            p_max: self.p_max.min(&b.p_max)
        }
    }

    pub fn overlaps(&self, b: &Self) -> bool
    where T: PartialOrd
    {
        let x = self.p_max.x >= b.p_min.x && self.p_min.x <= b.p_max.x;
        let y = self.p_max.y >= b.p_min.y && self.p_min.y <= b.p_max.y;

        x && y
    }

    pub fn inside(&self, p: &Point2<T>) -> bool
    where T: PartialOrd
    {
        p.x >= self.p_min.x && p.x <= self.p_max.x &&
        p.y >= self.p_min.y && p.y <= self.p_max.y
    }

    pub fn inside_exclusive(&self, p: &Point2<T>) -> bool
    where T: PartialOrd
    {
        p.x >= self.p_min.x && p.x < self.p_max.x &&
        p.y >= self.p_min.y && p.y < self.p_max.y
    }

    pub fn expand<U>(&self, delta: U) -> Self
    where T: Sub<U, Output=T> + Add<U, Output=T>,
          U: Copy
    {
        Self {
            p_min: self.p_min - Vector2::new(delta, delta),
            p_max: self.p_max + Vector2::new(delta, delta)
        }
    }

}

impl Bounds2f {
    pub fn bounding_sphere(&self) -> (Point2f, Float) {
        let c = (self.p_min + self.p_max) / 2 as Float;
        let rad = match self.inside(&c) {
            true => self.p_max.distance(&c),
            _ => 0 as Float
        };

        (c, rad)
    }
}

impl<T> From<Point2<T>> for Bounds2<T>
where T: Num + Copy
{
    fn from(p: Point2<T>) -> Self {
        Self {
            p_min: p,
            p_max: p
        }
    }
}

impl<T> Index<usize> for Bounds2<T>
where T: Num
{
    type Output = Point2<T>;

    fn index(&self, i: usize) -> &Self::Output {
        match i {
            0 => &self.p_min,
            1 => &self.p_max,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> IndexMut<usize> for Bounds2<T>
    where T: Num
{
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        match i {
            0 => &mut self.p_min,
            1 => &mut self.p_max,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

pub struct Bounds2iIterator {
    p_start: Point2i,
    b: Bounds2i
}

impl Iterator for Bounds2iIterator {
    type Item = Point2i;

    fn next(&mut self) -> Option<Self::Item> {
        self.p_start.x += 1;

        if self.p_start.x == self.b.p_max.x {
            self.p_start.x = self.b.p_min.x;
            self.p_start.y += 1;
        }

        if self.p_start.y == self.b.p_max.y {
            return None;
        } else {
            Some(self.p_start)
        }
    }
}

impl Bounds2i {
    pub fn iter(&self) -> Bounds2iIterator {
        let p_start = self.p_min;

        Bounds2iIterator{
            p_start,
            b: *self
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds3<T>
where T: Num {
    pub p_min: Point3<T>,
    pub p_max: Point3<T>
}

impl<T> Bounds3<T>
    where T: Num + Copy
{
    pub fn from_point(p: &Point3<T>) -> Self {
        Self {
            p_min: *p,
            p_max: *p
        }
    }

    pub fn from_points(p1: Point3<T>, p2: Point3<T>) -> Self
        where T: Ord
    {
        let p_min = Point3::new(std::cmp::min(p1.x, p2.x), std::cmp::min(p1.y, p2.y), std::cmp::min(p1.z, p2.z));
        let p_max = Point3::new(std::cmp::max(p1.x, p2.x), std::cmp::max(p1.y, p2.y), std::cmp::max(p1.z, p2.z));

        Self { p_min, p_max }
    }

    pub fn corner(&self, c: usize) -> Point3<T> {
        assert!(c < 8);

        let x_i = c & 1;
        let y_i = { if c & 2 != 0 {1} else {0} };
        let z_i = { if c & 4 == 0 {1} else {0} };

        Point3::new(self[x_i].x, self[y_i].y, self[z_i].z)
    }

    pub fn diagonal(&self) -> Vector3<T> {
        self.p_max - self.p_min
    }

    pub fn volume(&self) -> T {
        let d = self.diagonal();

        d.x * d.y * d.z
    }

    pub fn maximum_extent(&self) -> i32
        where T: PartialOrd
    {
        let d = self.diagonal();

        if d.x > d.y && d.x > d.z {
            0
        }
        else if d.y > d.z {
            1
        }
        else {
            2
        }
    }

    pub fn lerp(&self, t: &Point3f) -> Point3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T>
    {
        Point3::new(
            pbrt::lerp(t.x, self.p_min.x, self.p_max.x),
            pbrt::lerp(t.y, self.p_min.y, self.p_max.y),
            pbrt::lerp(t.z, self.p_min.z, self.p_max.z)
        )
    }

    pub fn offset(&self, p: &Point3<T>) -> Vector3<T>
        where T: DivAssign + PartialOrd
    {
        let mut o = *p - self.p_min;

        if self.p_max.x > self.p_min.x {
            o.x /= self.p_max.x - self.p_min.x
        }

        if self.p_max.y > self.p_min.y {
            o.y /= self.p_max.y - self.p_min.y
        }

        if self.p_max.z > self.p_min.z {
            o.z /= self.p_max.z - self.p_min.z
        }

        o
    }

    pub fn union_point(&self, p: &Point3<T>) -> Self
        where T: num::Float
    {
        Self {
            p_min: self.p_min.min(p),
            p_max: self.p_max.max(p)
        }
    }

    pub fn union_bounds(&self, b: &Self) -> Self
        where T: num::Float
    {
        Self {
            p_min: self.p_min.min(&b.p_min),
            p_max: self.p_max.max(&b.p_max)
        }
    }

    pub fn intersect(&self, b: &Self) -> Self
        where T: num::Float
    {
        Self {
            p_min: self.p_min.max(&b.p_min),
            p_max: self.p_max.min(&b.p_max)
        }
    }

    pub fn overlaps(&self, b: &Self) -> bool
        where T: PartialOrd
    {
        let x = self.p_max.x >= b.p_min.x && self.p_min.x <= b.p_max.x;
        let y = self.p_max.y >= b.p_min.y && self.p_min.y <= b.p_max.y;
        let z = self.p_max.z >= b.p_min.z && self.p_min.z <= b.p_max.z;

        x && y && z
    }

    pub fn inside(&self, p: &Point3<T>) -> bool
        where T: PartialOrd
    {
        p.x >= self.p_min.x && p.x <= self.p_max.x &&
        p.y >= self.p_min.y && p.y <= self.p_max.y &&
        p.z >= self.p_min.z && p.z <= self.p_max.z
    }

    pub fn inside_exclusive(&self, p: &Point3<T>) -> bool
        where T: PartialOrd
    {
        p.x >= self.p_min.x && p.x < self.p_max.x &&
        p.y >= self.p_min.y && p.y < self.p_max.y &&
        p.z >= self.p_min.z && p.z < self.p_max.z
    }

    pub fn expand<U>(&self, delta: U) -> Self
        where T: Sub<U, Output=T> + Add<U, Output=T>,
              U: Copy
    {
        Self {
            p_min: self.p_min - Vector3::new(delta, delta, delta),
            p_max: self.p_max + Vector3::new(delta, delta, delta)
        }
    }

}

impl<T> Default for Bounds3<T>
    where T: Bounded + Num + Copy
{
    fn default() -> Self {
        let min_num = T::min_value();
        let max_num = T::max_value();

        Self {
            p_min: Point3::new(max_num, max_num, max_num),
            p_max: Point3::new(min_num, min_num, min_num)
        }
    }
}

impl<T> Index<usize> for Bounds3<T>
    where T: Num
{
    type Output = Point3<T>;

    fn index(&self, i: usize) -> &Self::Output {
        match i {
            0 => &self.p_min,
            1 => &self.p_max,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl<T> IndexMut<usize> for Bounds3<T>
    where T: Num
{
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        match i {
            0 => &mut self.p_min,
            1 => &mut self.p_max,
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

impl Bounds3f {
    pub fn surface_area(&self) -> Float
        where
    {
        let d = self.diagonal();

        (d.x * d.y + d.x * d.z + d.y * d.z) * 2.0
    }

    pub fn bounding_sphere(&self) -> (Point3f, Float) {
        let c = (self.p_min + self.p_max) / 2 as Float;
        let rad = match self.inside(&c) {
            true => self.p_max.distance(&c),
            _ => 0 as Float
        };

        (c, rad)
    }
}


