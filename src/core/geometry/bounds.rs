use super::point::{Point2, Point3, Point2f, Point2i};
use super::vector::{Vector2, Vector3};
use crate::core::pbrt;
use crate::core::pbrt::Float;
use num::{Num, Bounded};
use std::ops::{Index, IndexMut, Mul, Add, DivAssign};

pub type Bounds2f = Bounds2<Float>;
pub type Bounds2i = Bounds2<isize>;
//pub type Bounds3f = Bounds3<Float>;
//pub type Bounds3i = Bounds3<isize>;

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
    pub fn from_single(p: &Point2<T>) -> Self {
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
            false => 1
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

//        let mut p_end = Point2i::new(self.p_min.x, self.p_max.y);
//
//        if self.p_min.x >= self.p_max.x || self.p_min.y >= self.p_max.y {
//            p_end = self.p_min;
//        }

        Bounds2iIterator{
            p_start,
            b: *self
        }
    }
}