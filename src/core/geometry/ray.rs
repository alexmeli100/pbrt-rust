use super::point::Point3f;
use super::vector::Vector3f;
use crate::core::pbrt::Float;
use crate::core::medium::Medium;

pub trait  BaseRay {
    fn o(&self) -> Point3f;
    fn d(&self) -> Vector3f;
    fn t_max(&self) -> Float;
    fn time(&self) -> Float;
    fn medium(&self) -> Option<&Medium>;

    fn find_point(&self, t: Float) -> Point3f {
        self.o() + self.d() * t
    }
}

impl BaseRay for Ray<'_> {
    fn o(&self) -> Point3f {
        self.o
    }

    fn d(&self) -> Vector3f {
        self.d
    }
    fn t_max(&self) -> Float { self.t_max }
    fn time(&self) -> Float { self.time }
    fn medium(&self) -> Option<&Medium> { self.medium }
}

#[derive(Debug)]
pub struct Ray<'a> {
    o: Point3f,
    d: Vector3f,
    t_max: Float,
    time: Float,
    medium: Option<&'a Medium>,
}

impl<'a> Ray<'a> {
    pub fn new(o: &Point3f, d: &Vector3f, t_max: Float, time: Float, medium: Option<&'a Medium>) -> Self {
        Self {
            o: *o,
            d: *d,
            t_max,
            time,
            medium
        }
    }
}

impl<'a> Default for Ray<'a> {
    fn default() -> Self {
        Ray {
            t_max: std::f32::INFINITY,
            medium: None,
            d: Vector3f::default(),
            o: Point3f::default(),
            time: 0 as Float
        }
    }
}

#[derive(Debug, Default)]
pub struct RayDifferential<'a> {
    r: Ray<'a>,
    rx_origin: Point3f,
    ry_origin: Point3f,
    rx_direction: Vector3f,
    ry_direction: Vector3f,
    has_differentials: bool
}

impl BaseRay for RayDifferential<'_> {
    fn o(&self) -> Point3f {
        self.r.o
    }

    fn d(&self) -> Vector3f {
        self.r.d
    }
    fn t_max(&self) -> Float { self.r.t_max }
    fn time(&self) -> Float { self.r.time }
    fn medium(&self) -> Option<&Medium> { self.r.medium }
}

impl<'a> From<Ray<'a>> for RayDifferential<'a> {
    fn from(r: Ray<'a>) -> Self {
        RayDifferential {
            r,
            ..Default::default()
        }
    }
}

impl<'a> RayDifferential<'a> {
    fn scale_differential(&mut self, s: Float) {
        self.rx_origin = self.o() + (self.rx_origin - self.o()) * s;
        self.ry_origin = self.o() + (self.ry_origin - self.o()) * s;
        self.rx_direction = self.d() + (self.rx_direction - self.d()) * s;
        self.ry_direction = self.d() + (self.ry_direction - self.d()) * s;
    }

    fn new(o: &Point3f, d: &Vector3f, t_max: Float, time: Float, medium: Option<&'a Medium>) -> Self {
        let r = Ray::new(o, d, t_max, time, medium);
        Self::from(r)
    }
}

