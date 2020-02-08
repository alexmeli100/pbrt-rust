use super::point::Point3f;
use super::vector::Vector3f;
use crate::core::pbrt::Float;
use crate::core::medium::Medium;
use std::sync::Arc;

pub trait  BaseRay {
    fn o(&self) -> Point3f;
    fn d(&self) -> Vector3f;
    fn t_max(&self) -> Float;
    fn time(&self) -> Float;
    fn medium(&self) -> Option<Arc<Medium>>;

    fn find_point(&self, t: Float) -> Point3f {
        self.o() + self.d() * t
    }
}

impl BaseRay for Ray {
    fn o(&self) -> Point3f {
        self.o
    }

    #[inline(always)]
    fn d(&self) -> Vector3f {
        self.d
    }
    
    #[inline(always)]
    fn t_max(&self) -> Float { self.t_max }
    
    #[inline(always)]
    fn time(&self) -> Float { self.time }
    
    #[inline(always)]
    fn medium(&self) -> Option<Arc<Medium>> {
        match &self.medium {
            Some(m) => Some(m.clone()),
            _ => None
        }
    }
}

#[derive(Debug)]
pub struct Ray {
    o: Point3f,
    d: Vector3f,
    t_max: Float,
    time: Float,
    medium: Option<Arc<Medium>>,
}

impl Ray {
    pub fn new(o: &Point3f, d: &Vector3f, t_max: Float, time: Float, medium: Option<Arc<Medium>>) -> Self {
        Self {
            o: *o,
            d: *d,
            t_max,
            time,
            medium
        }
    }
}

impl Default for Ray {
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
pub struct RayDifferential {
    pub r: Ray,
    pub rx_origin: Point3f,
    pub ry_origin: Point3f,
    pub rx_direction: Vector3f,
    pub ry_direction: Vector3f,
    pub has_differentials: bool
}

impl BaseRay for RayDifferential {
    #[inline(always)]
    fn o(&self) -> Point3f {
        self.r.o
    }

    #[inline(always)]
    fn d(&self) -> Vector3f {
        self.r.d
    }
    
    #[inline(always)]
    fn t_max(&self) -> Float { self.r.t_max }
    
    #[inline(always)]
    fn time(&self) -> Float { self.r.time }
    
    #[inline(always)]
    fn medium(&self) -> Option<Arc<Medium>> {
        match &self.r.medium() {
            Some(m) => Some(m.clone()),
            _ => None
        }
    }
}

impl From<Ray> for RayDifferential {
    fn from(r: Ray) -> Self {
        RayDifferential {
            r,
            ..Default::default()
        }
    }
}

impl RayDifferential {
    pub fn scale_differential(&mut self, s: Float) {
        self.rx_origin = self.o() + (self.rx_origin - self.o()) * s;
        self.ry_origin = self.o() + (self.ry_origin - self.o()) * s;
        self.rx_direction = self.d() + (self.rx_direction - self.d()) * s;
        self.ry_direction = self.d() + (self.ry_direction - self.d()) * s;
    }

    pub fn new(o: &Point3f, d: &Vector3f, t_max: Float, time: Float, medium: Option<Arc<Medium>>) -> Self {
        let r = Ray::new(o, d, t_max, time, medium);
        Self::from(r)
    }
}

