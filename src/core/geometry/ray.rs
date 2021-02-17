use super::point::Point3f;
use super::vector::Vector3f;
use crate::core::pbrt::Float;
use crate::core::medium::{Mediums};
use std::sync::Arc;
use std::fmt::{Display, Result, Formatter};

#[derive(Clone)]
pub struct Ray {
    pub o       : Point3f,
    pub d       : Vector3f,
    pub t_max   : Float,
    pub time    : Float,
    pub medium  : Option<Arc<Mediums>>,
    pub diff    : Option<RayDifferential>
}

impl Ray {
    pub fn find_point(&self, t: Float) -> Point3f {
        self.o + self.d * t
    }

    pub fn new(o: &Point3f, d: &Vector3f, t_max: Float, time: Float, medium: Option<Arc<Mediums>>, diff: Option<RayDifferential>) -> Self {
        Self {
            o: *o,
            d: *d,
            t_max,
            time,
            medium,
            diff
        }
    }

    pub fn scale_differential(&mut self, s: Float) {
        if let Some(ref mut d) = self.diff {
            d.rx_origin = self.o + (d.rx_origin - self.o) * s;
            d.ry_origin = self.o + (d.ry_origin - self.o) * s;
            d.rx_direction = self.d + (d.rx_direction - self.d) * s;
            d.ry_direction = self.d + (d.ry_direction - self.d) * s;
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
            time: 0 as Float,
            diff: None
        }
    }
}

impl Display for Ray {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f,
            "[ o={}, d={}, tMax={}, time={} ]",
            self.o, self.d, self.t_max, self.time)
    }
}

#[derive(Debug, Default, Copy, Clone)]
pub struct RayDifferential {
    pub rx_origin           : Point3f,
    pub ry_origin           : Point3f,
    pub rx_direction        : Vector3f,
    pub ry_direction        : Vector3f,
    pub has_differentials   : bool
}




