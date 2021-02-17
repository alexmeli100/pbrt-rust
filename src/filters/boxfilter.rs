use crate::core::filter::{Filter, Filters};
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::Float;
use crate::core::paramset::ParamSet;
use crate::core::geometry::vector::Vector2f;


pub struct BoxFilter {
    radius: Vector2f
}

impl BoxFilter {
    pub fn new(radius: Vector2f) -> Self {
        return Self{ radius }
    }
}

impl Filter for BoxFilter {
    fn evaluate(&self, _p: &Point2f) -> Float {
        1.0
    }

    fn radius(&self) -> Vector2f {
        self.radius
    }
}

pub fn create_box_filter(ps: &ParamSet) -> Filters {
    let xw = ps.find_one_float("xwidth", 0.5);
    let yw = ps.find_one_float("ywidth", 0.5);
    let b = BoxFilter::new(Vector2f::new(xw, yw));

    b.into()
}