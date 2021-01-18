use crate::core::geometry::vector::Vector2f;
use crate::core::filter::{Filter, Filters};
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::Float;
use crate::core::paramset::ParamSet;

pub struct TriangleFilter {
    radius: Vector2f
}

impl TriangleFilter {
    fn new(radius: Vector2f) -> Self {
        Self { radius }
    }
}

impl Filter for TriangleFilter {
    fn evaluate(&self, p: &Point2f) -> Float {
        0.0_f32.max(self.radius.x - p.x.abs()) *
        0.0_f32.max(self.radius.y - p.y.abs())
    }

    fn radius(&self) -> Vector2f {
        self.radius
    }
}

pub fn create_triangle_filter(ps: &ParamSet) -> Filters {
    let xw = ps.find_one_float("xwidth", 2.0);
    let yw = ps.find_one_float("ywidth", 2.0);

    TriangleFilter::new(Vector2f::new(xw, yw)).into()
}