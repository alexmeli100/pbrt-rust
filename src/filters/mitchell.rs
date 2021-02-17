use crate::core::pbrt::Float;
use crate::core::geometry::vector::Vector2f;
use crate::core::filter::{Filter, Filters};
use crate::core::geometry::point::Point2f;
use crate::core::paramset::ParamSet;

pub struct MitchellFilter {
    B: Float,
    C: Float,
    radius: Vector2f,
    inv_radius: Vector2f
}

impl MitchellFilter {
    pub fn new(radius: Vector2f, B: Float, C: Float) -> Self {
        let inv_radius = Vector2f::new(1.0/ radius.x, 1.0 / radius.y);

        Self {
            inv_radius,
            radius,
            B,
            C
        }
    }

    pub fn mitchell_1d(&self, x: Float) -> Float {
        let a = (2.0 * x).abs();

        if a > 1.0 {
            ((-self.B - 6.0 * self.C) * a * a * a + (6.0 * self.B * 30.0 * self.C) * a * a +
             (-12.0 * self.B - 48.0 * self.C) * a + (8.0 * self.B + 24.0 * self.C)) *
             (1.0 / 6.0)
        } else {
            ((12.0 - 9.0 * self.B - 6.0 * self.C) * a * a * a +
             (-18.0 + 12.0 * self.B + 6.0 * self.C) * x * x + (6.0 - 2.0 * self.B)) *
             (1.0 / 6.0)
        }
    }
}

impl Filter for MitchellFilter {
    fn evaluate(&self, p: &Point2f) -> f32 {
        self.mitchell_1d(p.x * self.inv_radius.x) * self.mitchell_1d(p.y * self.inv_radius.y)
    }

    fn radius(&self) -> Vector2f {
        self.radius
    }
}

pub fn create_mitchell_filter(ps: &ParamSet) -> Filters {
    let xw = ps.find_one_float("xwidth", 2.0);
    let yw = ps.find_one_float("ywidth", 2.0);
    let B = ps.find_one_float("B", 1.0 / 3.0);
    let C = ps.find_one_float("C", 1.0 / 3.0);

    MitchellFilter::new(Vector2f::new(xw, yw), B, C).into()
}