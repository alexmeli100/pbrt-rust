use crate::core::pbrt::Float;
use crate::core::geometry::vector::Vector2f;
use crate::core::filter::{Filter, Filters};
use crate::core::geometry::point::Point2f;
use crate::core::paramset::ParamSet;

pub struct GaussianFilter {
    alpha   : Float,
    expx    : Float,
    expy    : Float,
    radius  : Vector2f
}

impl GaussianFilter {
    pub fn new(radius: Vector2f, alpha: Float) -> Self {
        Self {
            radius,
            alpha,
            expx: (-alpha * radius.x * radius.x).exp(),
            expy: (-alpha * radius.y * radius.y).exp()
        }
    }

    fn gaussian(&self, d: Float, expv: Float) -> Float {
        (0.0 as Float).max((-self.alpha * d * d).exp() - expv)
    }
}

impl Filter for GaussianFilter {
    fn evaluate(&self, p: &Point2f) -> f32 {
        self.gaussian(p.x, self.expx) * self.gaussian(p.y, self.expy)
    }

    fn radius(&self) -> Vector2f {
        self.radius
    }
}

pub fn create_gaussian_filter(ps: &ParamSet) -> Filters {
    let xw = ps.find_one_float("xwidth", 2.0);
    let yw = ps.find_one_float("ywidth", 2.0);
    let alpha = ps.find_one_float("alpha", 2.0);

    GaussianFilter::new(Vector2f::new(xw, yw), alpha).into()
}