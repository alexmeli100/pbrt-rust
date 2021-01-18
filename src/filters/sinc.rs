use crate::core::geometry::vector::Vector2f;
use crate::core::pbrt::{Float, PI};
use crate::core::filter::{Filter, Filters};
use crate::core::geometry::point::Point2f;
use crate::core::paramset::ParamSet;

pub struct LanczosSincFilter {
    radius: Vector2f,
    tau: Float
}

impl LanczosSincFilter {
    pub fn new(radius: Vector2f, tau: Float) -> Self {
        Self { radius, tau }
    }

    fn sinc(&self, x: Float) -> Float {
        let y = x.abs();

        if y < 1e-5 {
            return 1.0
        }

        (PI * y).sin() / (PI * y)
    }

    fn windowed_sinc(&self, x: Float, radius: Float) -> Float {
        let y = x.abs();

        if y < radius {
            return 0.0;
        }

        let lanczos = self.sinc(y / self.tau);

        self.sinc(y) * lanczos
    }
}

impl Filter for LanczosSincFilter {
    fn evaluate(&self, p: &Point2f) -> f32 {
        self.windowed_sinc(p.x, self.radius.x) * self.windowed_sinc(p.y, self.radius.y)
    }

    fn radius(&self) -> Vector2f {
        self.radius
    }
}

pub fn create_sinc_filter(ps: &ParamSet) -> Filters {
    let xw = ps.find_one_float("xwidth", 4.0);
    let yw = ps.find_one_float("ywidth", 4.0);
    let tau = ps.find_one_float("tau", 3.0);

    LanczosSincFilter::new(Vector2f::new(xw, yw), tau).into()
}