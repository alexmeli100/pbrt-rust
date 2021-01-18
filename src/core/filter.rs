use enum_dispatch::enum_dispatch;

use crate::filters::boxfilter::BoxFilter;
use crate::filters::triangle::TriangleFilter;
use crate::filters::gaussian::GaussianFilter;
use crate::filters::mitchell::MitchellFilter;
use crate::filters::sinc::LanczosSincFilter;
use crate::core::geometry::vector::Vector2f;
use crate::core::pbrt::Float;
use crate::core::geometry::point::Point2f;

#[enum_dispatch(Filter)]
pub enum Filters {
    BoxFilter,
    TriangleFilter,
    GaussianFilter,
    MitchellFilter,
    LanczosSincFilter
}

#[enum_dispatch]
pub trait Filter {
    fn evaluate(&self, p: &Point2f) -> Float;
    fn radius(&self) -> Vector2f;
}

