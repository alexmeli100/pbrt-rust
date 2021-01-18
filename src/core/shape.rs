use crate::core::geometry::bounds::Bounds3f;
use crate::core::pbrt::Float;
use crate::core::geometry::ray::Ray;
use enum_dispatch::enum_dispatch;
use crate::core::interaction::{SurfaceInteraction, Interaction, Interactions};
use crate::core::geometry::point::Point2f;
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::shapes::sphere::Sphere;
use crate::shapes::cylinder::Cylinder;
use crate::shapes::disk::Disk;
use crate::shapes::triangle::Triangle;

#[enum_dispatch]
pub trait Shape {
    fn object_bound(&self) -> Bounds3f;
    fn world_bound(&self) -> Bounds3f {
        self.object_to_world().transform_bounds(&self.object_bound())
    }
    fn intersect(&self, r: &Ray, t_hit: &mut Float, isect: &mut SurfaceInteraction, test_aphatexture: bool) -> bool;
    fn intersect_p(&self, r: &Ray, test_alpha_texture: bool) -> bool {
        let mut t_hit = r.t_max;
        let mut isect = SurfaceInteraction::default();

        self.intersect(r, &mut t_hit, &mut isect, test_alpha_texture)
    }

    fn area(&self) -> Float;
    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Interactions;
    fn sample_interaction(&self, i: &Interactions, u: &Point2f, pdf: &mut Float) -> Interactions {
        let intr = self.sample(u, pdf);
        let mut wi = intr.p() - i.p();

        if wi.length_squared() == 0.0 {
            *pdf = 0.0;
        } else {
            wi = wi.normalize();
            // Convert from area measure, as returned by the Sample() call
            // above, to solid angle measure.
            *pdf *= i.p().distance_squared(&intr.p()) / intr.n().abs_dot_vec(&(-wi));

            if pdf.is_infinite() { *pdf = 0.0 }
        }

        intr
    }
    fn pdf(&self, i: &Interactions, wi: &Vector3f) -> Float;
    fn reverse_orientation(&self) -> bool;
    fn transform_swapshandedness(&self) -> bool;

    fn object_to_world(&self) -> Transform;
    fn world_to_object(&self) -> Transform;
}

#[enum_dispatch(Shape)]
pub enum Shapes {
    Sphere,
    Cylinder,
    Disk,
    Triangle
}