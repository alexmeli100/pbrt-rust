use crate::core::geometry::bounds::Bounds3f;
use crate::core::pbrt::Float;
use crate::core::geometry::ray::{Ray, BaseRay};
use std::rc::Rc;
use enum_dispatch::enum_dispatch;
use crate::core::interaction::{SurfaceInteraction, Interaction};
use crate::core::geometry::point::Point2f;
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::shapes::sphere::Sphere;
use std::sync::Arc;

#[enum_dispatch(Shape)]
pub trait IShape {
    fn object_bound(&self) -> Bounds3f;
    fn world_bound(&self) -> Bounds3f {
        self.object_to_world().transform_bounds(&self.object_bound())
    }
    fn intersect(&self, fray: &Ray, t_hit: &mut Float, isect: &mut Rc<SurfaceInteraction>, test_aphatexture: bool) -> bool;
    fn intersect_p(&self, ray: &Ray, test_alpha_texture: bool) -> bool {
        let mut t_hit = ray.t_max();
        let mut isect = Rc::new(SurfaceInteraction::default());

        self.intersect(ray, &mut t_hit, &mut isect, test_alpha_texture)
    }

    fn area(&self) -> Float;
    fn sample(&self, u: &Point2f) -> Interaction;
    fn sample_interaction(&self, i: &Interaction, u: &Point2f) -> Interaction;
    fn pdf(&self, i: &Interaction, wi: &Vector3f) -> Float;
    fn reverse_orientation(&self) -> bool;
    fn transform_swapshandedness(&self) -> bool;

    fn object_to_world(&self) -> Rc<Transform>;
    fn world_to_object(&self) -> Rc<Transform>;
}

#[enum_dispatch]
pub enum Shape {
    Sphere
}