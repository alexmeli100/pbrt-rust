use crate::core::pbrt::{Float, clamp, radians};
use crate::core::transform::Transform;
use std::rc::Rc;
use crate::core::shape::Shape;
use crate::core::geometry::point::Point2;
use crate::core::interaction::{Interaction, SurfaceInteraction};
use crate::core::geometry::bounds::Bounds3;
use crate::core::geometry::vector::Vector3;
use crate::core::geometry::ray::Ray;
use std::sync::Arc;

pub struct Sphere {
    pub radius: Float,
    pub zmin: Float,
    pub zmax: Float,
    pub theta_min: Float,
    pub theta_max: Float,
    pub phi_max: Float,
    pub object_to_world: Rc<Transform>,
    pub world_to_object: Rc<Transform>,
    pub reverse_orientation: bool
}

impl Sphere {
    pub fn new(object_to_world: Rc<Transform>, world_to_object: Rc<Transform>, reverse_orientation: bool, radius: Float, zmin: Float, zmax: Float, phi_max: Float) -> Self {
        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            zmin: clamp(zmin.min(zmax), -radius, radius),
            zmax: clamp(zmin.max(zmax), -radius, radius),
            theta_min: clamp(zmin / radius, -1.0, 1.0).acos(),
            theta_max: clamp(zmax / radius, -1.0, 1.0).acos(),
            phi_max: radians(clamp(phi_max, 0.0, 360.0))
        }
    }
}

impl Shape for Sphere {
    fn object_bound(&self) -> Bounds3<f32> {
        unimplemented!()
    }

    fn intersect(&self, fray: &Ray, t_hit: &mut f32, isect: &mut Rc<SurfaceInteraction>, test_aphatexture: bool) -> bool {
        unimplemented!()
    }

    fn area(&self) -> f32 {
        unimplemented!()
    }

    fn sample(&self, u: &Point2<f32>) -> Arc<dyn Interaction> {
        unimplemented!()
    }

    fn sample_interaction(&self, i: &dyn Interaction, u: &Point2<f32>) -> Arc<dyn Interaction> {
        unimplemented!()
    }

    fn pdf(&self, i: &dyn Interaction, wi: &Vector3<f32>) -> f32 {
        unimplemented!()
    }

    fn reverse_orientation(&self) -> bool {
        unimplemented!()
    }

    fn transform_swapshandedness(&self) -> bool {
        unimplemented!()
    }

    fn object_to_world(&self) -> Rc<Transform> {
        unimplemented!()
    }

    fn world_to_object(&self) -> Rc<Transform> {
        unimplemented!()
    }
}
