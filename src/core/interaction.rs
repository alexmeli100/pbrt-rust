use crate::core::geometry::point::Point3f;
use crate::core::geometry::vector::Vector3f;
use super::pbrt::*;
use crate::core::geometry::normal::Normal3f;
use super::medium::MediumInterface;
use std::sync::Arc;
use crate::core::medium::Medium;
use crate::core::geometry::geometry::*;
use crate::core::geometry::ray::Ray;

pub trait Interaction {
    fn p(&self) -> Point3f;
    fn time(&self) -> Float;
    fn p_error(&self) -> Vector3f;
    fn wo(&self) -> Vector3f;
    fn n(&self) -> Normal3f;
    fn medium_interface(&self) -> MediumInterface;

    fn spawn_ray(&self, d: &Vector3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), d);

        Ray::new(&o, d, INFINITY, self.time(), Some(self.get_medium_vec(d)))
    }

    fn spawn_rayto_point(&self, p2: &Point3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(*p2 - self.p()));
        let d = *p2 - o;

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), Some(self.get_medium_vec(&d)))
    }

    fn spawn_rayto_interaction(&self, it: &impl Interaction) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(it.p() - self.p()));
        let t = offset_ray_origin(&it.p(), &it.p_error(), &it.n(), &(o - it.p()));

        let d = t - o;

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), Some(self.get_medium_vec(&d)))
    }

    fn get_medium_vec(&self, w: &Vector3f) -> Arc<dyn Medium> {
        if w.dot_norm(&self.n()) > 0.0 {
            self.medium_interface().outside.clone()
        } else {
            self.medium_interface().inside.clone()
        }
    }

    fn get_medium(&self) -> Arc<dyn Medium> {
        self.medium_interface().inside.clone()
    }

    fn is_surface_interaction(&self) -> bool {
        self.n() != Normal3f::default()
    }

    fn is_medium_interaction(&self) -> bool {
        !self.is_surface_interaction()
    }
}