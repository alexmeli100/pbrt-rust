use crate::core::geometry::point::{Point3f, Point2f, Point3};
use crate::core::geometry::vector::{Vector3f, Vector3};
use super::pbrt::*;
use crate::core::geometry::normal::{Normal3f, Normal3};
use super::medium::MediumInterface;
use std::sync::Arc;
use crate::core::medium::Medium;
use crate::core::geometry::geometry::*;
use crate::core::geometry::ray::Ray;
use crate::core::primitive::Primitive;
use crate::core::reflection::BSDF;
use crate::core::bssrdf::BSSRDF;
use std::cell::Cell;
use crate::core::shape::Shape;

pub trait Interaction {
    fn p(&self) -> Point3f;
    fn time(&self) -> Float;
    fn p_error(&self) -> Vector3f;
    fn wo(&self) -> Vector3f;
    fn n(&self) -> Normal3f;
    fn medium_interface(&self) -> Option<MediumInterface>;

    fn spawn_ray(&self, d: &Vector3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), d);

        Ray::new(&o, d, INFINITY, self.time(), self.get_medium_vec(d))
    }

    fn spawn_rayto_point(&self, p2: &Point3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(*p2 - self.p()));
        let d = *p2 - o;

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), self.get_medium_vec(&d))
    }

    fn spawn_rayto_interaction(&self, it: &impl Interaction) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(it.p() - self.p()));
        let t = offset_ray_origin(&it.p(), &it.p_error(), &it.n(), &(o - it.p()));

        let d = t - o;

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), self.get_medium_vec(&d))
    }

    fn get_medium_vec(&self, w: &Vector3f) -> Option<Arc<Medium>> {
        if w.dot_norm(&self.n()) > 0.0 {
            if let Some(ref m) = self.medium_interface() {
                Some(m.outside.clone())
            } else {
                None
            }
        } else {
            if let Some(ref m) = self.medium_interface() {
                Some(m.inside.clone())
            } else {
                None
            }
        }
    }

    fn get_medium(&self) -> Option<Arc<Medium>> {
        if let Some(ref m) = self.medium_interface() {
            return Some(m.inside.clone());
        }

        None
    }

    fn is_surface_interaction(&self) -> bool {
        self.n() != Normal3f::default()
    }

    fn is_medium_interaction(&self) -> bool {
        !self.is_surface_interaction()
    }
}

#[derive(Default, Debug, Clone)]
pub struct InteractionData {
    pub p: Point3f,
    pub time: Float,
    pub p_error: Vector3f,
    pub wo: Vector3f,
    pub n: Normal3f,
    pub medium_interface: Option<MediumInterface>
}

impl InteractionData {
    fn new(p: &Point3f, n: & Normal3f, p_error: &Vector3f, wo: &Vector3f, time: Float, medium_interface: Option<MediumInterface>) -> Self {
        Self { p: *p, time, p_error: *p_error, wo: wo.normalize(), n: *n, medium_interface: medium_interface.clone() }
    }
}

#[derive(Default)]
pub struct SurfaceInteraction {
    pub interaction_data: InteractionData,
    pub uv: Point2f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f,
    pub shape: Option<Arc<dyn Shape>>,
    pub shading: Shading,
    pub primitive: Option<Arc<dyn Primitive>>,
    pub bsdf: Option<Arc<BSDF>>,
    pub bssrdf: Option<Arc<dyn BSSRDF>>,
    pub dpdx: Cell<Vector3f>,
    pub dpdy: Cell<Vector3f>,
    pub dudx: Cell<Float>,
    pub dvdx: Cell<Float>,
    pub dudy: Cell<Float>,
    pub dvdy: Cell<Float>
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Shading {
    pub n: Normal3f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f
}

impl SurfaceInteraction {
    pub fn new(p: &Point3f, p_error: &Vector3f, uv: &Point2f, wo: &Vector3f, dpdu: &Vector3f, dpdv: &Vector3f, dndu: &Normal3f, dndv: &Normal3f, time: Float, shape: Option<Arc<dyn Shape>>) -> Self {
        let n: Normal3f = dpdu.cross(dpdv).normalize().into();
        let mut interaction_data = InteractionData::new(p, &n.into(), p_error, wo, time, None);
        let mut shading = Shading {n, dpdu: *dpdu, dpdv: *dpdv, dndu: *dndu, dndv: *dndv };

        if let Some(ref s) = shape {
            if s.reverse_orientation() ^ s.transform_swapshandedness() {
                interaction_data.n *= -1.0;
                shading.n *= -1.0;
            }
        }

        let shape_c = match shape {
            Some(ref s) => Some(s.clone()),
            _ => None
        };
        SurfaceInteraction {
            interaction_data,
            shading,
            uv: *uv,
            dpdu: *dpdu,
            dpdv: *dpdv,
            dndu: *dndu,
            dndv: *dndv,
            shape: shape_c,
            primitive: None,
            bsdf: None,
            bssrdf: None,
            dpdx: Cell::new(Vector3f::default()),
            dpdy: Cell::new(Vector3f::default()),
            dudx: Cell::new(0.0),
            dvdx: Cell::new(0.0),
            dudy: Cell::new(0.0),
            dvdy: Cell::new(0.0)
        }
    }

    pub fn set_shading_geometry(&mut self, dpdus: &Vector3f, dpdvs: &Vector3f, dndus: &Normal3f, dndvs: &Normal3f, orientation_is_authoritative: bool) {
        self.shading.n = dpdus.cross(dpdvs).normalize().into();

        if let Some(ref s) = self.shape {
            if s.reverse_orientation() ^ s.transform_swapshandedness() {
                self.shading.n = -self.shading.n;
            }

            if orientation_is_authoritative {
                self.interaction_data.n = self.interaction_data.n.face_foward(&self.shading.n);
            } else {
                self.shading.n = self.shading.n.face_foward(&self.interaction_data.n);
            }
        }

        self.shading.dpdu = *dpdus;
        self.shading.dpdv = *dpdvs;
        self.shading.dndu = *dndus;
        self.shading.dndv = *dndvs;
    }

    pub fn get_bsdf(&self) -> Option<Arc<BSDF>> {
        match self.bsdf {
            Some(ref bsdf) => Some(bsdf.clone()),
            _ => None
        }
    }

    pub fn get_shape(&self) -> Option<Arc<dyn Shape>> {
        match self.shape {
            Some(ref shape) => Some(shape.clone()),
            _ => None
        }
    }

    pub fn get_bssrdf(&self) -> Option<Arc<dyn BSSRDF>> {
        match self.bssrdf {
            Some(ref bssrdf) => Some(bssrdf.clone()),
            _ => None
        }
    }

    pub fn get_primitive(&self) -> Option<Arc<dyn Primitive>> {
        match self.primitive {
            Some(ref primitive) => Some(primitive.clone()),
            _ => None
        }
    }
}

impl Interaction for SurfaceInteraction {
    fn p(&self) -> Point3<f32> {
        self.interaction_data.p
    }

    fn time(&self) -> f32 {
        self.interaction_data.time
    }

    fn p_error(&self) -> Vector3<f32> {
        self.interaction_data.p_error
    }

    fn wo(&self) -> Vector3<f32> {
        self.interaction_data.wo
    }

    fn n(&self) -> Normal3<f32> {
        self.interaction_data.n
    }

    fn medium_interface(&self) -> Option<MediumInterface> {
        match self.interaction_data.medium_interface {
            Some(ref m) => Some(m.clone()),
            _ => None
        }
    }
}