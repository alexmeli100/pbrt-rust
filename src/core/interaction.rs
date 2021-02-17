use crate::core::geometry::point::{Point3f, Point2f, Point3};
use crate::core::geometry::vector::{Vector3f, Vector3};
use enum_dispatch::enum_dispatch;
use super::pbrt::*;
use crate::core::geometry::normal::{Normal3f, Normal3};
use super::medium::MediumInterface;
use std::sync::Arc;
use bumpalo_herd::Member;
use crate::core::medium::{Mediums, PhaseFunctions};
use crate::core::geometry::geometry::*;
use crate::core::geometry::ray::Ray;
use crate::core::primitive::{Primitive, Primitives};
use crate::core::reflection::BSDF;
use crate::core::bssrdf::{BSSRDFs};
use std::cell::Cell;
use crate::core::shape::{Shapes, Shape};
use crate::core::transform::solve_linearsystem_2x2;
use crate::core::material::TransportMode;
use crate::core::spectrum::Spectrum;
use crate::core::light::Light;
use crate::integrators::bdpt::EndpointInteraction;

#[enum_dispatch(Interactions)]
pub trait Interaction {
    fn p(&self) -> Point3f;
    fn time(&self) -> Float;
    fn p_error(&self) -> Vector3f;
    fn wo(&self) -> Vector3f;
    fn n(&self) -> Normal3f;
    fn medium_interface(&self) -> Option<MediumInterface>;

    fn spawn_ray(&self, d: &Vector3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), d);

        Ray::new(&o, d, INFINITY, self.time(), self.get_medium_vec(d), None)
    }

    fn spawn_rayto_point(&self, p2: &Point3f) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(*p2 - self.p()));
        let d = *p2 - self.p();

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), self.get_medium_vec(&d), None)
    }

    fn spawn_rayto_interaction(&self, it: &InteractionData) -> Ray {
        let o = offset_ray_origin(&self.p(), &self.p_error(), &self.n(), &(it.p() - self.p()));
        let t = offset_ray_origin(&it.p(), &it.p_error(), &it.n(), &(o - it.p()));

        let d = t - o;

        Ray::new(&o, &d, 1.0 - SHADOW_EPSILON, self.time(), self.get_medium_vec(&d), None)
    }

    fn get_medium_vec(&self, w: &Vector3f) -> Option<Arc<Mediums>> {
        if w.dot_norm(&self.n()) > 0.0 {
            self.medium_interface()
                .map_or_else(|| None, |m| m.outside)
        } else {
            self.medium_interface()
                .map_or_else(|| None, |m| m.inside)
        }
    }

    fn get_medium(&self) -> Option<Arc<Mediums>> {
        // TODO: Check eq
        //assert!(Arc::ptr_eq())

        if let Some(ref m) = self.medium_interface() {
            return m.inside.clone();
        }

        None
    }

    fn get_data(&self) -> InteractionData {
        InteractionData {
            p: self.p(),
            time: self.time(),
            p_error: self.p_error(),
            wo: self.wo(),
            n: self.n(),
            medium_interface: self.medium_interface()
        }
    }

    fn is_surface_interaction(&self) -> bool {
        self.n() != Normal3f::default()
    }

    fn is_medium_interaction(&self) -> bool {
        !self.is_surface_interaction()
    }
}

// For use with Light types
#[derive(Clone, Default)]
pub struct InteractionData {
    pub p                   : Point3f,
    pub time                : Float,
    pub p_error             : Vector3f,
    pub wo                  : Vector3f,
    pub n                   : Normal3f,
    pub medium_interface    : Option<MediumInterface>
}

impl InteractionData {
    pub fn new(
        p: Point3f, n: Normal3f,  p_error: Vector3f, wo: Vector3f,
        time: Float, medium_interface: Option<MediumInterface>) -> Self {
        Self {
            p, time, p_error, wo,
            n, medium_interface
        }
    }
}

impl Interaction for InteractionData {
    fn p(&self) -> Point3f {
        self.p
    }

    fn time(&self) -> f32 {
        self.time
    }

    fn p_error(&self) -> Vector3f {
        self.p_error
    }

    fn wo(&self) -> Vector3f {
        self.wo
    }

    fn n(&self) -> Normal3f {
        self.n
    }

    fn medium_interface(&self) -> Option<MediumInterface> {
        self.medium_interface.clone()
    }
}

#[derive(Default)]
pub struct SurfaceInteraction<'a> {
    // Interaction Data
    pub p                   : Point3f,
    pub time                : Float,
    pub p_error             : Vector3f,
    pub wo                  : Vector3f,
    pub n                   : Normal3f,
    pub medium_interface    : Option<MediumInterface>,
    //pub interaction_data: InteractionData,
    pub uv                  : Point2f,
    pub dpdu                : Vector3f,
    pub dpdv                : Vector3f,
    pub dndu                : Normal3f,
    pub dndv                : Normal3f,
    pub shape               : Option<Arc<Shapes>>,
    pub shading             : Shading,
    pub primitive           : Option<Arc<Primitives>>,
    pub bsdf                : Option<&'a mut BSDF<'a>>,
    pub bssrdf              : Option<&'a BSSRDFs>,
    pub dpdx                : Cell<Vector3f>,
    pub dpdy                : Cell<Vector3f>,
    pub dudx                : Cell<Float>,
    pub dvdx                : Cell<Float>,
    pub dudy                : Cell<Float>,
    pub dvdy                : Cell<Float>
}

#[derive(Debug, Default, Copy, Clone)]
pub struct Shading {
    pub n   : Normal3f,
    pub dpdu: Vector3f,
    pub dpdv: Vector3f,
    pub dndu: Normal3f,
    pub dndv: Normal3f
}

impl<'a> SurfaceInteraction<'a> {
    pub fn new(p: &Point3f, p_error: &Vector3f, uv: &Point2f, wo: &Vector3f,
               dpdu: &Vector3f, dpdv: &Vector3f, dndu: &Normal3f, dndv: &Normal3f,
               time: Float, shape: Option<Arc<Shapes>>) -> Self {
        let mut n: Normal3f = dpdu.cross(dpdv).normalize().into();
        //let mut interaction_data = InteractionData::new(p, &n, p_error, wo, time, None);
        let mut shading = Shading {n, dpdu: *dpdu, dpdv: *dpdv, dndu: *dndu, dndv: *dndv };


        if let Some(ref s) = shape {
            if s.reverse_orientation() ^ s.transform_swapshandedness() {
                n *= -1.0;
                shading.n *= -1.0;
            }
        }

        SurfaceInteraction {
            //interaction_data,
            n,
            time,
            shape,
            shading,
            p_error: *p_error,
            wo: *wo,
            p: *p,
            uv: *uv,
            dpdu: *dpdu,
            dpdv: *dpdv,
            dndu: *dndu,
            dndv: *dndv,
            medium_interface: None,
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

    pub fn set_shading_geometry(
        &mut self, dpdus: &Vector3f, dpdvs: &Vector3f, dndus: &Normal3f,
        dndvs: &Normal3f, orientation_is_authoritative: bool) {
        self.shading.n = dpdus.cross(dpdvs).normalize().into();

        if let Some(ref s) = self.shape {
            if s.reverse_orientation() ^ s.transform_swapshandedness() {
                self.shading.n = -self.shading.n;
            }

            if orientation_is_authoritative {
                self.n = self.n.face_foward(&self.shading.n);
            } else {
                self.shading.n = self.shading.n.face_foward(&self.n);
            }
        }

        self.shading.dpdu = *dpdus;
        self.shading.dpdv = *dpdvs;
        self.shading.dndu = *dndus;
        self.shading.dndv = *dndvs;
    }


    pub fn get_shape(&self) -> Option<Arc<Shapes>> {
        self.shape.clone()
    }

    pub fn get_bssrdf(&self) -> Option<&'a BSSRDFs> {
        self.bssrdf
    }

    pub fn get_primitive(&self) -> Option<Arc<Primitives>> {
        self.primitive.clone()
    }

    pub fn compute_scattering_functions(
        &mut self, r: &Ray, arena: &Member<'a>,
        allow_multiple_lobes: bool, mode: TransportMode) {
        self.compute_differentials(r);
        if let Some(p) = self.primitive.clone() {
            p.compute_scattering_functions(self, arena, mode, allow_multiple_lobes)
        }
    }

    pub fn compute_differentials(&mut self, r: &Ray) {
        macro_rules! fail {
            () => {{
                self.dudx = Cell::new(0.0);
                self.dvdx = Cell::new(0.0);
                self.dudy = Cell::new(0.0);
                self.dvdy = Cell::new(0.0);
                self.dpdx = Cell::new(Default::default());
                self.dpdy = Cell::new(Default::default());

                return;
            }}
        }

        if r.diff.is_none() { fail!() }

        let diff = r.diff.as_ref().unwrap();

        if !diff.has_differentials {
            fail!()
        }

        let d = self.n.dot_vec(&Vector3f::new(self.p.x, self.p.y, self.p.z));
        let tx =
            -(self.n.dot_vec(&Vector3f::from(diff.rx_origin)) - d) /
             self.n.dot_vec(&diff.rx_direction);

        if tx.is_infinite() || tx.is_nan() { fail!() }

        let px = diff.rx_origin + diff.rx_direction * tx;
        let ty =
            -(self.n.dot_vec(&Vector3f::from(diff.ry_origin)) - d) /
            self.n.dot_vec(&diff.ry_direction);

        if ty.is_infinite() || ty.is_nan() { fail!() }

        let py = diff.ry_origin + diff.ry_direction * ty;
        self.dpdx = Cell::new(px - self.p);
        self.dpdy = Cell::new(py - self.p);

        // Compute (u, v) offsets at auxilliary points

        // Choose two dimensions to use for ray offset computation
        let mut dim = [0; 2];

        if self.n.x.abs() > self.n.y.abs() && self.n.x.abs() > self.n.z.abs() {
            dim[0] = 1;
            dim[1] = 2;
        } else if self.n.y.abs() > self.n.z.abs() {
            dim[0] = 0;
            dim[1] = 2;
        } else {
            dim[0] = 0;
            dim[1] = 1;
        }

        // Initialize A, Bx, and By matrices for offset computation
        let A = [
            [self.dpdu[dim[0]], self.dpdv[dim[0]]],
            [self.dpdu[dim[1]], self.dpdv[dim[1]]]
        ];
        let Bx = [px[dim[0]] - self.p[dim[0]], px[dim[1]] - self.p[dim[1]]];
        let By = [py[dim[0]] - self.p[dim[0]], py[dim[1]] - self.p[dim[1]]];

        if !solve_linearsystem_2x2(A, Bx, self.dudx.get_mut(), self.dvdx.get_mut()) {
            self.dudx.set(0.0);
            self.dvdx.set(0.0);
        }

        if !solve_linearsystem_2x2(A, By, self.dudy.get_mut(), self.dvdy.get_mut()) {
            self.dudy.set(0.0);
            self.dvdy.set(0.0);
        }
    }

    pub fn le(&self, w: &Vector3f) -> Spectrum {
        match self.primitive.as_ref().unwrap().get_area_light() {
            Some(ref a) => a.l(self, w),
            _          => Spectrum::new(0.0)
        }
    }
}

impl<'a> Interaction for SurfaceInteraction<'a> {
    #[inline]
    fn p(&self) -> Point3<f32> {
        self.p
    }

    #[inline]
    fn time(&self) -> f32 {
        self.time
    }

    #[inline]
    fn p_error(&self) -> Vector3<f32> {
        self.p_error
    }

    #[inline]
    fn wo(&self) -> Vector3<f32> {
        self.wo
    }

    #[inline]
    fn n(&self) -> Normal3<f32> {
        self.n
    }

    #[inline]
    fn medium_interface(&self) -> Option<MediumInterface> {
        self.medium_interface.clone()
    }
}

#[derive(Default, Clone)]
pub struct MediumInteraction {
    // Interaction Data
    pub p                   : Point3f,
    pub time                : Float,
    pub p_error             : Vector3f,
    pub wo                  : Vector3f,
    pub n                   : Normal3f,
    pub medium_interface    : Option<MediumInterface>,
    pub phase               : Option<PhaseFunctions>
}

impl MediumInteraction {
    pub fn new(
        p: &Point3f, wo: &Vector3f, time: Float,
        medium: Option<MediumInterface>,
        phase: Option<PhaseFunctions>) -> Self {
        Self {
            time,
            phase,
            medium_interface: medium,
            p               : *p,
            wo              : *wo,
            n               : Default::default(),
            p_error         : Default::default()
        }
    }

    pub fn isvalid(&self) -> bool {
        self.phase.is_some()
    }
}

impl Interaction for MediumInteraction {
    fn p(&self) -> Point3f {
        self.p
    }

    fn time(&self) -> f32 {
        self.time
    }

    fn p_error(&self) -> Vector3f {
        self.p_error
    }

    fn wo(&self) -> Vector3f {
        self.wo
    }

    fn n(&self) -> Normal3f {
        self.n
    }

    fn medium_interface(&self) -> Option<MediumInterface> {
        self.medium_interface.clone()
    }
}


#[enum_dispatch]
pub enum Interactions<'a> {
    InteractionData(InteractionData),
    MediumInteraction(MediumInteraction),
    EndpointInteraction(EndpointInteraction<'a>),
    SurfaceInteraction(SurfaceInteraction<'a>)
}

impl<'a> Interactions<'a> {
    pub fn get_surfaceinteraction(self) -> SurfaceInteraction<'a> {
        match self {
            Interactions::SurfaceInteraction(s) => s,
            _ => panic!("Not a SurfaceInteraction")
        }
    }

    pub fn get_mediuminteraction(self) -> MediumInteraction {
        match self {
            Interactions::MediumInteraction(m) => m,
            _ => panic!("Not a MediumInteraction")
        }
    }
}