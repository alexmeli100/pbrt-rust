use enum_dispatch::enum_dispatch;
use crate::core::interaction::{Interaction, Interactions, SurfaceInteraction};
use crate::core::scene::Scene;
use crate::core::sampler::Samplers;
use crate::core::spectrum::Spectrum;
use crate::core::primitive::Primitive;
use crate::core::medium::Medium;
use crate::core::geometry::ray::Ray;
use crate::core::geometry::vector::Vector3f;
use crate::core::pbrt::Float;
use crate::core::geometry::point::Point2f;
use crate::core::geometry::normal::Normal3f;
use crate::lights::projection::ProjectionLight;
use crate::lights::distant::DistantLight;
use crate::lights::spot::SpotLight;
use crate::lights::goniometric::GonioPhotometricLight;
use crate::lights::point::PointLight;
use crate::lights::diffuse::DiffuseAreaLight;

#[macro_export]
macro_rules! init_light_data {
    ($l:ident, $f:expr, $n:expr, $mi:expr, $ltw:expr) => {
        $l.flags = $f;
        $l.nsamples = std::cmp::max(1, $n);
        $l.medium_interface = $mi;
        $l.light_to_world = *$ltw;
        $l.world_to_light = crate::core::transform::Transform::inverse($ltw);
    }
}

#[repr(u8)]
pub enum LightFlags {
    DeltaPosition   = 1,
    DeltaDirection  = 2,
    Area            = 4,
    Infinite        = 8
}

#[enum_dispatch]
pub trait Light {
    fn power(&self, ) -> Spectrum;

    fn preprocess(&mut self, _scene: &Scene){}

    fn le(&self, r: &Ray) -> Spectrum;

    fn pdf_li(&self, re: &Interactions, wi: &Vector3f) -> Float;

    fn sample_li(
        &self, re: &Interactions, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut Float, vis: &mut VisibilityTester) -> Spectrum;

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: Float,
        ray: &mut Ray, nlight: &mut Normal3f, pdf_pos: &mut Float,
        pdf_dir: &mut Float) -> Spectrum;

    fn pdf_le(
        &self, ray: &Ray, nlight: &Normal3f,
        pdf_pos: &mut Float, pdf_dir: &mut Float);

}

#[enum_dispatch(Light)]
pub enum Lights {
    AreaLights,
    SpotLight,
    PointLight,
    DistantLight,
    ProjectionLight,
    DiffuseAreaLight,
    GonioPhotometricLight

}

#[enum_dispatch]
pub trait AreaLight: Light + Clone {
    fn l<I: Interaction>(&self, intr: &I, w: &Vector3f) -> Spectrum;
}

#[enum_dispatch(AreaLight, Light)]
#[derive(Clone)]
pub enum AreaLights {
    DiffuseAreaLight
}

pub struct VisibilityTester<'a, 'b> {
    p0: Interactions<'a>,
    p1: Interactions<'b>
}

impl<'a, 'b> VisibilityTester<'a, 'b> {
    pub fn new(p0: Interactions<'a>, p1: Interactions<'b>) -> Self {
        Self { p0, p1 }
    }

    pub fn p0(&self) -> &Interactions<'a> {
        &self.p0
    }

    pub fn p1(&self) -> &Interactions<'b> {
        &self.p1
    }

    pub fn unoccluded(&self, scene: &Scene) -> bool {
        let mut r = self.p0.spawn_rayto_interaction(&self.p1);
        !scene.intersect_p(&mut r)
    }

    pub fn tr(&self, scene: &Scene, sampler: &mut Samplers) -> Spectrum {
        let mut ray = self.p0.spawn_rayto_interaction(&self.p1);
        let mut Tr = Spectrum::new(1.0);

        loop {
            let mut isect = SurfaceInteraction::default();

            let hitt = scene.intersect(&mut ray, &mut isect);

            // Handle opaque surface along ray's path
            if hitt && isect.primitive.as_ref().unwrap().get_material().is_some() {
                return Spectrum::new(0.0);
            }

            // Update transmittance for current ray segment
            if let Some(ref m) = ray.medium {
                Tr *= m.tr(&ray, sampler);
            }

            // Generate next ray segment or return final transmittance
            if !hitt { break; }
            ray = isect.spawn_rayto_interaction(&self.p1)
        }

        Tr
    }
}