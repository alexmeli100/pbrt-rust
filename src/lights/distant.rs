use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::pbrt::{Float, PI};
use crate::init_light_data;
use crate::core::light::{LightFlags, Light, VisibilityTester};
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interactions, SurfaceInteraction, Interaction};
use crate::core::scene::Scene;
use crate::core::geometry::normal::Normal3f;

#[derive(Default)]
pub struct DistantLight {
    L                : Spectrum,
    wlight           : Vector3f,
    world_center     : Point3f,
    world_radius     : Float,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl DistantLight {
    pub fn new(l2w: &Transform, L: &Spectrum, w: &Vector3f) -> Self {
        // Initialize Light data
        let flags = LightFlags::DeltaDirection as u8;
        let mut dl = DistantLight::default();
        init_light_data!(dl, flags, 1, MediumInterface::default(), l2w);

        dl.L = *L;
        dl.wlight = l2w.transform_vector(w).normalize();

        dl
    }
}

impl Light for DistantLight {
    fn power(&self) -> Spectrum {
        self.L * PI * self.world_radius * self.world_radius
    }

    fn preprocess(&mut self, scene: &Scene) {
        let (p, f) = scene.wb.bounding_sphere();
        self.world_center = p;
        self.world_radius = f;
    }

    fn le(&self, r: &Ray) -> Spectrum {
        unimplemented!()
    }

    fn pdf_li(&self, re: &Interactions, wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(
        &self, re: &Interactions, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        *wi = self.wlight;
        *pdf = 1.0;
        let poutside = re.p() + self.wlight * (2.0 * self.world_radius);
        let s1 = SurfaceInteraction {
            p       : re.p(),
            n       : re.n(),
            time    : re.time(),
            p_error : re.p_error(),
            wo      : re.wo(),
            ..Default::default()

        };
        let s2 = SurfaceInteraction {
            p               : poutside,
            time            : re.time(),
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(s1.into(), s2.into());

        self.L
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: Float, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        unimplemented!()
    }

    fn pdf_le(&self, ray: &Ray, nlight: &Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) {
        unimplemented!()
    }
}