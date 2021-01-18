use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::Spectrum;
use crate::init_light_data;
use crate::core::transform::Transform;
use crate::core::medium::MediumInterface;
use crate::core::light::{LightFlags, Light, VisibilityTester};
use crate::core::geometry::vector::Vector3f;
use crate::core::interaction::{Interactions, SurfaceInteraction};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::normal::Normal3f;
use crate::core::scene::Scene;
use crate::core::pbrt::{PI, Float};
use crate::core::interaction::Interaction;

#[derive(Default)]
pub struct PointLight {
    plight          : Point3f,
    I               : Spectrum,
    // Light Data
    flags           : u8,
    nsamples        : usize,
    medium_interface: MediumInterface,
    light_to_world  : Transform,
    world_to_light  : Transform
}

impl PointLight {
    pub fn new(l2w: &Transform, mi: MediumInterface, I: &Spectrum) -> Self {
        let flags = LightFlags::DeltaPosition as u8;
        let mut pl = PointLight::default();
        init_light_data!(pl, flags, 1, mi, l2w);

        pl.plight = l2w.transform_point(&Point3f::new(0.0, 0.0, 0.0));
        pl.I = *I;

        pl
    }
}

impl Light for PointLight {
    fn power(&self) -> Spectrum {
        self.I * 4.0 * PI
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
        *wi = (self.plight - re.p()).normalize();
        *pdf = 1.0;
        let s1 = SurfaceInteraction {
            p       : re.p(),
            n       : re.n(),
            time    : re.time(),
            p_error : re.p_error(),
            wo      : re.wo(),
            ..Default::default()

        };
        let s2 = SurfaceInteraction {
            p               : self.plight,
            time            : re.time(),
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(s1.into(), s2.into());

        self.I / self.plight.distance_squared(&re.p())
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: f32,
        ray: &mut Ray, nlight: &mut Normal3f,
        pdf_pos: &mut f32, pdf_dir: &mut f32) -> Spectrum {
        unimplemented!()
    }

    fn pdf_le(&self, ray: &Ray, nlight: &Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) {
        unimplemented!()
    }
}