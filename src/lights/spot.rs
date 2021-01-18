use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::Spectrum;
use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::pbrt::{Float, radians, PI};
use crate::core::light::{LightFlags, Light, VisibilityTester};
use crate::init_light_data;
use crate::core::geometry::vector::Vector3f;
use crate::core::interaction::{Interactions, SurfaceInteraction, Interaction};
use crate::core::geometry::ray::Ray;
use crate::core::scene::Scene;
use crate::core::geometry::normal::Normal3f;

#[derive(Default)]
pub struct SpotLight {
    plight              : Point3f,
    I                   : Spectrum,
    cos_total_width     : Float,
    cos_falloff_start   : Float,
    // Light Data
    flags               : u8,
    nsamples            : usize,
    medium_interface    : MediumInterface,
    light_to_world      : Transform,
    world_to_light      : Transform
}

impl SpotLight {
    pub fn new(
        l2w: &Transform, mi: MediumInterface, I: &Spectrum,
        total_width: Float, falloff_start: Float) -> Self {
        let flags = LightFlags::DeltaPosition as u8;
        let mut sl = SpotLight::default();
        init_light_data!(sl, flags, 1, mi, l2w);

        sl.plight = l2w.transform_point(&Point3f::new(0.0, 0.0, 0.0));
        sl.I = *I;
        sl.cos_total_width = radians(total_width).cos();
        sl.cos_falloff_start = radians(falloff_start).cos();

        sl
    }

    pub fn falloff(&self, w: &Vector3f) -> Float {
        let wl = self.world_to_light.transform_vector(w);
        let cos_theta = wl.z;

        if cos_theta < self.cos_total_width { return 0.0; }
        if cos_theta >= self.cos_falloff_start { return 1.0; }

        // Compute falloff inside spotlight cone
        let delta =
            (cos_theta - self.cos_total_width) / (self.cos_falloff_start - self.cos_total_width);

        (delta * delta) * (delta * delta)
    }
}

impl Light for SpotLight {
    fn power(&self) -> Spectrum {
        self.I * 2.0 * PI * (1.0 - 0.5 * (self.cos_falloff_start + self.cos_total_width))
    }

    fn le(&self, r: &Ray) -> Spectrum {
        unimplemented!()
    }

    fn pdf_li(&self, re: &Interactions, wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(&self, re: &Interactions, u: &Point2f, wi: &mut Vector3f, pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
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

        self.I * self.falloff(&(-*wi)) / self.plight.distance_squared(&re.p())
    }

    fn sample_le(&self, u1: &Point2f, u2: &Point2f, time: f32, ray: &mut Ray, nlight: &mut Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) -> Spectrum {
        unimplemented!()
    }

    fn pdf_le(&self, ray: &Ray, nlight: &Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) {
        unimplemented!()
    }
}