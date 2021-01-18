use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::{Spectrum, RGBSpectrum, SpectrumType};
use crate::core::mipmap::MIPMap;
use crate::mipmap;
use crate::init_light_data;
use crate::core::light::{LightFlags, Light, VisibilityTester};
use crate::core::imageio::read_image;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::geometry::{spherical_theta, spherical_phi};
use crate::core::pbrt::{INV2_PI, INV_PI, Float, PI};
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interactions, SurfaceInteraction, Interaction};
use crate::core::geometry::normal::Normal3f;

#[derive(Default)]
pub struct GonioPhotometricLight {
    plight           : Point3f,
    I                : Spectrum,
    mipmap           : Option<MIPMap<RGBSpectrum>>,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl GonioPhotometricLight {
    pub fn new(
        l2w: &Transform, mi: MediumInterface,
        I: &Spectrum, texname: &str) -> Self {
        let flags = LightFlags::DeltaPosition as u8;
        let mut cl = GonioPhotometricLight::default();
        init_light_data!(cl, flags, 1, mi, l2w);

        match read_image(texname) {
            Ok((m, res)) => cl.mipmap = Some(mipmap!(&res, m)),
            _            => cl.mipmap = None
        }

        cl.I = *I;
        cl.plight = l2w.transform_point(&Point3f::new(0.0, 0.0, 0.0));

        cl
    }

    fn scale(&self, w: &Vector3f) -> Spectrum {
        let mut wp = self.world_to_light.transform_vector(w);
        std::mem::swap(&mut wp.y, &mut wp.z);
        let theta = spherical_theta(&wp);
        let phi = spherical_phi(&wp);
        let st = Point2f::new(phi * INV2_PI, theta * INV_PI);

        match self.mipmap.as_ref() {
            Some(m) => {
                let s = m.lookup(&st, 0.0);
                Spectrum::from_rgb_spectrum(&s, SpectrumType::Illuminant)
            },
            _       => Spectrum::new(1.0)
        }
    }
}

impl Light for GonioPhotometricLight {
    fn power(&self) -> Spectrum {
        let r = match self.mipmap.as_ref() {
            Some(m) => m.lookup(&Point2f::new(0.5, 0.5), 0.5),
            _       => RGBSpectrum::new(1.0)
        };
        let s = Spectrum::from_rgb_spectrum(&r, SpectrumType::Illuminant);

        self.I * s * 4.0 * PI
    }

    fn le(&self, r: &Ray) -> Spectrum {
        unimplemented!()
    }

    fn pdf_li(&self, re: &Interactions, wi: &Vector3f) -> f32 {
        0.0
    }

    fn sample_li(
        &self, re: &Interactions, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut Float, vis: &mut VisibilityTester) -> Spectrum {
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

        self.I * self.scale(&(-*wi)) / self.plight.distance_squared(&re.p())
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