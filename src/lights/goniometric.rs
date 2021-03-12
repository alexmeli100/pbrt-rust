use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::{Spectrum, RGBSpectrum, SpectrumType};
use crate::core::mipmap::MIPMap;
use crate::mipmap;
use crate::init_light_data;
use crate::core::light::{LightFlags, Light, VisibilityTester, Lights};
use crate::core::imageio::read_image;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::geometry::{spherical_theta, spherical_phi};
use crate::core::pbrt::{INV2_PI, INV_PI, Float, PI, INFINITY};
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interaction, InteractionData};
use crate::core::geometry::normal::Normal3f;
use crate::core::paramset::ParamSet;
use std::sync::Arc;
use crate::core::sampling::{uniform_sample_sphere, uniform_sphere_pdf};

#[allow(dead_code)]
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
        let mut wp = self.world_to_light.transform_vector(w).normalize();
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

    fn pdf_li(&self, _re: &InteractionData, _wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(
        &self, re: &InteractionData, _u: &Point2f, wi: &mut Vector3f,
        pdf: &mut Float, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        *wi = (self.plight - re.p()).normalize();
        *pdf = 1.0;

        let s2 = InteractionData {
            p               : self.plight,
            time            : re.time(),
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(re.clone(), s2);


        self.I * self.scale(&(-*wi)) / self.plight.distance_squared(&re.p())
    }

    fn sample_le(
        &self, u1: &Point2f, _u2: &Point2f, time: Float, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        // TODO: ProfilePhase
        *ray = Ray::new(
            &self.plight, &uniform_sample_sphere(u1), INFINITY,
            time, self.medium_interface.inside.clone(), None);
        *nlight = Normal3f::from(ray.d);
        *pdf_pos = 1.0;
        *pdf_dir = uniform_sphere_pdf();

        self.I * self.scale(&ray.d)
    }

    fn pdf_le(&self, _ray: &Ray, _nlight: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // TODO ProfilePhase
        *pdf_pos = 0.0;
        *pdf_dir = uniform_sphere_pdf();
    }

    fn nsamples(&self) -> usize { self.nsamples }

    fn flags(&self) -> u8 {
        self.flags
    }
}

pub fn create_goniometriclight(l2w: &Transform, mi: MediumInterface, params: &ParamSet) -> Option<Arc<Lights>> {
    let I = params.find_one_spectrum("I", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("sc", Spectrum::new(1.0));
    let texname = params.find_one_filename("mapname", "");
    let l = GonioPhotometricLight::new(l2w, mi, &(I * sc), &texname);

    Some(Arc::new(l.into()))
}