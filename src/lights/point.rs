use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::Spectrum;
use crate::init_light_data;
use crate::core::transform::Transform;
use crate::core::medium::MediumInterface;
use crate::core::light::{LightFlags, Light, VisibilityTester, Lights};
use crate::core::geometry::vector::Vector3f;
use crate::core::interaction::{InteractionData};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::normal::Normal3f;
use crate::core::pbrt::{PI, Float, INFINITY};
use crate::core::interaction::Interaction;
use crate::core::paramset::ParamSet;
use std::sync::Arc;
use crate::core::sampling::{uniform_sample_sphere, uniform_sphere_pdf};

#[allow(dead_code)]
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

    fn pdf_li(&self, _re: &InteractionData, _wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(
        &self, re: &InteractionData, _u: &Point2f, wi: &mut Vector3f,
        pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        *wi = (self.plight - re.p).normalize();
        *pdf = 1.0;

        let s2 = InteractionData {
            p               : self.plight,
            time            : re.time(),
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(re.clone(), s2);

        self.I / self.plight.distance_squared(&re.p)
    }

    fn sample_le(
        &self, u1: &Point2f, _u2: &Point2f, time: Float,
        ray: &mut Ray, nlight: &mut Normal3f,
        pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        *ray = Ray::new(
            &self.plight, &uniform_sample_sphere(u1), INFINITY,
            time, self.medium_interface.inside.clone(), None);
        *nlight = Normal3f::from(ray.d);
        *pdf_pos = 1.0;
        *pdf_dir = uniform_sphere_pdf();

        self.I
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

pub fn create_pointlight(l2w: &Transform, mi: MediumInterface, params: &ParamSet) -> Option<Arc<Lights>> {
    let I = params.find_one_spectrum("I", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("scale", Spectrum::new(1.0));
    let P = params.find_one_point3f("from", Default::default());
    let l2wn = Transform::translate(&Vector3f::new(P.x, P.y, P.x)) * *l2w;
    let l = PointLight::new(&l2wn, mi, &(I * sc));

    Some(Arc::new(l.into()))
}