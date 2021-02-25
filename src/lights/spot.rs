use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::spectrum::Spectrum;
use crate::core::medium::MediumInterface;
use crate::core::transform::{Transform, Matrix4x4};
use crate::core::pbrt::{Float, radians, PI, INFINITY};
use crate::core::light::{LightFlags, Light, VisibilityTester, Lights};
use crate::init_light_data;
use crate::core::geometry::vector::{Vector3f, vec3_coordinate_system};
use crate::core::interaction::{Interaction, InteractionData};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::normal::Normal3f;
use crate::core::paramset::ParamSet;
use std::sync::Arc;
use crate::core::sampling::{uniform_sample_cone, uniform_cone_pdf};
use crate::core::reflection::cos_theta;

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

    fn pdf_li(&self, _re: &InteractionData, _wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(&self, re: &InteractionData, _u: &Point2f, wi: &mut Vector3f, pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        *wi = (self.plight - re.p()).normalize();
        *pdf = 1.0;

        let s2 = InteractionData {
            p               : self.plight,
            time            : re.time,
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(re.clone(), s2);

        self.I * self.falloff(&(-*wi)) / self.plight.distance_squared(&re.p)
    }

    fn sample_le(
        &self, u1: &Point2f, _u2: &Point2f, time: Float, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        // TODO: ProfilePhase
        let w = uniform_sample_cone(u1, self.cos_total_width);
        *ray = Ray::new(
            &self.plight, &self.light_to_world.transform_vector(&w),
            INFINITY, time, self.medium_interface.inside.clone(), None);
        *nlight = Normal3f::from(ray.d);
        *pdf_pos = 1.0;
        *pdf_dir = uniform_cone_pdf(self.cos_total_width);

        self.I * self.falloff(&ray.d)
    }

    fn pdf_le(&self, ray: &Ray, _nlight: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // TODO ProfilePhase
        *pdf_pos = 0.0;
        let v = self.world_to_light.transform_vector(&ray.d);
        *pdf_dir = if cos_theta(&v) > self.cos_total_width {
            uniform_cone_pdf(self.cos_total_width)
        } else {
            0.0
        };
    }

    fn nsamples(&self) -> usize { self.nsamples }

    fn flags(&self) -> u8 {
        self.flags
    }
}

pub fn create_spotlight(l2w: &Transform, mi: MediumInterface, params: &ParamSet) -> Option<Arc<Lights>> {
    let I = params.find_one_spectrum("I", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("scale", Spectrum::new(1.0));
    let coneangle = params.find_one_float("coneangle", 30.0);
    let conedelta = params.find_one_float("conedeltaangle", 5.0);
    // Compute spotlight world to light transformation
    let from = params.find_one_point3f("from", Default::default());
    let to = params.find_one_point3f("to", Point3f::new(0.0, 0.0, 1.0));
    let dir = (to - from).normalize();
    let mut du = Vector3f::default();
    let mut dv = Vector3f::default();
    vec3_coordinate_system(&dir, &mut du, &mut dv);
    let mat = Matrix4x4::from_row_slice(&[
        du.x,  du.y,  du.z,  0.0,
        dv.x,  dv.y,  dv.z,  0.0,
        dir.x, dir.y, dir.z, 0.0,
        0.0,   0.0,   0.0,   1.0
    ]);

    let dirtoz = Transform::from_matrix(&mat);
    let v = Vector3f::new(from.x, from.y, from.z);
    let light2world = *l2w * Transform::translate(&v) * Transform::inverse(&dirtoz);
    let l = SpotLight::new(&light2world, mi, &(I * sc), coneangle, coneangle - conedelta);

    Some(Arc::new(l.into()))
}