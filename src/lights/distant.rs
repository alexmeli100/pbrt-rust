use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;
use crate::core::geometry::vector::{Vector3f, vec3_coordinate_system};
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::pbrt::{Float, PI, INFINITY};
use crate::init_light_data;
use crate::core::light::{LightFlags, Light, VisibilityTester, Lights};
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{InteractionData};
use crate::core::scene::Scene;
use crate::core::geometry::normal::{Normal3f};
use std::sync::{RwLock, Arc};
use crate::core::paramset::ParamSet;
use crate::core::sampling::concentric_sample_disk;

#[allow(dead_code)]
#[derive(Default)]
pub struct DistantLight {
    L                : Spectrum,
    wlight           : Vector3f,
    world_center     : RwLock<Point3f>,
    world_radius     : RwLock<Float>,
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
        let wr = *self.world_radius.read().unwrap();
        self.L * PI * wr * wr
    }

    fn preprocess(&self, scene: &Scene) {
        let (p, f) = scene.wb.bounding_sphere();
        let mut wc = self.world_center.write().unwrap();
        let mut wr = self.world_radius.write().unwrap();
        *wc = p;
        *wr = f;
    }

    fn pdf_li(&self, _re: &InteractionData, _wi: &Vector3f) -> Float {
        0.0
    }

    fn sample_li(
        &self, re: &InteractionData, _u: &Point2f, wi: &mut Vector3f,
        pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        *wi = self.wlight;
        *pdf = 1.0;
        let wr = *self.world_radius.read().unwrap();
        let poutside = re.p + self.wlight * (2.0 * wr);

        let s2 = InteractionData {
            p               : poutside,
            time            : re.time,
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(re.clone(), s2);

        self.L
    }

    fn sample_le(
        &self, u1: &Point2f, _u2: &Point2f, time: Float, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        // TODO ProfilePhase
        let wc = self.world_center.read().unwrap();
        let wr = self.world_radius.read().unwrap();

        // Choose point on disk oriented toward infinite light direction
        let mut v1 = Vector3f::default();
        let mut v2 = Vector3f::default();
        vec3_coordinate_system(&self.wlight, &mut v1, &mut v2);
        let cd = concentric_sample_disk(u1);
        let pdisk = *wc + (v1 * cd.x + v2 * cd.y) * *wr;

        // Set ray origin and direction for infinite light ray
        *ray = Ray::new(
            &(pdisk + self.wlight * *wr), &(-self.wlight), INFINITY, time, None, None);
        *nlight = Normal3f::from(ray.d);
        *pdf_pos = 1.0 / (PI * *wr * *wr);
        *pdf_dir = 1.0;

        self.L
    }

    fn pdf_le(&self, _ray: &Ray, _nlight: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // TODO ProfilePhase
        let wr = self.world_radius.read().unwrap();

        *pdf_pos = 1.0 / (PI * *wr * *wr);
        *pdf_dir = 0.0;
    }

    fn nsamples(&self) -> usize { self.nsamples }

    fn flags(&self) -> u8 {
        self.flags
    }
}

pub fn create_distantlight(l2w: &Transform, params: &ParamSet) -> Option<Arc<Lights>> {
    let L = params.find_one_spectrum("L", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("scale", Spectrum::new(1.0));
    let from = params.find_one_point3f("from", Default::default());
    let to = params.find_one_point3f("to", Point3f::new(0.0, 0.0, 1.0));
    let dir = from - to;
    let l = DistantLight::new(l2w, &(L * sc), &dir);

    Some(Arc::new(l.into()))
}