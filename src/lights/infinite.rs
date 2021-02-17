use crate::core::mipmap::MIPMap;
use crate::core::spectrum::{RGBSpectrum, Spectrum, SpectrumType};
use crate::core::geometry::point::{Point2i, Point3f, Point2f};
use crate::core::pbrt::{Float, PI, INV2_PI, INV_PI, Options, INFINITY};
use crate::core::sampling::{Distribution2D, concentric_sample_disk};
use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::light::{LightFlags, Light, VisibilityTester, Lights};
use crate::init_light_data;
use crate::mipmap;
use rayon::prelude::*;
use crate::core::imageio::read_image;
use crate::core::geometry::vector::{Vector3f, vec3_coordinate_system};
use crate::core::interaction::InteractionData;
use crate::core::geometry::ray::Ray;
use crate::core::scene::Scene;
use crate::core::geometry::normal::Normal3f;
use crate::core::geometry::geometry::{spherical_theta, spherical_phi};
use std::sync::{RwLock, Arc};
use crate::core::paramset::ParamSet;

pub struct InfiniteAreaLight {
    map              : MIPMap<RGBSpectrum>,
    world_centre     : RwLock<Point3f>,
    world_radius     : RwLock<Float>,
    distribution     : Distribution2D,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl InfiniteAreaLight {
    pub fn new(l2w: &Transform, power: &Spectrum, nsamples: usize, map: &str) -> Self {
        macro_rules! no_texels {
            () => {
                (vec![RGBSpectrum::from(*power); 1], Point2i::new(1, 1))
            }
        }

        // Read data from texmap and initialize map
        let rgb = RGBSpectrum::from(*power);
        let (s, res) = if !map.is_empty() {
            match read_image(map) {
                Ok((mut s, r)) => {
                    for i in 0..r.x * r.y {
                        s[i as usize] *= rgb;
                    }

                    (s, r)
                },
                _ => no_texels!()
            }
        } else {
            no_texels!()
        };



        let map = mipmap!(&res, s);

        // TODO: Initialize distribution
        let width = 2 * map.width();
        let height = 2 * map.height();
        let mut img = vec![0.0; width * height];
        let fwidth = 0.5 / std::cmp::min(width, height) as Float;

        img
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, val)| {
                let v = i / width;
                let u = i % width;
                let vp = (v as Float + 0.5) / height as Float;
                let sin_theta = (PI * (v as Float + 0.5) / height as Float).sin();
                let up = (u as Float + 0.5) / width as Float;
                *val = map.lookup(&Point2f::new(up, vp), fwidth).y();
                *val *= sin_theta;
            });

        let distribution = Distribution2D::new(&img, width, height);

        let mut il = InfiniteAreaLight{
            map,
            nsamples,
            distribution,
            world_radius    : RwLock::new(0.0),
            world_centre    : RwLock::new(Default::default()),
            flags           : Default::default(),
            medium_interface: Default::default(),
            light_to_world  : Default::default(),
            world_to_light  : Default::default()
        };

        // Initalize Light data
        let flags = LightFlags::DeltaPosition as u8;
        init_light_data!(il, flags, nsamples, MediumInterface::default(), l2w);

        il
    }
}

impl Light for InfiniteAreaLight {
    fn power(&self) -> Spectrum {
        let v = self.map.lookup(&Point2f::new(0.5, 0.5), 0.5);
        let s = Spectrum::from_rgb_spectrum(&v, SpectrumType::Illuminant);
        let wr = *self.world_radius.read().unwrap();

        s * wr * wr * PI
    }

    fn preprocess(&self, scene: &Scene) {
        let (c, rad) =  scene.wb.bounding_sphere();
        let mut wc = self.world_centre.write().unwrap();
        *wc = c;
        let mut wr = self.world_radius.write().unwrap();
        *wr = rad;
    }

    fn le(&self, r: &Ray) -> Spectrum {
        let w = self.world_to_light.transform_vector(&r.d).normalize();
        let st = Point2f::new(
            spherical_phi(&w) * INV2_PI,
            spherical_theta(&w) * INV_PI);

        Spectrum::from_rgb_spectrum(&self.map.lookup(&st, 0.0), SpectrumType::Illuminant)
    }

    fn pdf_li(&self, _re: &InteractionData, w: &Vector3f) -> f32 {
        // TODO: ProfilePhase
        let wi = self.world_to_light.transform_vector(w);
        let theta = spherical_theta(&wi);
        let phi = spherical_phi(&wi);
        let sin_theta = theta.sin();
        if sin_theta == 0.0 { return 0.0 }
        let p = Point2f::new(phi * INV2_PI, theta * INV_PI);

        self.distribution.pdf(&p) / (2.0 * PI * PI * sin_theta)
    }

    fn sample_li(
        &self, re: &InteractionData, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        // Find (u, v) sample coordinates in infinite light texture
        let mut map_pdf = 0.0;
        let uv = self.distribution.sample_continuous(u, &mut map_pdf);
        if map_pdf == 0.0 { return Spectrum::new(0.0); }

        // Convert infinite light sample point to direction
        let theta = uv[1] * PI;
        let phi = uv[0] * 2.0 * PI;
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        let v = Vector3f::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
        *wi = self.light_to_world.transform_vector(&v);

        // Compute PDF for sampled infinite light distribution
        *pdf = map_pdf / (2.0 * PI * PI * sin_theta);
        if sin_theta == 0.0 { *pdf = 0.0; }

        // Return radiance value for infinite light direction
        let wr = *self.world_radius.read().unwrap();
        let intr = InteractionData {
            p: re.p + *wi * ( 2.0 * wr),
            time: re.time,
            medium_interface: Some(self.medium_interface.clone()),
            ..Default::default()
        };

        *vis = VisibilityTester::new(re.clone(), intr);
        let l = self.map.lookup(&uv, 0.0);

        Spectrum::from_rgb_spectrum(&l, SpectrumType::Illuminant)
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: f32, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        // TODO ProfilePhase
        // Compute direction for infinite light sample ray;
        let u = u1;
        let wc = self.world_centre.read().unwrap();
        let wr = self.world_radius.read().unwrap();

        // Find (u, v) sample coordinates in infinite light texture
        let mut mappdf = 0.0;
        let uv = self.distribution.sample_continuous(u, &mut mappdf);
        if mappdf == 0.0 { return Spectrum::new(0.0); }
        let theta = uv[1] * PI;
        let phi = uv[0] * 2.0 * PI;
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        let v = Vector3f::new(sin_theta * cos_phi, sin_theta * sin_phi, cos_theta);
        let d = -(self.light_to_world.transform_vector(&v));
        *nlight = Normal3f::from(d);

        // Compute origin for infinite light sample ray
        let mut v1 = Vector3f::default();
        let mut v2 = Vector3f::default();
        vec3_coordinate_system(&(-d), &mut v1, &mut v2);
        let cd = concentric_sample_disk(u2);
        let pdisk = *wc + (v1 * cd.x + v2 * cd.y) * *wr;
        *ray = Ray::new(&(pdisk + -d * *wr), &d, INFINITY, time, None, None );

        // Compute InfiniteAreaLight ray PDFs
        *pdf_dir = if sin_theta == 0.0 {
            0.0
        } else {
            mappdf / (2.0 / PI * PI * sin_theta)
        };
        *pdf_pos = 1.0 / (PI * *wr * *wr);

        Spectrum::from_rgb_spectrum(&self.map.lookup(&uv, 0.0), SpectrumType::Illuminant)

    }

    fn pdf_le(&self, ray: &Ray, _nlight: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // TODO: ProfilePhase
        let d = -(self.world_to_light.transform_vector(&ray.d));
        let theta = spherical_theta(&d);
        let phi = spherical_phi(&d);
        let uv = Point2f::new(phi * INV2_PI, theta * INV_PI);
        let mappdf = self.distribution.pdf(&uv);
        let wr = self.world_radius.read().unwrap();

        *pdf_dir = mappdf / (2.0 * PI * PI * theta.sin());
        *pdf_pos = 1.0 / (PI * *wr * *wr);
    }

    fn nsamples(&self) -> usize { self.nsamples }

    fn flags(&self) -> u8 {
        self.flags
    }
}

pub fn create_infinitelight(
    l2w: &Transform, params: &ParamSet,
    opts: &Options) -> Option<Arc<Lights>> {
    let L = params.find_one_spectrum("L", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("scale", Spectrum::new(1.0));
    let texmap = params.find_one_filename("mapname", "");
    let ns = params.find_one_int("nsamples", 1);
    let mut nsamples = params.find_one_int("samples", ns) as usize;

    if opts.quick_render {
        nsamples = std::cmp::max(1, nsamples / 4);
    }

    let l = InfiniteAreaLight::new(l2w, &(L * sc), nsamples, &texmap);

    Some(Arc::new(l.into()))
}