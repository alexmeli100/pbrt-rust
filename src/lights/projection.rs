use crate::core::mipmap::MIPMap;
use crate::core::spectrum::{RGBSpectrum, Spectrum, SpectrumType};
use crate::core::geometry::point::{Point3f, Point2i, Point2f};
use crate::core::transform::Transform;
use crate::core::pbrt::{Float, PI};
use crate::core::geometry::bounds::Bounds2f;
use crate::core::medium::MediumInterface;
use crate::init_light_data;
use crate::mipmap;
use crate::core::light::{LightFlags, Light, VisibilityTester};
use crate::core::imageio::read_image;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interactions, SurfaceInteraction, Interaction};
use crate::core::geometry::normal::Normal3f;

#[derive(Default)]
pub struct ProjectionLight {
    projection_map   : Option<MIPMap<RGBSpectrum>>,
    plight           : Point3f,
    I                : Spectrum,
    light_projection : Transform,
    hither           : Float,
    yon              : Float,
    screen_bounds    : Bounds2f,
    cos_totalwidth   : Float,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl ProjectionLight {
    pub fn new(
        l2w: &Transform, mi: MediumInterface,
        I: &Spectrum, texname: &str, fov: Float) -> Self {
        // Initalize Light data
        let flags = LightFlags::DeltaPosition as u8;
        let mut pl = ProjectionLight::default();
        init_light_data!(pl, flags, 1, mi, l2w);

        pl.plight = l2w.transform_point(&Point3f::new(0.0, 0.0, 0.0));
        pl.I = *I;

        // Create ProjectionLight MIP map
        let (texels, res) = read_image(texname).map_or_else(
            |_| (None, Point2i::default()),
            |(v, p)| (Some(v), p));



        let aspect = if let Some(ref _t) = texels {
            res.x as Float / res.y as Float
        } else {
            1.0
        };

        pl.projection_map = texels.map(|t| mipmap!(&res, t));
        pl.screen_bounds = if aspect > 1.0 {
            Bounds2f::new(&Point2f::new(-aspect, -1.0), &Point2f::new(aspect, 1.0))
        } else {
            Bounds2f::new(&Point2f::new(-1.0, -1.0 / aspect), &Point2f::new(1.0, 1.0 / aspect))
        };

        pl.hither = 1.0e-3;
        pl.yon = 1.0e30;
        pl.light_projection = Transform::perspective(fov, pl.hither, pl.yon);

        let stl = Transform::inverse(&pl.light_projection);
        let pcorner = Point3f::new(pl.screen_bounds.p_max.x, pl.screen_bounds.p_max.y, 0.0);
        let wconer = Vector3f::from(stl.transform_point(&pcorner)).normalize();
        pl.cos_totalwidth = wconer.z;

        pl
    }

    fn projection(&self, w: &Vector3f) -> Spectrum {
        let wl = self.world_to_light.transform_vector(w);
        // Discar directions behind projection light
        if wl.z < self.hither { return Spectrum::new(0.0); }

        // Project point ont projection plane and compute light
        let p = self.light_projection.transform_point(&Point3f::new(wl.x, wl.y, wl.z));

        if !self.screen_bounds.inside(&Point2f::new(p.x, p.y)) { return Spectrum::new(0.0); }
        if self.projection_map.is_none() { return Spectrum::new(1.0); }

        let st = Point2f::from(self.screen_bounds.offset(&Point2f::new(p.x, p.y)));
        let l = self.projection_map.as_ref().unwrap().lookup(&st, 0.0);

        Spectrum::from_rgb_spectrum(&l, SpectrumType::Illuminant)
    }
}

impl Light for ProjectionLight {
    fn power(&self) -> Spectrum {
        let s = if let Some(ref m) = self.projection_map {
            let l = m.lookup(&Point2f::new(0.5, 0.5), 0.5);

            Spectrum::from_rgb_spectrum(&l, SpectrumType::Illuminant)
        } else {
            Spectrum::new(1.0)
        };

        s * self.I * 2.0 * PI * (1.0 - self.cos_totalwidth)
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

        self.I * self.projection(&(-*wi)) / self.plight.distance_squared(&re.p())
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: f32, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) -> Spectrum {
        unimplemented!()
    }

    fn pdf_le(&self, ray: &Ray, nlight: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut f32) {
        unimplemented!()
    }
}