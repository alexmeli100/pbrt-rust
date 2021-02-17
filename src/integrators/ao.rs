use crate::core::camera::{Cameras, Camera};
use std::sync::Arc;
use crate::core::sampler::{Samplers, Sampler};
use crate::core::geometry::bounds::Bounds2i;
use log::{warn, error};
use crate::core::integrator::{SamplerIntegrator, Integrator, Integrators};
use crate::core::geometry::ray::Ray;
use crate::core::scene::Scene;
use bumpalo_herd::Member;
use crate::core::spectrum::Spectrum;
use crate::core::interaction::{SurfaceInteraction, Interaction};
use crate::core::geometry::vector::Vector3f;
use crate::core::sampling::{cosine_sample_hemisphere, cosine_hemisphere_pdf, uniform_sample_sphere, uniform_sphere_pdf};
use crate::core::pbrt::{Float, Options};
use crate::core::material::TransportMode;
use crate::core::paramset::ParamSet;
use crate::core::geometry::point::Point2i;

pub struct AOIntegrator {
    camera      : Arc<Cameras>,
    sampler     : Box<Samplers>,
    bounds      : Bounds2i,
    cos_sample  : bool,
    nsamples    : usize
}

impl AOIntegrator {
    pub fn new(
        cos_sample: bool, ns: usize, camera: Arc<Cameras>,
        sampler: Box<Samplers>, bounds: Bounds2i) -> Self {
        let nsamples = sampler.round_count(ns);

        if ns != nsamples {
            warn!("Taking {} samples, not {} as specified", nsamples, ns);
        }

        Self {
            camera, sampler, nsamples,
            cos_sample, bounds
        }
    }
}

impl Integrator for AOIntegrator {
    fn render(&mut self, scene: &Scene) {
        SamplerIntegrator::render(self, scene)
    }
}

impl SamplerIntegrator for AOIntegrator {
    fn camera(&self) -> Arc<Cameras> {
        self.camera.clone()
    }

    fn sampler(&self) -> &Samplers {
        &self.sampler
    }

    fn bounds(&self) -> Bounds2i {
        self.bounds
    }

    fn preprocess(&mut self, _scene: &Scene) {
        self.sampler.request_2d_array(self.nsamples)
    }

    fn li(
        &self, r: &mut Ray, scene: &Scene,
        sampler: &mut Samplers, arena: &Member,
        _depth: usize) -> Spectrum {
        // TODO: ProfilePhase
        let mut L = Spectrum::new(0.0);
        let mut ray = r.clone();

        // Intersect ray with scene and store intersection in isect;
        let mut isect = SurfaceInteraction::default();

        if scene.intersect(&mut ray, &mut isect) {
            isect.compute_scattering_functions(&ray, arena, true, TransportMode::Radiance);

            // Compute coordinate frame based on true geometry, not shading geometry
            let n = isect.n.face_foward_vec(&(-ray.d));
            let s = isect.dpdu.normalize();
            let t = isect.n.cross_vec(&s);

            let uopt = sampler.get_2d_array(self.nsamples);

            if let Some(u) = uopt {
                for p in u.iter().take(self.nsamples) {
                    let mut wi: Vector3f;
                    let pdf = if self.cos_sample {
                        wi = cosine_sample_hemisphere(&p);
                        cosine_hemisphere_pdf(wi.z.abs())
                    } else {
                        wi = uniform_sample_sphere(&p);
                        uniform_sphere_pdf()
                    };

                    // Transform wi from local frame to world space
                    wi = Vector3f::new(
                        s.x * wi.x + t.x * wi.y + n.x * wi.z,
                        s.y * wi.x + t.y * wi.y + n.y * wi.z,
                        s.z * wi.x + t.z * wi.y + n.z * wi.z);

                    if !scene.intersect_p(&mut isect.spawn_ray(&wi)) {
                        L += Spectrum::new(wi.dot_norm(&n) / (pdf * self.nsamples as Float));
                    }
                }
            }
        }

        L
    }
}

pub fn create_ao_integrator(
    params: &ParamSet, sampler: Box<Samplers>, camera: Arc<Cameras>,
    opts: &Options) -> Option<Integrators> {
    let mut np = 0;
    let pb = params.find_int("pixelbounds", &mut np);
    let mut pbounds = camera.film().get_sample_bounds();

    if let Some(p) = pb {
        if p.len() != 4 {
            error!("Expected four values for \"pixelbounds\" parameter. Got {}.", np);
        } else {
            let b = Bounds2i::from_points(
                &Point2i::new(p[0], p[2]),
                &Point2i::new(p[1], p[3]));
            pbounds = b.intersect(&pbounds);

            if pbounds.area() == 0 {
                error!("Degenerate \"pixelbounds\" specified.");
            }
        }
    }

    let cossample = params.find_one_bool("cossample", true);
    let mut nsamples = params.find_one_int("nsamples", 64) as usize;

    if opts.quick_render { nsamples = 1; }

    Some(AOIntegrator::new(cossample, nsamples, camera, sampler, pbounds).into())
}