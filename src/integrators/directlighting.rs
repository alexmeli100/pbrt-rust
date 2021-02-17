use crate::core::camera::{Cameras, Camera};
use crate::core::sampler::{Samplers, Sampler};
use crate::core::light::Light;
use crate::core::geometry::bounds::Bounds2i;
use crate::core::integrator::{SamplerIntegrator, Integrator, uniform_sample_all_lights, uniform_sample_onelight, Integrators};
use crate::core::geometry::ray::Ray;
use crate::core::scene::Scene;
use crate::core::spectrum::Spectrum;
use std::sync::Arc;
use crate::core::interaction::{SurfaceInteraction, Interaction, Interactions};
use crate::core::material::TransportMode;
use bumpalo_herd::Member;
use crate::core::paramset::ParamSet;
use crate::core::geometry::point::Point2i;
use log::{error, warn};

pub enum LightStrategy { UniformSampleAll, UniformSampleOne }

pub struct DirectLightingIntegrator {
    // SamplerIntegrator Data
    camera          : Arc<Cameras>,
    sampler         : Box<Samplers>,
    bounds          : Bounds2i,
    stategy         : LightStrategy,
    max_depth       : usize,
    nlight_samples  : Vec<usize>
}

impl DirectLightingIntegrator {
    pub fn new(
        stategy: LightStrategy, max_depth: usize, camera: Arc<Cameras>,
        sampler: Box<Samplers>, bounds: &Bounds2i) -> Self {
        Self {
            stategy, max_depth,
            camera, sampler,
            bounds: *bounds,
            nlight_samples: Vec::new()
        }
    }
}

impl Integrator for DirectLightingIntegrator {
    fn render(&mut self, scene: &Scene) {
        SamplerIntegrator::render(self, scene);
    }
}

impl SamplerIntegrator for DirectLightingIntegrator {
    fn camera(&self) -> Arc<Cameras> {
        self.camera.clone()
    }

    fn sampler(&self) -> &Samplers {
        &self.sampler
    }

    fn bounds(&self) -> Bounds2i {
        self.bounds
    }

    fn preprocess(&mut self, scene: &Scene) {
        if let LightStrategy::UniformSampleAll = self.stategy {
            // Compute number of samplers to use for each light
            for light in scene.lights.iter() {
                self.nlight_samples.push(self.sampler.round_count(light.nsamples()));
            }

            // Request samples for sampling all lights
            for _i in 0..self.max_depth {
                for j in 0..scene.lights.len() {
                    self.sampler.request_2d_array(self.nlight_samples[j]);
                    self.sampler.request_2d_array(self.nlight_samples[j]);
                }
            }
        }
    }

    fn li(&self, r: &mut Ray, scene: &Scene, sampler: &mut Samplers, arena: &Member, depth: usize) -> Spectrum {
        // TODO: ProfilePhase
        let mut L = Spectrum::new(0.0);
        // Find closest ray intersection or return background radiance
        let mut isect = SurfaceInteraction::default();
        if !scene.intersect(r, &mut isect) {
            for light in scene.lights.iter() { L += light.le(r); }
            return L;
        }

        // Compute scattering functions for surface interaction
        isect.compute_scattering_functions(r, arena, false, TransportMode::Radiance);
        if isect.bsdf.is_none() {
            let mut ray = isect.spawn_ray(&r.d);
            return self.li(&mut ray, scene, sampler, arena, depth);
        }

        let wo = isect.wo;

        // Compute emitted light if ray hit an area light source
        L += isect.le(&wo);
        let intr: Interactions = isect.into();
        if scene.lights.len() > 0 {
            // Compute direct lighting for DirectLightingIntegrator integrator
            let v = match self.stategy {
                LightStrategy::UniformSampleAll => uniform_sample_all_lights(&intr, scene, arena, sampler, &self.nlight_samples, false),
                _                               => uniform_sample_onelight(&intr, scene, arena, sampler, false, None)
            };

            L += v;
        }

        if depth + 1 < self.max_depth {
            // Trace rays for specular reflection and refraction

            if let Interactions::SurfaceInteraction(ref s) = intr {
                L += self.specular_reflect(r, s, scene, sampler, arena, depth);
                L += self.specular_transmit(r, s, scene, sampler, arena, depth);
            }
        }

        L

    }
}

pub fn create_directlighting_integrator(
    params: &ParamSet, sampler: Box<Samplers>,
    camera: Arc<Cameras>) -> Option<Integrators> {
    let maxdepth = params.find_one_int("maxdepth", 5) as usize;let mut np = 0;
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

    let strategy = match params.find_one_string("strategy", "all".to_string()).as_str() {
        "one" => LightStrategy::UniformSampleOne,
        "all" => LightStrategy::UniformSampleAll,
        s     => {
            warn!("Strategy \"{}\" for direct lighting unknown. Using \"all\".", s);
            LightStrategy::UniformSampleAll
        }
    };

    Some(DirectLightingIntegrator::new(strategy, maxdepth, camera, sampler, &pbounds).into())

}