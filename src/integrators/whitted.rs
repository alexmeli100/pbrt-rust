use std::sync::Arc;
use crate::core::camera::{Cameras, Camera};
use crate::core::sampler::Samplers;
use crate::core::geometry::bounds::Bounds2i;
use crate::core::integrator::{Integrator, SamplerIntegrator, Integrators};
use crate::core::scene::Scene;
use crate::core::sampler::Sampler;
use crate::core::light::{Light, VisibilityTester};
use crate::core::geometry::ray::Ray;
use crate::core::spectrum::Spectrum;
use crate::core::interaction::{SurfaceInteraction, Interaction};
use crate::core::material::TransportMode;
use crate::core::geometry::vector::Vector3f;
use crate::core::reflection::BxDFType;
use bumpalo_herd::Member;
use crate::core::paramset::ParamSet;
use log::error;
use crate::core::geometry::point::Point2i;

pub struct WhittedIntegrator {
    camera    : Arc<Cameras>,
    sampler   : Box<Samplers>,
    bounds    : Bounds2i,
    max_depth : usize
}

impl WhittedIntegrator {
    pub fn new(
        max_depth: usize, camera: Arc<Cameras>,
        sampler: Box<Samplers>, bounds: Bounds2i) -> Self {
        Self { max_depth, camera, sampler, bounds }
    }
}

impl Integrator for WhittedIntegrator {
    fn render(&mut self, scene: &Scene) {
        SamplerIntegrator::render(self, scene)
    }
}

impl SamplerIntegrator for WhittedIntegrator {
    fn camera(&self) -> Arc<Cameras> {
        self.camera.clone()
    }

    fn sampler(&self) -> &Samplers {
        &self.sampler
    }

    fn bounds(&self) -> Bounds2i {
        self.bounds
    }

    fn li(
        &self, r: &mut Ray, scene: &Scene, sampler: &mut Samplers,
        arena: &Member, depth: usize) -> Spectrum {
        let mut L = Spectrum::new(0.0);
        // Find closes ray intersection or return background radiance
        let mut isect = SurfaceInteraction::default();
        if !scene.intersect(r, &mut isect) {
            for light in scene.lights.iter() {
                L += light.le(r);
            }

            return L;
        }

        // Compute emitted and reflected light at ray intersection point

        // Initialize common variables for Whitted integrator
        let n = isect.n;
        let wo = isect.wo;

        // Compute scattering functions for surface interaction
        isect.compute_scattering_functions(r, arena, false, TransportMode::Radiance);
        if isect.bsdf.is_none() {
            let mut ray = isect.spawn_ray(&r.d);

            return self.li(&mut ray, scene, sampler, arena, depth);
        }

        // Compute emitted light if ray hit an area light source
        L += isect.le(&wo);

        // Add contribution of each light source
        for light in scene.lights.iter() {
            let mut wi = Vector3f::default();
            let mut pdf = 0.0;
            let mut vis = VisibilityTester::default();
            let Li = light.sample_li(&isect.get_data(), &sampler.get_2d(), &mut wi, &mut pdf, &mut vis);
            if Li.is_black() || pdf == 0.0 { continue; }
            let flags = BxDFType::All as u8;
            let f = isect.bsdf.as_ref().unwrap().f(&wo, &wi, flags);

            if !f.is_black() && vis.unoccluded(scene) {
                L += f * Li * wi.abs_dot_norm(&n) / pdf;
            }
        }

        if depth + 1 < self.max_depth {
            // Trace rays for specular reflection and refraction
            L += self.specular_reflect(r, &isect, scene, sampler, arena, depth);
            L += self.specular_transmit(r, &isect, scene, sampler, arena, depth);
        }

        L
    }
}

pub fn create_whitted_integrator(
    params: &ParamSet, sampler: Box<Samplers>,
    camera: Arc<Cameras>) -> Option<Integrators> {
    let maxdepth = params.find_one_int("maxdepth", 5) as usize;
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

    Some(WhittedIntegrator::new(maxdepth, camera, sampler, pbounds).into())
}