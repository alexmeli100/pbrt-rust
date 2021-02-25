use crate::core::pbrt::Float;
use crate::core::lightdistrib::{LightDistributions, create_light_sample_distribution, LightDistribution};
use crate::core::camera::{Cameras, Camera};
use std::sync::Arc;
use crate::core::sampler::Samplers;
use crate::core::geometry::bounds::Bounds2i;
use crate::core::integrator::{Integrator, SamplerIntegrator, uniform_sample_onelight, Integrators};
use crate::core::scene::Scene;
use crate::core::geometry::ray::Ray;
use crate::core::spectrum::Spectrum;
use log::{debug, error};
use crate::core::light::Light;
use crate::core::interaction::{SurfaceInteraction, Interaction, Interactions};
use crate::core::material::TransportMode;
use crate::core::reflection::BxDFType;
use crate::{stat_percent, stat_int_distribution};
use crate::core::geometry::vector::Vector3f;
use crate::core::sampler::Sampler;
use crate::core::bssrdf::BSSRDF;
use bumpalo_herd::Member;
use crate::core::paramset::ParamSet;
use crate::core::geometry::point::Point2i;

stat_percent!("Integrator/Zero-radiance paths", zeroradiance_total);
stat_int_distribution!("Integrator/Path length", path_length);

pub fn init_stats() {
    zeroradiance_total::init();
    path_length::init();
}

pub struct PathIntegrator {
    camera                  : Arc<Cameras>,
    sampler                 : Box<Samplers>,
    bounds                  : Bounds2i,
    max_depth               : usize,
    rr_threshold            : Float,
    light_sample_strategy   : String,
    light_distribution      : Option<Arc<LightDistributions>>
}

impl PathIntegrator {
    pub fn new(
        max_depth: usize, camera: Arc<Cameras>,
        sampler: Box<Samplers>, bounds: Bounds2i,
        rr_threshold: Float, light_sample_strategy: String) -> Self {
        Self {
            max_depth, camera, sampler, bounds,
            rr_threshold, light_sample_strategy,
            light_distribution: None
        }
    }
}

impl Integrator for PathIntegrator {
    fn render(&mut self, scene: &Scene) {
        SamplerIntegrator::render(self, scene)
    }
}

impl SamplerIntegrator for PathIntegrator {
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
        self.light_distribution = create_light_sample_distribution(
            &self.light_sample_strategy, scene);
    }

    fn li(&self, r: &mut Ray, scene: &Scene, sampler: &mut Samplers, arena: &Member, _depth: usize) -> Spectrum {
        // Todo: ProfilePhase
        let light_distrib = self.light_distribution.as_ref().unwrap();
        let mut L = Spectrum::new(0.0);
        let mut beta = Spectrum::new(1.0);
        let mut ray = r.clone();
        let mut specular_bounce = false;
        let mut bounces = 0;
        // Added after book publication: etaScale tracks the accumulated effect
        // of radiance scaling due to rays passing through refractive
        // boundaries (see the derivation on p. 527 of the third edition). We
        // track this value in order to remove it from beta when we apply
        // Russian roulette; this is worthwhile, since it lets us sometimes
        // avoid terminating refracted rays that are about to be refracted back
        // out of a medium and thus have their beta value increased.
        let mut etascale = 1.0;


        loop {
            // Find next path vertex and accumulate contribution
            debug!("Path tracer bounce {}, current L = {}, beta = {}", bounces, L, beta);

            // Intersect ray with scene and store intersection isect
            let mut isect = SurfaceInteraction::default();
            let found_isect = scene.intersect(&mut ray, &mut isect);

            // Possibly add emitted light at intersection
            if bounces == 0 || specular_bounce {
                // Add emitted light at path vertex or from the environment
                if found_isect {
                    L += isect.le(&(-ray.d)) * beta;
                    debug!("Added Le -> L = {}", L);
                } else {
                    for light in scene.infinite_lights.iter() {
                        L += light.le(&ray);
                    }
                    debug!("Added infinite area lights -> L = {}", L);
                }
            }

            // Terminate path if ray escaped or maxDepth was reached
            if !found_isect || bounces >= self.max_depth { break; }

            // Compute scattering function and skip over media boundaries
            isect.compute_scattering_functions(&ray, arena, true, TransportMode::Radiance);
            if isect.bsdf.is_none() {
                debug!("Skipping intersection due to null bsdf");
                ray = isect.spawn_ray(&ray.d);
                bounces -= 1;
                continue
            }

            let mut bsdf = isect.bsdf.as_ref().unwrap();
            let distrib = light_distrib.lookup(&isect.p);

            // Sample illumination from lights to find path contribution.
            // (But skip this for perfectly specular BSDFs.)
            let nc = bsdf.num_components(BxDFType::All as u8 & !(BxDFType::Specular as u8));
            let i: Interactions = isect.into();
            if nc > 0 {
                zeroradiance_total::inc_den();
                let Ld = uniform_sample_onelight(&i, scene, arena, sampler, false, Some(&distrib));
                debug!("Sampled direct lighting Ld = {}", Ld);
                if Ld.is_black() { zeroradiance_total::inc_num(); }
                assert!(Ld.y() >= 0.0);
                L += Ld;
            }

            // Sample BSDF to get new path direction
            let wo = -ray.d;
            let mut wi = Vector3f::default();
            let mut pdf = 0.0;
            let mut flags = 0u8;
            isect = i.get_surfaceinteraction();
            bsdf = isect.bsdf.as_ref().unwrap();
            let ty = BxDFType::All as u8;
            let f = bsdf.sample_f(&wo, &mut wi, &sampler.get_2d(), &mut pdf, ty, &mut flags);
            debug!("Sampled BSDF, f = {}, pdf = {}", f, pdf);

            if f.is_black() || pdf == 0.0 { break; }
            beta *= f * wi.abs_dot_norm(&isect.shading.n) / pdf;
            debug!("Updated beta = {}", beta);
            assert!(beta.y() >= 0.0);
            assert!(!(beta.y().is_infinite()));
            specular_bounce = (flags & BxDFType::Specular as u8) != 0;

            if (flags & BxDFType::Specular as u8) != 0 && (flags & BxDFType::Transmission as u8) != 0 {
                let eta = bsdf.eta;
                // Update the term that tracks radiance scaling for refraction
                // depending on whether the ray is entering or leaving the
                // medium.
                etascale *= if wo.dot_norm(&isect.n) > 0.0 { eta * eta } else { 1.0 / (eta * eta) };
            }

            ray = isect.spawn_ray(&wi);

            // Account for subsurface scattering if applicable
            if isect.bssrdf.is_some() && (flags & BxDFType::Transmission as u8) != 0 {
                // Importance sample the BSSRDF
                let mut pi = SurfaceInteraction::default();
                let bssrdf = isect.bssrdf.as_ref().unwrap();
                let S = bssrdf.sample_s(scene, sampler.get_1d(), &sampler.get_2d(), arena, &mut pi, &mut pdf);
                assert!(!(beta.y().is_infinite()));
                if S.is_black() || pdf == 0.0 { break; }
                beta *= S / pdf;

                // Account for direct subsurface scattering component
                let d = light_distrib.lookup(&pi.p);
                let ip: Interactions = pi.into();

                L += beta * uniform_sample_onelight(&ip, scene, arena, sampler, false, Some(&d));

                pi = ip.get_surfaceinteraction();
                let pibsdf = pi.bsdf.as_ref().unwrap();
                // Account for the indirect subsurface scattering component
                let ty = BxDFType::All as u8;
                let ff = pibsdf.sample_f(&pi.wo, &mut wi, &sampler.get_2d(), &mut pdf, ty, &mut flags);
                if ff.is_black() || pdf == 0.0 { break; }
                beta *= ff * wi.abs_dot_norm(&pi.shading.n) / pdf;
                assert!(!(beta.y().is_infinite()));
                specular_bounce = (flags & BxDFType::Specular as u8) != 0;
                ray = pi.spawn_ray(&wi);
            }

            // Possibly terminate the path with Russian roulette.
            // Factor out radiance scaling due to refraction to rrBeta.
            let rrbeta = beta * etascale;
            if rrbeta.max_component_value() < self.rr_threshold && bounces > 3 {
                let q = (1.0 - rrbeta.max_component_value()).max(0.05);
                if sampler.get_1d() < q { break; }
                beta /= 1.0 - q;
                assert!(!(beta.y().is_infinite()));
            }

            bounces += 1;
        }

        path_length::report_value(bounces as u64);

        L
    }
}

pub fn create_path_integrator(
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

    let rr_threshold = params.find_one_float("rrthreshold", 1.0);
    let lstrategy = params.find_one_string("lightsamplestrategy", "spatial".to_owned());

    Some(PathIntegrator::new(
        maxdepth, camera, sampler, pbounds, rr_threshold, lstrategy).into())
}