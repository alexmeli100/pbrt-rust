use crate::core::pbrt::Float;
use std::sync::Arc;
use crate::core::lightdistrib::{LightDistributions, create_light_sample_distribution};
use crate::core::geometry::bounds::Bounds2i;
use crate::core::sampler::{Samplers, Sampler};
use crate::core::camera::{Cameras, Camera};
use crate::core::integrator::{SamplerIntegrator, Integrator, uniform_sample_onelight, Integrators};
use log::error;
use crate::core::bssrdf::BSSRDF;
use crate::core::geometry::ray::Ray;
use crate::core::scene::Scene;
use crate::core::spectrum::Spectrum;
use crate::core::medium::{Medium, PhaseFunction};
use crate::core::interaction::{SurfaceInteraction, MediumInteraction, Interactions, Interaction};
use crate::{stat_int_distribution, stat_counter};
use crate::core::light::Light;
use crate::core::geometry::vector::Vector3f;
use crate::core::material::TransportMode;
use crate::core::lightdistrib::LightDistribution;
use crate::core::reflection::BxDFType;
use bumpalo_herd::Member;
use crate::core::paramset::ParamSet;
use crate::core::geometry::point::Point2i;

stat_int_distribution!("Integrator/Path lenght", path_length);
stat_counter!("Integrator/Volume interactions", volume_interactions);
stat_counter!("Integrator/Surface interactions", surface_interactions);

pub fn init_stats() {
    path_length::init();
    volume_interactions::init();
    surface_interactions::init();
}

pub struct VolPathIntegrator {
    camera              : Arc<Cameras>,
    sampler             : Box<Samplers>,
    bounds              : Bounds2i,
    max_depth           : usize,
    rr_threshold        : Float,
    strategy            : String,
    light_distrib       : Option<Arc<LightDistributions>>
}

impl VolPathIntegrator {
    pub fn new(
        max_depth: usize, camera: Arc<Cameras>, sampler: Box<Samplers>,
        bounds: Bounds2i, rr_threshold: Float, strategy: String) -> Self {
        Self {
            camera, sampler, bounds, max_depth,
            rr_threshold, strategy,
            light_distrib: None
        }
    }
}

impl Integrator for VolPathIntegrator {
    fn render(&mut self, scene: &Scene) {
        SamplerIntegrator::render(self, scene)
    }
}

impl SamplerIntegrator for VolPathIntegrator {
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
        self.light_distrib =
            create_light_sample_distribution(&self.strategy, scene)

    }

    fn li(
        &self, r: &mut Ray, scene: &Scene, sampler: &mut Samplers,
        arena: &Member, _depth: usize) -> Spectrum {
        // TODO: ProfilePhase
        let distrib = self.light_distrib.as_ref().unwrap();
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
            // Intersect ray with scene and store intersection isect
            let mut isect = SurfaceInteraction::default();
            let found = scene.intersect(&mut ray, &mut isect);

            // Sample the perticipating medium if present
            let mut mi = MediumInteraction::default();
            if let Some(ref med) = ray.medium {
                beta *= med.sample(&ray, sampler, &mut mi);
            }
            if beta.is_black() { break; }

            // Handle an interaction with a medium or a surface
            if mi.isvalid() {
                // Terminate path if ray escaped or max_depth was reached
                if bounces >= self.max_depth { break; }

                volume_interactions::inc();
                // Handle scattering at point in medium for volumetric path tracer
                let d = distrib.lookup(&mi.p);
                let i: Interactions = mi.into();
                L += beta * uniform_sample_onelight(
                    &i, scene, arena, sampler,
                    true, Some(&d));

                let wo = -ray.d;
                let mut wi = Vector3f::default();
                let mi = i.get_mediuminteraction();
                mi.phase.as_ref().unwrap().sample_p(&wo, &mut wi, &sampler.get_2d());
                ray = mi.spawn_ray(&wi);
                specular_bounce = false;
            } else {
                surface_interactions::inc();
                // Handle scattering at point on surface for volumetric path tracer

                // Possibly add emitted light at intersection
                if bounces == 0 || specular_bounce {
                    // Add emitted light at intersection
                    if found {
                        L += beta * isect.le(&(-ray.d));
                    } else {
                        for light in scene.infinite_lights.iter() {
                            L += beta * light.le(&ray)
                        }
                    }
                }

                // Terminate path if ray escaped or max_depth was reached
                if !found || bounces >= self.max_depth { break; }

                // Compute scattering functions and skip over medium boundaries
                isect.compute_scattering_functions(&ray, arena, true, TransportMode::Radiance);
                if isect.bsdf.is_none() {
                    ray = isect.spawn_ray(&ray.d);
                    bounces -= 1;
                    continue;
                }

                // Sample illumination from lights to find attenuated path contribution
                let d = distrib.lookup(&isect.p);
                let i: Interactions = isect.into();
                L += beta * uniform_sample_onelight(&i, scene, arena, sampler, true, Some(&d));

                // Sample BSDF to get new path direction
                let wo = -ray.d;
                let mut wi = Vector3f::default();
                let mut pdf = 0.0;
                let mut flags = 0;
                let ty = BxDFType::All as u8;
                isect = i.get_surfaceinteraction();
                let bsdf = isect.bsdf.as_ref().unwrap();
                let f = bsdf.sample_f(&wo, &mut wi, &(sampler.get_2d()), &mut pdf, ty, &mut flags);

                if f.is_black() || pdf == 0.0 { break; }
                beta *= f * wi.abs_dot_norm(&isect.shading.n) / pdf;
                assert!(!(beta.y().is_infinite()));
                specular_bounce = (flags & BxDFType::Specular as u8) != 0;

                if (flags & BxDFType::Specular as u8) != 0 && (flags & BxDFType::Transmission as u8) != 0 {
                    let eta = bsdf.eta;
                    // Update the term that tracks radiance scaling for refraction
                    // depending on whether the ray is entering or leaving the
                    // medium.
                    etascale *= if (wo.dot_norm(&isect.n)) > 0.0 { eta * eta } else { 1.0 / (eta * eta) };
                }
                ray = isect.spawn_ray(&wi);

                // Account for attenuated subsurface scattering if applicable;
                if isect.bssrdf.is_some() && (flags & BxDFType::Transmission as u8) != 0 {
                    // Importance sample the BSSRDF
                    let mut pi = SurfaceInteraction::default();
                    let bssrdf = isect.bssrdf.as_ref().unwrap();
                    let S = bssrdf.sample_s(scene, sampler.get_1d(), &(sampler.get_2d()), arena, &mut pi, &mut pdf);
                    assert!(!(beta.y().is_infinite()));
                    if S.is_black() || pdf == 0.0 { break; }
                    beta *= S / pdf;

                    // Account for the attenuated direct subsurface scattering component
                    let dis = distrib.lookup(&pi.p);
                    let i: Interactions = pi.into();

                    L += beta * uniform_sample_onelight(&i, scene, arena, sampler, true, Some(&dis));

                    // Account for the indirect subsurface scattering component
                    pi = i.get_surfaceinteraction();
                    let pibsdf = pi.bsdf.as_ref().unwrap();
                    let f = pibsdf.sample_f(&pi.wo, &mut wi, &sampler.get_2d(), &mut pdf, ty, &mut flags);
                    if f.is_black() || pdf == 0.0 { break };
                    beta *= f * wi.abs_dot_norm(&pi.shading.n) / pdf;
                    assert!(!(beta.y().is_infinite()));
                    specular_bounce = (flags & BxDFType::Specular as u8) != 0;
                    ray = pi.spawn_ray(&wi);
                }
            }

            // Possibly terminate the path with Russian roulette
            // Factor out radiance scaling due to refraction in rrBeta.
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

pub fn create_volpath_integrator(
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

    Some(VolPathIntegrator::new(
        maxdepth, camera, sampler, pbounds, rr_threshold, lstrategy).into())
}