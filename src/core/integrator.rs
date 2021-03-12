use enum_dispatch::enum_dispatch;
use crate::core::scene::Scene;
use bumpalo::core_alloc::sync::Arc;
use crossbeam::crossbeam_channel::bounded;
use indicatif::{ProgressBar, ProgressStyle};
use crate::core::camera::{Cameras, Camera};
use crate::core::sampler::{Samplers, Sampler};
use crate::core::geometry::bounds::{Bounds2i, Bounds3i};
use bumpalo_herd::Member;
use crate::core::geometry::ray::{Ray, RayDifferential};
use crate::core::interaction::{SurfaceInteraction, Interactions, Interaction};
use crate::core::spectrum::Spectrum;
use crate::core::film::Film;
use crate::core::geometry::point::{Point2i, Point2f};
use rayon::prelude::*;
use log::{info, error, debug};
use crate::core::pbrt::{Float, set_progress_bar};
use crate::core::light::{Light, is_delta_light};
use crate::stat_counter;
use crate::core::light::{Lights, VisibilityTester};
use crate::core::reflection::BxDFType;
use crate::core::geometry::vector::Vector3f;
use crate::core::medium::PhaseFunction;
use crate::core::sampling::{power_heuristic, Distribution1D};
use crate::core::primitive::Primitive;
use crate::integrators::ao::AOIntegrator;
use crate::integrators::mlt::MLTIntegrator;
use crate::integrators::path::PathIntegrator;
use crate::integrators::bdpt::BDPTIntegrator;
use crate::integrators::whitted::WhittedIntegrator;
use crate::integrators::volpath::VolPathIntegrator;
use crate::integrators::sppm::SPPMIntegrator;
use crate::integrators::directlighting::DirectLightingIntegrator;


stat_counter!("Integrator/Camera rays traced", ncamera_rays);

pub fn init_stats() {
    ncamera_rays::init();
}

pub fn uniform_sample_all_lights(
    it: &Interactions, scene: &Scene, arena: &Member, sampler: &mut Samplers,
    nlight_samples: &[usize], handle_media: bool) -> Spectrum {
    // TODO: ProfilePhase
    let mut L = Spectrum::new(0.0);

    for j in 0..scene.lights.len() {
        // Accumulate contribution of jth light to L
        let light = &scene.lights[j];
        let nsamples = nlight_samples[j];
        let ulight_array = sampler.get_2d_array(nsamples).map(|a| a.to_vec());
        let uscattering_array = sampler.get_2d_array(nsamples).map(|a| a.to_vec());

        if ulight_array.is_none() || uscattering_array.is_none() {
            // Use a single sample for Illumination from light
            let ulight = sampler.get_2d();
            let uscattering = sampler.get_2d();

            L += estimate_direct(
                it, &uscattering, light, &ulight,
                scene, sampler, arena, handle_media, false);
        } else {
            // estimate direct lighting using sample arrays
            let larray = ulight_array.unwrap();
            let sarray = uscattering_array.unwrap();
            let mut Ld = Spectrum::new(0.0);
            for k in 0..nsamples {
                Ld += estimate_direct(
                    it, &sarray[k], light, &larray[k],
                    scene, sampler, arena, handle_media, false);
            }

            L += Ld / nsamples as Float;
        }
    }

    L
}

pub fn uniform_sample_onelight(
    it: &Interactions, scene: &Scene, arena: &Member, sampler: &mut Samplers,
    handle_media: bool, light_distrib: Option<&Distribution1D>) -> Spectrum {
    // TODO: ProfilePahse
    let nlights = scene.lights.len();
    if nlights == 0 { return Spectrum::new(0.0); }

    let lightnum: usize;
    let mut lightpdf = 0.0;

    if let Some(d) = light_distrib {
        lightnum = d.sample_discrete(sampler.get_1d(), Some(&mut lightpdf), None);
        if lightpdf == 0.0 { return Spectrum::new(0.0); }
    }  else {
        lightnum = std::cmp::min((sampler.get_1d() * nlights as Float) as usize, nlights - 1);
        lightpdf = 1.0 / nlights as Float;
    }

    let light = &scene.lights[lightnum];
    let ulight = sampler.get_2d();
    let uscattering = sampler.get_2d();

    estimate_direct(
        it, &uscattering, light, &ulight, scene, sampler,
        arena, handle_media , false) / lightpdf
}


pub fn estimate_direct(
    it: &Interactions, uscatt: &Point2f, light: &Arc<Lights>,
    ulight: &Point2f, scene: &Scene, sampler: &mut Samplers, _arena: &Member,
    handle_media: bool, specular: bool) -> Spectrum {
    let bsdf_flags = if specular {
        BxDFType::All as u8
    } else {
        (BxDFType::All as u8) & !(BxDFType::Specular as u8)
    };
    let mut Ld = Spectrum::new(0.0);

    //Sample light source with multiple importance sampling
    let mut wi = Vector3f::default();
    let mut lightpdf: Float = 0.0;
    let mut scattpdf: Float = 0.0;
    let mut visibility = VisibilityTester::default();
    let mut Li = light.sample_li(&it.get_data(), ulight, &mut wi, &mut lightpdf, &mut visibility);
    debug!(
        "EstimateDirect uLight {} -> Li: {}, wi: {}, pdf {}",
        ulight, Li, wi, lightpdf);

    if lightpdf > 0.0 && !Li.is_black() {
        // Compute BSDF or phase function's value for light sample
        let mut f = Spectrum::default();
        match it {
            Interactions::SurfaceInteraction(ref s) => {
                let bsdf = s.bsdf.as_ref().unwrap();
                f = bsdf.f(&s.wo, &wi, bsdf_flags) *
                    wi.abs_dot_norm(&s.shading.n);
                scattpdf = bsdf.pdf(&s.wo, &wi, bsdf_flags);
                debug!("  surf f*dot : {}, scatteringPdf: {}", f, scattpdf);
            },
            Interactions::MediumInteraction(ref m) => {
                let p = m.phase.as_ref().unwrap().p( &m.wo, &wi);
                f = Spectrum::new(p);
                scattpdf = p;
                debug!("  medium p: {}", p);
            }
            _ => ()
        }

        if !f.is_black() {
            // compute effect of visibility for light source sample
            if handle_media {
                Li *= visibility.tr(scene, sampler);
                debug!("  after Tr, Li: {}", Li);
            } else if !visibility.unoccluded(scene) {
                debug!("  shadow ray blocked");
                Li = Spectrum::new(0.0);
            } else {
                debug!("  shadow ray unoccluded")
            }

            // Add light's contribution to reflected radiance
            if !Li.is_black() {
                Ld += if is_delta_light(light.flags()) {
                    f * Li / lightpdf
                } else {
                    let weight = power_heuristic(1, lightpdf, 1, scattpdf);

                    f * Li * weight / lightpdf
                }
            }
        }
    }

    // Sample BSDF with multiple importance sampling
    if !is_delta_light(light.flags()) {
        let mut f = Spectrum::default();
        let mut sampled_specular = false;

        match it {
            Interactions::SurfaceInteraction(ref s) => {
                let mut sampled_type = 0;
                f = s.bsdf.as_ref().unwrap().sample_f(
                    &s.wo, &mut wi, uscatt, &mut scattpdf,
                    bsdf_flags, &mut sampled_type);
                f *= wi.abs_dot_norm(&s.shading.n);
                sampled_specular = (sampled_type & BxDFType::Specular as u8) != 0;
            },
            Interactions::MediumInteraction(ref m) => {
                let p = m.phase.as_ref().unwrap().sample_p(&m.wo, &mut wi, uscatt);
                f = Spectrum::new(p);
                scattpdf = p;
            },
            _ => ()
        }

        debug!("  BSDF / phase sampling f: {}, scatteringPdf: ", scattpdf);

        if !f.is_black() && scattpdf > 0.0 {
            // Account for light contributions along sampled direction wi
            let mut weight = 1.0;
            if !sampled_specular {
                lightpdf = light.pdf_li(&it.get_data(), &wi);
                if lightpdf == 0.0 { return Ld; }
                weight = power_heuristic(1, scattpdf, 1, lightpdf);
            }

            // Find intersection and compute transmittance
            let mut light_isect = SurfaceInteraction::default();
            let mut ray = it.spawn_ray(&wi);
            let mut Tr = Spectrum::new(1.0);
            let found_surface_intr = if handle_media {
                scene.intersect_tr(ray.clone(), sampler, &mut light_isect, &mut Tr)
            } else {
                scene.intersect(&mut ray, &mut light_isect)
            };

            // Add light contribution from material sampling
            let mut li = Spectrum::new(0.0);

            if found_surface_intr {
                let alight = light_isect.primitive.as_ref().unwrap().get_area_light();

                if let Some(ref l) = alight {
                    if Arc::ptr_eq(l, light) {
                        li = light_isect.le(&(-wi));
                    }
                }
            } else {
                li = light.le(&ray);
            }
            if !li.is_black() { Ld += f * li * Tr * weight / scattpdf; }
        }
    }

    Ld
}

pub fn compute_light_power_distribution(scene: &Scene) -> Option<Arc<Distribution1D>> {
    if scene.lights.is_empty() { return None; }

    let mut light_power = Vec::with_capacity(scene.lights.len());
    scene.lights.iter().for_each(|l| { light_power.push(l.power().y()); });

    Some(Arc::new(Distribution1D::new(light_power)))

}

#[enum_dispatch]
pub trait Integrator {
    fn render(&mut self, scene: &Scene);
}

pub trait SamplerIntegrator: Integrator + Send + Sync {
    fn camera(&self) -> Arc<Cameras>;

    fn sampler(&self) -> &Samplers;

    fn bounds(&self) -> Bounds2i;

    fn preprocess(&mut self, _scene: &Scene) {}

    fn render(&mut self, scene: &Scene) {
        self.preprocess(scene);

        let camera = self.camera();
        let bounds = self.bounds();
        let sampler = self.sampler();

        // Redner image tile in parallel

        // compute number of tiles ntiles to use for parallel rendering
        let film: Arc<Film> = camera.film();
        let sbounds = film.get_sample_bounds();
        let sextent = sbounds.diagonal();
        let tilesize = 16;
        let ntiles = Point2i::new(
            (sextent.x + tilesize - 1) / tilesize,
            (sextent.y + tilesize - 1) / tilesize);

        let tiles = (0..ntiles.x * ntiles.y)
            .map(|i| Point2i::new(i % ntiles.x, i / ntiles.x))
            .collect::<Vec<_>>();

        let pb = Arc::new(ProgressBar::new(tiles.len() as _));
        pb.set_style(ProgressStyle::default_bar()
            .template("[{elapsed_precise}] [{wide_bar}] {percent}% [{pos}/{len}] ({eta})"));
        set_progress_bar(Some(Arc::downgrade(&pb)));

        let (sendt, recvt) = bounded(tiles.len());

        // TODO: Progress reporter
        tiles
            .par_iter()
            .for_each(|Point2i { x, y }| {
                let tiles = sendt.clone();
                // Render section of image corresponding to tile

                // Allocate MemoryArena for tile
                // Get sampler instance for tile
                let seed = y * ntiles.x + x;
                let mut tile_sampler = Sampler::clone(&*sampler, seed);

                // Compute sample bounds for tile
                let x0 = sbounds.p_min.x + x * tilesize;
                let x1 = std::cmp::min(x0 + tilesize, sbounds.p_max.x);
                let y0 = sbounds.p_min.y + y * tilesize;
                let y1 = std::cmp::min(y0 + tilesize, sbounds.p_max.y);
                let p1 = Point2i::new(x0, y0);
                let p2 = Point2i::new(x1, y1);
                let tile_bounds = Bounds2i::from_points(&p1, &p2);
                println!("Starting image tile {}", tile_bounds);

                // Get FilmTile for tile
                let mut film_tile = film.get_film_tile(&tile_bounds);

                // Loop over pixels in tile to render them
                {
                    for pixel in &tile_bounds {
                        // TODO: ProfilePhase
                        tile_sampler.start_pixel(&pixel);

                        // Do this check after the start_pixel() call; this keeps
                        // the usage of RNG values from (most) sampler that use
                        // RNGs consistent, which improves reproducability /
                        // debugging.
                        if !bounds.inside_exclusive(&pixel) { continue; }
                        let mut herd = bumpalo_herd::Herd::new();

                        loop {
                            {
                                let arena = herd.get();
                                // Initialize CameraSample for current sample
                                let camera_sample = tile_sampler.get_camera_sample(&pixel);

                                // Generate camera ray for current sample
                                let mut ray = Ray::default();
                                let ray_weight = camera.generate_ray_differential(&camera_sample, &mut ray);
                                ray.scale_differential(1.0 / (tile_sampler.samples_per_pixel() as Float).sqrt());
                                ncamera_rays::inc();

                                // Evaluate radiance along camera ray
                                let mut L = Spectrum::new(0.0);
                                if ray_weight > 0.0 {
                                    L = self.li(&mut ray, scene, &mut tile_sampler, &arena, 0);
                                }

                                // Issue warning if unexpected radiance value returned
                                if L.has_nans() {
                                    error!(
                                        "Not-a-number radiance value returned \
                                        for pixel ({}, {}), sample {}. Setting to black",
                                        x, y, tile_sampler.current_sample_number());
                                    L = Spectrum::new(0.0);
                                } else if L.y() < -1.0e-5 {
                                    error!(
                                        "Negative luminance value, {}, returned \
                                        for pixel ({}, {}), sample {}. Setting to black.",
                                        L.y(), x, y, tile_sampler.current_sample_number());
                                    L = Spectrum::new(0.0);
                                } else if L.y().is_infinite() {
                                    error!(
                                        "Infinite luminance value returned \
                                        for pixel ({}, {}), sample {}. Setting to black.",
                                        x, y, tile_sampler.current_sample_number());
                                    L = Spectrum::new(0.0);
                                }

                                debug!(
                                    "Camera sample: {} -> ray: {} -> L = {}",
                                    camera_sample, ray, L);
                                // Add camera ray's contribution to image
                                film_tile.add_sample(&camera_sample.pfilm, L, ray_weight);

                                if !tile_sampler.start_next_sample() { break; }
                            }
                            // Free MemoryArena memory from computing image sample value
                            herd.reset();
                        }
                    }
                }

                info!("Finished image tile {}", tile_bounds);

                // Send image tile to main thead for merging
                tiles.send(film_tile).unwrap();
                pb.inc(1);
            });

        // Merge film tiles in main thread
        for _ in 0..tiles.len() {
            let mut tile = recvt.recv().unwrap();
            // Merge image tile into film
            film.merge_film_tile(&mut tile);
        }

        info!("Rendering finished");
        pb.finish_and_clear();

        // Save final image after rendering
        film.write_image(1.0).unwrap();
    }

    fn li(
        &self, r: &mut Ray, scene: &Scene, sampler: &mut Samplers,
        arena: &Member, depth: usize) -> Spectrum;

    fn specular_reflect(
        &self, r: &Ray, isect: &SurfaceInteraction, scene: &Scene,
        sampler: &mut Samplers, arena: &Member, depth: usize) -> Spectrum {
        // Compute specular reflection direction wi and BSDF value
        let wo = isect.wo;
        let mut wi = Vector3f::default();
        let mut pdf = 0.0;
        let ty = BxDFType::Reflection as u8 | BxDFType::Specular as u8;
        let bsdf = isect.bsdf.as_ref().unwrap();
        let f = bsdf.sample_f(&wo, &mut wi, &sampler.get_2d(), &mut pdf, ty, &mut 0);

        // Return contribution of specular reflection
        let ns = &isect.shading.n;

        if pdf > 0.0 && !f.is_black() && wi.abs_dot_norm(ns) != 0.0 {
            // Compute ray differential rd for specular reflection
            let mut rd = isect.spawn_ray(&wi);

            if let Some(ref diff) = r.diff {
                let rx_origin = isect.p + isect.dpdx.get();
                let ry_origin = isect.p + isect.dpdy.get();
                // Compute differential reflected directions
                let dndx = isect.shading.dndu * isect.dudx.get() +
                           isect.shading.dndv * isect.dvdx.get();
                let dndy = isect.shading.dndu * isect.dudy.get() +
                           isect.shading.dndv * isect.dvdy.get();
                let dwodx = -diff.rx_direction - wo;
                let dwody = -diff.ry_direction - wo;
                let ddndx = dwodx.dot_norm(ns) + wo.dot_norm(&dndx);
                let ddndy = dwody.dot_norm(ns) + wo.dot_norm(&dndy);
                let rx_direction =
                    wi - dwodx + Vector3f::from(dndx * wo.dot_norm(ns) + *ns * ddndx) * 2.0;
                let ry_direction =
                    wi - dwody + Vector3f::from(dndy * wo.dot_norm(ns) + *ns * ddndy) * 2.0;
                let d = RayDifferential {
                    rx_origin, ry_origin,
                    rx_direction, ry_direction,
                    has_differentials: true
                };

                rd.diff = Some(d);
            }

            f * self.li(&mut rd, scene, sampler, arena, depth + 1) * wi.abs_dot_norm(ns) / pdf
        } else {
            Spectrum::new(0.0)
        }
    }

    fn specular_transmit(
        &self, r: &Ray, isect: &SurfaceInteraction, scene: &Scene,
        sampler: &mut Samplers, arena: &Member, depth: usize) -> Spectrum {
        let wo = isect.wo;
        let mut wi = Vector3f::default();
        let mut pdf = 0.0;
        let p = &isect.p;
        let bsdf = isect.bsdf.as_ref().unwrap();
        let ty = BxDFType::Transmission as u8 | BxDFType::Specular as u8;
        let f = bsdf.sample_f(&wo, &mut wi, &sampler.get_2d(), &mut pdf, ty, &mut 0);
        let mut L = Spectrum::new(0.0);
        let ns = isect.shading.n;

        if pdf > 0.0 && !f.is_black() && wi.abs_dot_norm(&ns) != 0.0 {
            // Compute ray differential rd for specular transmission
            let mut rd = isect.spawn_ray(&wi);

            if let Some(ref d) = r.diff {
                let rx_origin = *p + isect.dpdx.get();
                let ry_origin = *p + isect.dpdy.get();

                let dndx = isect.shading.dndu * isect.dudx.get() +
                                        isect.shading.dndv * isect.dvdx.get();
                let dndy = isect.shading.dndu * isect.dudy.get() +
                                        isect.shading.dndv * isect.dvdy.get();

                // The BSDF stores the IOR of the interior of the object being
                // intersected.  Compute the relative IOR by first out by
                // assuming that the ray is entering the object.
                let mut eta = bsdf.eta;
                let w = -wo;

                if wo.dot_norm(&ns) < 0.0 {
                    // If the ray isn't entering, then we need to invert the
                    // relative IOR and negate the normal and its derivatives.
                    eta = 1.0 / eta;
                }

                let dwodx = -d.rx_direction - wo;
                let dwody = -d.ry_direction - wo;
                let ddndx = dwodx.dot_norm(&ns) + wo.dot_norm(&dndx);
                let ddndy = dwody.dot_norm(&ns) + wo.dot_norm(&dndy);

                let mu = eta * w.dot_norm(&ns) - wi.dot_norm(&ns);
                let dmudx = (eta - (eta * eta * w.dot_norm(&ns)) / wi.dot_norm(&ns)) * ddndx;
                let dmudy = (eta - (eta * eta * w.dot_norm(&ns)) / wi.dot_norm(&ns)) * ddndy;

                let rx_direction = wi + dwodx * eta - Vector3f::from(dndx * mu + ns * dmudx);
                let ry_direction = wi + dwody * eta - Vector3f::from(dndy * mu + ns * dmudy);
                let diff = RayDifferential {
                    rx_origin, ry_origin,
                    rx_direction, ry_direction,
                    has_differentials: true
                };

                rd.diff = Some(diff);
            }

            L = f * self.li(&mut rd, scene, sampler, arena, depth + 1) * (wi.abs_dot_norm(&ns) / pdf)
        }

        L
    }


}

#[enum_dispatch(Integrator)]
pub enum Integrators {
    AOIntegrator,
    MLTIntegrator,
    PathIntegrator,
    BDPTIntegrator,
    WhittedIntegrator,
    VolPathIntegrator,
    SPPMIntegrator,
    DirectLightingIntegrator
}

