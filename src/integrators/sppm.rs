use crate::{stat_ratio, stat_counter, stat_int_distribution, stat_memory_counter, stat_float_distribution};
use crate::core::geometry::point::{Point3f, Point2i, Point3i, Point2f};
use crate::core::geometry::vector::Vector3f;
use crate::core::spectrum::Spectrum;
use crate::core::pbrt::{Float, clamp, lerp, PI, Options};
use crate::core::reflection::{BSDF, BxDFType};
use crate::core::parallel::AtomicFloat;
use std::sync::atomic::{AtomicI32, Ordering};
use atom::{AtomSetOnce, Atom};
use std::sync::{Arc};
use bumpalo_herd::{Herd};
use crossbeam::crossbeam_channel::bounded;
use crate::core::camera::{Cameras, Camera};
use crate::core::integrator::{Integrator, compute_light_power_distribution, uniform_sample_onelight, Integrators};
use crate::core::scene::Scene;
use crate::core::film::Film;
use crate::samplers::halton::HaltonSampler;
use rayon::prelude::*;
use crate::core::sampler::Sampler;
use crate::core::geometry::bounds::{Bounds2i, Bounds3f};
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{SurfaceInteraction, Interaction, Interactions};
use crate::core::light::Light;
use crate::core::material::{TransportMode};
use crate::core::lowdiscrepancy::radical_inverse;
use crate::core::geometry::normal::Normal3f;
use crate::core::paramset::ParamSet;

stat_ratio!(
    "Stochastic Progressive Photon Mapping/Visible points checked per photon intersection",
    vpc_per_tpsi);
stat_counter!("Stochastic Progressive Photon Mapping/Photon paths followed", photon_paths);
stat_int_distribution!(
    "Stochastic Progressive Photon Mapping/Grid cells per visible point",
    grid_cells_per_visible_point);
stat_memory_counter!("Memory/SPPM Pixels", pixel_memory_bytes);
stat_float_distribution!("Memory/SPPM BSDF and Grid Memory", memory_arena_mb);

pub fn init_stats() {
    vpc_per_tpsi::init();
    photon_paths::init();
    grid_cells_per_visible_point::init();
    pixel_memory_bytes::init();
    memory_arena_mb::init();
}

pub struct SPPMIntegrator {
    camera                  : Arc<Cameras>,
    initial_search_radius   : Float,
    niterations             : isize,
    max_depth               : isize,
    photons_per_iteration   : isize,
    write_frequency         : isize
}

impl SPPMIntegrator {
    pub fn new(
        camera: Arc<Cameras>, niterations: isize, ppi: isize,
        max_depth: isize, initial_search_radius: Float,
        write_frequency: isize) -> Self {
        let photons_per_iteration = if ppi > 0 {
            ppi
        } else {
            camera.film().cropped_pixel_bounds.area()
        };

        Self {
            camera, niterations, photons_per_iteration,
            max_depth, initial_search_radius, write_frequency
        }
    }
}

impl Integrator for SPPMIntegrator {
    fn render(&mut self, scene: &Scene) {
        // TODO: ProfilePhase
        // Initiliaze pixelBounds and pixels array for SPPM
        let mut herd = Herd::new();
        let film: Arc<Film> = self.camera.film();
        let camera = &self.camera;
        let pbounds = film.cropped_pixel_bounds;
        let npixels = pbounds.area() as usize;
        let mut pixels = Vec::with_capacity(npixels);

        for _i in 0..npixels {
            let p = SPPMPixel{ radius: self.initial_search_radius, ..Default::default() };
            pixels.push(p);
        }
        let inv_sqrt_spp = 1.0 / (self.niterations as Float).sqrt();
        pixel_memory_bytes::add((npixels * std::mem::size_of::<SPPMPixel>()) as u64);
        // Compute lightDistr for sampling proportional to power
        let distr = compute_light_power_distribution(scene).unwrap();

        // Perform niterations of SPPM integration
        let sampler = HaltonSampler::new(self.niterations as usize, &pbounds, false);

        // Compute number of tiles to use for SPPM camera pass
        let pextent = pbounds.diagonal();
        let tilesize = 16;
        let ntiles = Point2i::new(
            (pextent.x + tilesize - 1) / tilesize,
            (pextent.y + tilesize - 1) / tilesize);
        let mut tiles = Vec::with_capacity((ntiles.x  * ntiles.y) as usize);

        for y in 0..ntiles.y {
            for x in 0..ntiles.x{
                tiles.push(Point2i::new(x, y));
            }
        }



        // TODO ProgressReporter
        for iter in 0..self.niterations {

            {
            let (sendt, recvt) = bounded(tiles.len());

            // TODO: ProfilePhase
            tiles
                .par_iter()
                .for_each_init(|| herd.get(), |arena, Point2i { x, y }| {
                    let sendtx = sendt.clone();
                    //let arena = herd.get();
                    let mut pixs = Vec::new();
                    let tidx = y * ntiles.x + x;
                    let mut tsampler = sampler.clone(tidx);

                    // Compute tileBounds for SPPM tile
                    let x0 = pbounds.p_min.x + x * tilesize;
                    let x1 = std::cmp::min(x0 + tilesize, pbounds.p_max.x);
                    let y0 = pbounds.p_min.y + y * tilesize;
                    let y1 = std::cmp::min(y0 + tilesize, pbounds.p_max.y);
                    let tbounds = Bounds2i::from_points(
                        &Point2i::new(x0, y0),
                        &Point2i::new(x1, y1));

                    for ppixel in &tbounds {
                        // Prepare tileSampler for ppixel
                        tsampler.start_pixel(&ppixel);
                        tsampler.set_sample_number(iter as u64);

                        // Generate camera ray for pixel for SPPM
                        let csample = tsampler.get_camera_sample(&ppixel);
                        let mut ray = Ray::default();
                        let mut beta = Spectrum::new(camera.generate_ray_differential(&csample, &mut ray));
                        if beta.is_black() { continue; }
                        ray.scale_differential(inv_sqrt_spp);

                        // Follow camera ray path until a visible point is created

                        // Get SPPMPixel for ppixel
                        let ppixelo = Point2i::from(ppixel - pbounds.p_min);
                        let pixel_offset = ppixelo.x + ppixelo.y * (pbounds.p_max.x - pbounds.p_min.x);
                        let mut Ld = Spectrum::default();
                        let mut vispoint = VisiblePoint::default();
                        let mut specular_bounce = false;

                        for depth in 0..self.max_depth {
                            let mut isect = SurfaceInteraction::default();
                            vpc_per_tpsi::inc_den();
                            if !scene.intersect(&mut ray, &mut isect) {
                                // Accumulate light contributions for ray with no intersection
                                for light in scene.lights.iter() {
                                    Ld += beta * light.le(&ray);
                                }
                                break;
                            }

                            // Process camera ray intersection
                            isect.compute_scattering_functions(&ray, arena, true, TransportMode::Radiance);

                            // TODO: check --depth
                            if isect.bsdf.is_none() {
                                ray = isect.spawn_ray(&ray.d);
                                continue;
                            }

                            //let bsdf = isect.bsdf.unwrap();


                            // Accumulate direct illumination at SPPM camera ray intersection
                            let wo = -ray.d;
                            if depth == 0 || specular_bounce {
                                Ld += beta * isect.le(&wo);
                            } else {
                                let i: Interactions = isect.into();
                                Ld += uniform_sample_onelight(&i, scene, arena, &mut tsampler, false, None);
                                isect = i.get_surfaceinteraction()
                            }

                            let bsdf = isect.bsdf.take().unwrap();

                            // Possibly create visible point and camera path
                            let is_diffuse = bsdf
                                .num_components(BxDFType::Diffuse as u8 | BxDFType::Reflection as u8 | BxDFType::Transmission as u8) > 0;
                            let is_glossy = bsdf
                                .num_components(BxDFType::Glossy as u8 | BxDFType::Reflection as u8 | BxDFType::Transmission as u8) > 0;

                            if is_diffuse || (is_glossy && depth == self.max_depth - 1) {
                                vispoint = VisiblePoint::new(isect.p, wo, Some(bsdf), beta);
                                break;
                            }

                            // Spawn ray from SPPM camera path vertex
                            if depth < self.max_depth - 1 {
                                let mut pdf = 0.0;
                                let mut wi = Vector3f::default();
                                let mut ty = 0;
                                let f = bsdf.sample_f(
                                    &wo, &mut wi, &tsampler.get_2d(),
                                    &mut pdf, BxDFType::All as u8, &mut ty);

                                if pdf == 0.0 || f.is_black() { break; }
                                specular_bounce = (ty & BxDFType::Specular as u8) != 0;
                                beta *= f * wi.abs_dot_norm(&isect.shading.n) / pdf;

                                if beta.y() < 0.25 {
                                    let continue_prob = beta.y().min(1.0);
                                    if tsampler.get_1d() > continue_prob { break; }
                                    beta /= continue_prob;
                                }

                                ray = isect.spawn_ray(&wi);
                            }
                        }

                        pixs.push((pixel_offset, Ld, vispoint));
                    }

                    // Send pixels for tile to main thread
                    sendtx.send(pixs).unwrap();
                });

            for _i in 0..tiles.len() {
                let tile = recvt.recv().unwrap();
                for (offset, ld, vp) in tile {
                    let pixel = &mut pixels[offset as usize];
                    pixel.ld += ld;
                    pixel.vp = vp;
                }
            }
        }

            // Create grid of all SPPM visible points
            let mut gridres = [0; 3];
            let mut grid_bounds = Bounds3f::default();
            // Allocate grid for SPPM visible points
            let hash_size = npixels;

            let mut gridtemp: Vec<Atom<&SPPMPixelListNode>> = Vec::with_capacity(hash_size);
            let mut grid: Vec<AtomSetOnce<&SPPMPixelListNode>> = Vec::with_capacity(hash_size);

            for _i in 0..hash_size {
                gridtemp.push(Atom::empty());
                grid.push(AtomSetOnce::empty());
            }

            // TODO ProfilePhase

            // Compute grid bounds for SPPM visible points
            let mut max_radius = 0.0;
            for pixel in pixels.iter() {
                if pixel.vp.beta.is_black() { continue; }
                let vpbound = Bounds3f::from_point(&pixel.vp.p).expand(pixel.radius);
                grid_bounds = grid_bounds.union_bounds(&vpbound);
                max_radius = pixel.radius.max(max_radius);
            }

            // Compute resolution of SPPM grid in each dimension
            let diag = grid_bounds.diagonal();
            let maxdiag = diag.max_component();
            let base_grid_res = (maxdiag / max_radius) as isize;
            assert!(base_grid_res > 0);

            for (i, res) in gridres.iter_mut().enumerate() {
                *res = std::cmp::max((base_grid_res as Float * diag[i]) as isize, 1);
            }

            pixels
                .par_iter()
                .for_each_init(|| herd.get(), |arena, pixel| {
                    if !pixel.vp.beta.is_black() {
                        // Add pixel's point to applicable grid cells
                        let radius = pixel.radius;
                        let mut pmin = Point3i::default();
                        let mut pmax = Point3i::default();

                        to_grid(&(pixel.vp.p - Vector3f::new(radius, radius, radius)),
                                &grid_bounds, &gridres, &mut pmin);
                        to_grid(&(pixel.vp.p + Vector3f::new(radius, radius, radius)),
                                &grid_bounds, &gridres, &mut pmax);

                        for z in pmin.z..pmax.z {
                            for y in pmin.y..pmax.y {
                                for x in pmin.x..pmax.x {
                                    // Add visible point to grid cell (x, y, z)
                                    let h = hash(Point3i::new(x, y, z), hash_size as isize);
                                    let node = arena.alloc(SPPMPixelListNode::new(pixel));
                                    let old = gridtemp[h].swap(node, Ordering::SeqCst);

                                    if let Some(oldval) = old {
                                        node.next.set_if_none(oldval, Ordering::SeqCst);
                                    }
                                }
                            }
                        }

                        let val = (1 + pmax.x - pmin.x) * (1 + pmax.y - pmin.y) * (1 + pmax.z - pmin.z);
                        grid_cells_per_visible_point::report_value(val as u64);
                    }
                });

            for i in 0..hash_size {
                let node = gridtemp[i].take(Ordering::Relaxed);

                if let Some(s) = node {
                    grid[i].set_if_none(s, Ordering::SeqCst);
                }
            }

            std::mem::drop(gridtemp);
            //Trace photons and accumulate contributions
            // TODO: ProfilePhase
            (0..self.photons_per_iteration)
                .into_par_iter()
                .for_each_init(|| herd.get(), | arena, index| {
                    // Follow photon path for photonIndex
                    let halton_index = iter as u64 * self.photons_per_iteration as u64 + index as u64;
                    let mut haltondim = 0;

                    // Choose light to shoot photon from
                    let mut light_pdf = 0.0;
                    let light_sample = radical_inverse(haltondim, halton_index);
                    haltondim += 1;
                    let light_num = distr.sample_discrete(light_sample, Some(&mut light_pdf), None);
                    let light = &scene.lights[light_num];

                    // Compute sample values for photon ray leaving light source
                    let ulight0 = Point2f::new(
                        radical_inverse(haltondim, halton_index),
                        radical_inverse(haltondim + 1, halton_index));
                    let ulight1 = Point2f::new(
                        radical_inverse(haltondim + 2, halton_index),
                        radical_inverse(haltondim + 3, halton_index));
                    let ulight_time = lerp(
                        radical_inverse(haltondim + 4, halton_index),
                        camera.shutter_open(), camera.shutter_close());
                    haltondim += 5;

                    // Generate photonRay from light source and initialize beta
                    let mut photon_ray = Ray::default();
                    let mut nlight = Normal3f::default();
                    let mut pdfpos = 0.0;
                    let mut pdfdir = 0.0;
                    let le = light.sample_le(
                        &ulight0, &ulight1, ulight_time, &mut photon_ray,
                        &mut nlight, &mut pdfpos, &mut pdfdir);

                    if pdfpos == 0.0 || pdfdir == 0.0 || le.is_black() { return; }

                    let mut beta =
                        (le * nlight.abs_dot_vec(&photon_ray.d)) /
                        (light_pdf * pdfpos * pdfdir);

                    if beta.is_black() { return; }

                    // Follow photon path through scene and record intersections
                    let mut isect = SurfaceInteraction::default();

                    for depth in 0..self.max_depth {
                        if !scene.intersect(&mut photon_ray, &mut isect) { break; }
                        vpc_per_tpsi::inc_den();

                        if depth > 0 {
                            // Add photon contribution to nearby visible points
                            let mut photon_grid_index = Point3i::default();
                            if to_grid(&isect.p, &grid_bounds, &gridres, &mut photon_grid_index) {
                                let h = hash(photon_grid_index, hash_size as isize);
                                // Add photon contribution to visible points in grid[h]
                                let mut opt = grid[h].get(Ordering::Relaxed);

                                while let Some(node) = opt {
                                    vpc_per_tpsi::inc_num();
                                    let pixel = node.pixel;
                                    let radius = pixel.radius;
                                    if pixel.vp.p.distance_squared(&isect.p) > radius * radius {
                                        opt = node.next.get(Ordering::Relaxed);
                                        continue;
                                    }
                                    // Update pixel Phi and M for nearby photon
                                    let wi = -photon_ray.d;
                                    let bsdf = pixel.vp.bsdf.as_ref().unwrap();
                                    let phi = beta * bsdf.f(&pixel.vp.wo, &wi, BxDFType::All as u8);
                                    for i in 0..Spectrum::n() {
                                        pixel.phi[i].add(phi[i]);
                                    }

                                    pixel.m.fetch_add(1, Ordering::Relaxed);
                                }
                            }
                        }

                        // Sample new photon ray direction

                        // Compute BSDF at photon intersection point
                        let mode = TransportMode::Importance;
                        isect.compute_scattering_functions(&photon_ray, arena, true, mode);

                        if isect.bsdf.is_none() {
                            photon_ray = isect.spawn_ray(&photon_ray.d);
                            continue;
                        }

                        let pbsdf = isect.bsdf.take().unwrap();

                        // Sample BSDF fr and direction wi for reflected photon
                        let mut wi = Vector3f::default();
                        let wo = -photon_ray.d;
                        let mut pdf = 0.0;
                        let mut flags = 0;

                        // Generate bsdfSample for outgoing photon sample
                        let bsdf_sample = Point2f::new(
                            radical_inverse(haltondim, halton_index),
                            radical_inverse(haltondim + 1, halton_index));
                        haltondim += 2;
                        let ty = BxDFType::All as u8;
                        let fr = pbsdf.sample_f(&wo, &mut wi, &bsdf_sample, &mut pdf, ty, &mut flags);
                        if fr.is_black() || pdf == 0.0 { break; }
                        let bnew = beta * fr * wi.abs_dot_norm(&isect.shading.n) / pdf;

                        // Possibly terminate photon path with Russian roulette
                        let q = (1.0 - bnew.y() / beta.y()).max(0.0);
                        if radical_inverse(haltondim, halton_index) < q { break; }
                        haltondim += 1;
                        beta = bnew / (1.0 - q);
                        photon_ray = isect.spawn_ray(&wi);
                    }
                });
            std::mem::drop(grid);

            photon_paths::add(self.photons_per_iteration as u64);

            // Update pixel values from this pass's photons
            // TODO: ProfilePhase
            pixels
                .par_iter_mut()
                .for_each(|p: &mut SPPMPixel| {
                    let mval = p.m.load(Ordering::Relaxed);
                    if mval > 0 {
                        // Update pixel photon count, search radius, and tau from photons
                        let gamma = 2.0 / 3.0;
                        let nnew = p.n + gamma * mval as Float;
                        let rnew = p.radius * (nnew / p.n + mval as Float).sqrt();
                        let mut phi = Spectrum::default();

                        for j in 0..Spectrum::n() {
                            phi[j] = Float::from(&p.phi[j]);
                        }
                        p.tau = (p.tau + p.vp.beta * phi) * (rnew * rnew) / (p.radius * p.radius);
                        p.n = nnew;
                        p.radius = rnew;
                        p.m.store(0, Ordering::Relaxed);

                        for j in 0..Spectrum::n() {
                            p.phi[j] = AtomicFloat::new(0.0);
                        }
                    }

                    // Reset VisiblePoint in pixel
                    p.vp.beta = Spectrum::new(0.0);
                    p.vp.bsdf.take();
                });

            // Periodically store SPPM image in film and write image
            if iter + 1 == self.niterations || (iter + 1) % self.write_frequency == 0 {
                let x0 = pbounds.p_min.x as usize;
                let x1 = pbounds.p_max.x as usize;
                let Np = (iter + 1) as u64 * self.photons_per_iteration as u64;
                let mut image = vec![Spectrum::default(); pbounds.area() as usize];
                let mut offset = 0;

                for y in (pbounds.p_min.y as usize)..(pbounds.p_max.y as usize)  {
                    for x in x0..x1 {
                        // compute radiance L for SPPM pixel pixel
                        let pixel =
                            &pixels[(y - pbounds.p_min.y as usize) * (x1 - x0) + (x - x0)];
                        let mut L = pixel.ld / (iter + 1) as Float;
                        L += pixel.tau / (Np as Float * PI * pixel.radius * pixel.radius);
                        image[offset] = L;
                        offset += 1;
                    }
                }

                film.set_image(&image);
                film.write_image(1.0).unwrap();
                // TODO: Write SPPM radius image if requested
            }

        }
        herd.reset();

    }
}

#[derive(Default, Clone)]
struct VisiblePoint<'a> {
    p   : Point3f,
    wo  : Vector3f,
    bsdf: Option<&'a BSDF<'a>>,
    beta: Spectrum
}

impl<'a> VisiblePoint<'a> {
    pub fn new(p: Point3f, wo: Vector3f, bsdf: Option<&'a BSDF<'a>>, beta: Spectrum) -> Self {
        Self { p, wo, bsdf, beta }
    }
}

struct SPPMPixel<'a> {
    radius  : Float,
    ld      : Spectrum,
    vp      : VisiblePoint<'a>,
    phi     : [AtomicFloat; Spectrum::n()],
    m       : AtomicI32,
    n       : Float,
    tau     : Spectrum
}

impl<'a> Default for SPPMPixel<'a> {
    fn default() -> Self {
        Self {
            radius: 0.0,
            ld    : Default::default(),
            vp    : Default::default(),
            phi   : array_init::array_init(|_| AtomicFloat::default()),
            m     : Default::default(),
            n     : 0.0,
            tau   : Default::default()
        }
    }
}

struct SPPMPixelListNode<'a> {
    pixel : &'a SPPMPixel<'a>,
    next  : AtomSetOnce<&'a SPPMPixelListNode<'a>>
}

impl<'a> SPPMPixelListNode<'a> {
    pub fn new(pixel: &'a SPPMPixel<'a>) -> Self {
        Self {
            pixel,
            next: AtomSetOnce::empty()
        }
    }
}

fn to_grid(p: &Point3f, bounds: &Bounds3f, gridres: &[isize; 3], pi: &mut Point3i) -> bool {
    let mut inbounds = true;
    let pg = bounds.offset(p);

    for i in 0..3 {
        (*pi)[i] = (gridres[i] as Float * pg[i]) as isize;
        inbounds &= (*pi)[i] >= 0 && (*pi)[i] <= gridres[i];
        (*pi)[i] = clamp((*pi)[i], 0, gridres[i] - 1);
    }

    inbounds
}

fn hash(p: Point3i, size: isize) -> usize {
    let (x, _) = p.x.overflowing_mul(73856093);
    let (y, _) = p.y.overflowing_mul(19349663);
    let (z, _) = p.z.overflowing_mul(83492791);

    (x ^ y ^ z) as usize % size as usize
}

pub fn create_sppm_integrator(params: &ParamSet, camera: Arc<Cameras>, opts: &Options) -> Option<Integrators> {
    let numi = params.find_one_int("numiterations", 64);
    let mut niterations = params.find_one_int("iterations", numi);
    let maxdepth = params.find_one_int("maxdepth", 5);
    let photons_per_iter = params.find_one_int("photonsperiteration", -1);
    let writefreq = params.find_one_int("imagewritefrequency", 1 << 31);
    let radius = params.find_one_float("radius", 1.0);

    if opts.quick_render { niterations = std::cmp::max(1, niterations / 16); }

    Some(
        SPPMIntegrator::new(
            camera, niterations, photons_per_iter,
            maxdepth, radius,writefreq).into())
}