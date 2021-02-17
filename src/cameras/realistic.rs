use crate::core::pbrt::{Float, quadratic, INFINITY, lerp};
use crate::core::geometry::bounds::{Bounds2f};
use crate::core::transform::{AnimatedTransform, Transform};
use bumpalo::core_alloc::sync::Arc;
use crate::core::film::Film;
use crate::core::medium::Mediums;
use log::{info, warn, error};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::normal::Normal3f;
use crate::core::geometry::vector::Vector3f;
use crate::core::reflection::refract;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::lowdiscrepancy::radical_inverse;
use num::integer::Roots;
use rayon::prelude::*;
use lazy_static::lazy_static;
use crate::stat_percent;
use crate::core::camera::{Camera, CameraSample, Cameras};
use crate::core::paramset::ParamSet;
use crate::get_camera_data;
use crate::core::floatfile::read_float_file;

stat_percent!("Camera/Rays vignetted by lens system", vignetted_total);

pub fn init_stats() {
    vignetted_total::init();
}

struct LensElementInterface {
    curvature_radius    : Float,
    thickness           : Float,
    eta                 : Float,
    aperture_radius     : Float
}

pub struct RealisticCamera {
    pub camera_to_world     : AnimatedTransform,
    pub shutter_open        : Float,
    pub shutter_close       : Float,
    pub film                : Arc<Film>,
    pub medium              : Option<Arc<Mediums>>,
    simple_weighting        : bool,
    element_interface       : Vec<LensElementInterface>,
    exit_pupil_bounds       : Vec<Bounds2f>
}

impl RealisticCamera {
    pub fn new(
        camera_to_world: AnimatedTransform, shutter_open: Float, shutter_close: Float,
        aperture_diameter: Float, focus_distance: Float, simple_weighting: bool,
        lens_data: &[Float], film: Arc<Film>, medium: Option<Arc<Mediums>>) -> Self {
        let mut element_interface = Vec::new();

        for i in (0..lens_data.len()).step_by(4) {
            let mut d = lens_data[i + 3];
            if lens_data[i] == 0.0 {
                if aperture_diameter > lens_data[i + 3] {
                    warn!("Specified aperture diameter {} is greater than maximum \
                        possible {}. Clamping it.", aperture_diameter, lens_data[i + 3]);
                } else {
                    d = aperture_diameter;
                }
            }

            element_interface.push(LensElementInterface {
                curvature_radius: lens_data[i] * 0.001,
                thickness       : lens_data[i + 1] * 0.001,
                eta             : lens_data[i + 2],
                aperture_radius : d * 0.001 / 2.0
            });
        }

        let mut cam = RealisticCamera {
            film: film.clone(),
            exit_pupil_bounds: Vec::new(),
            camera_to_world, shutter_open, shutter_close,
            medium, element_interface, simple_weighting
        };

        // Compute lens--film distance for given focus distance
        let fb = cam.focus_binary_search(focus_distance);
        info!("Binary search focus: {} -> {}\n", fb, cam.focus_distance(fb));
        cam.element_interface.last_mut().unwrap().thickness = cam.focus_thicklens(focus_distance);
        info!(
            "Thick lens focus: {} -> {}\n",
            cam.element_interface.last().unwrap().thickness,
            cam.focus_distance(cam.element_interface.last().unwrap().thickness));

        // Compute exit pupil bounds at sampled points on the film
        let nsamples = 64;
        let mut exit_pupil_bounds = vec![Bounds2f::default(); nsamples];

        exit_pupil_bounds
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, p)| {
                let r0 = i as Float / nsamples as Float * film.diagonal / 2.0;
                let r1 = (i + 1) as Float / nsamples as Float * film.diagonal / 2.0;
                *p = cam.bound_exit_pupil(r0, r1);
            });

        cam.exit_pupil_bounds = exit_pupil_bounds;

        cam
    }

    fn lens_rearz(&self) -> Float {
        self.element_interface.last().unwrap().thickness
    }

    fn lens_frontz(&self) -> Float {
        self.element_interface
            .iter()
            .fold(0.0, |acc, elem| acc + elem.thickness )
    }

    fn rear_element_radius(&self) -> Float {
        self.element_interface.last().unwrap().aperture_radius
    }

    fn intersect_spherical_element(
        radius: Float, zcenter: Float, ray: &Ray,
        t: &mut Float, n: &mut Normal3f) -> bool {
        // Compute t0 and t1 for ray element intersection
        let o = ray.o - Vector3f::new(0.0, 0.0, zcenter);
        let A = ray.d.x * ray.d.x + ray.d.y * ray.d.y + ray.d.z * ray.d.z;
        let B = 2.0 * (ray.d.x * o.x + ray.d.y * o.y + ray.d.z * o.z);
        let C = o.x * o.x + o.y * o.y + o.z * o.z - radius * radius;
        let mut t0 = 0.0; let mut t1 = 0.0;
        if !quadratic(A, B, C, &mut t0, &mut t1) { return false; }

        // Select intersection t based on ray direction and element curvature
        let use_closert = (ray.d.z > 0.0) ^ (radius < 0.0);
        *t = if use_closert { t0.min(t1) } else { t0.max(t1) };
        if *t < 0.0 { return false; }

        // Compute surface normal of element at ray intersection point
        *n = Normal3f::from(Vector3f::from(o + ray.d * *t));
        *n = (*n).normalize().face_foward_vec(&(-ray.d));

        true
    }

    fn trace_lenses_from_film(&self, rcamera: &Ray, rout: Option<&mut Ray>) -> bool {
        let mut elementz = 0.0;

        // Transform rcamera from camera to lens system space
        lazy_static! {
            static ref CAMERA_TO_LENS: Transform = Transform::scale(1.0, 1.0, -1.0);
        }


        let mut rlens = CAMERA_TO_LENS.transform_ray(rcamera);

        for (i, elem) in self.element_interface.iter().enumerate().rev() {
            elementz -= elem.thickness;

            // Compute intersection of ray with lens element
            let mut t = 0.0;
            let mut n = Normal3f::default();
            let is_stop = elem.curvature_radius == 0.0;

            if is_stop {
                // The refracted ray computed in the previous lens element
                // interface may be pointed towards film plane(+z) in some
                // extreme situations; in such cases, 't' becomes negative.
                if rlens.d.z >= 0.0 { return false; }
                t = (elementz - rlens.o.z) / rlens.d.z;
            } else {
                let radius = elem.curvature_radius;
                let zcenter = elementz + elem.curvature_radius;
                let cond = RealisticCamera::intersect_spherical_element(
                    radius, zcenter, &rlens, &mut t, &mut n);

                if !cond { return false; }
            }

            assert!(t >= 0.0);

            // Test intersection point against element aperture
            let phit = rlens.find_point(t);
            let r2 = phit.x * phit.x + phit.y * phit.y;
            if r2 > elem.aperture_radius * elem.aperture_radius { return false; }
            rlens.o = phit;

            // Update ray path for element interface interaction
            if !is_stop {
                let mut w = Vector3f::default();
                let etai = elem.eta;
                let etat = if i > 0 && self.element_interface[i - 1].eta != 0.0 {
                    self.element_interface[i - 1].eta
                } else {
                    1.0
                };

                if !refract(&(-rlens.d).normalize(), &n, etai / etat, &mut w) { return false; }
                rlens.d = w;
            }
        }

        // Transform rlens from lens system space back to camera space
        if let Some(r) = rout {

            lazy_static! {
                static ref lens2cam: Transform = Transform::scale(1.0, 1.0, -1.0);
            }

            *r = lens2cam.transform_ray(&rlens);
        }

        true
    }

    fn trace_lenses_from_scene(&self, rcamera: &Ray, rout: Option<&mut Ray>) -> bool {
        let mut elementz = -self.lens_frontz();

        // Transform rcamera from camera to lens system space

        lazy_static! {
            static ref CAMERA_TO_LENS: Transform = Transform::scale(1.0, 1.0, -1.0);
        }

        let mut rlens = CAMERA_TO_LENS.transform_ray(rcamera);

        for (i, elem) in self.element_interface.iter().enumerate() {
            let mut t = 0.0;
            let mut n = Normal3f::default();
            let is_stop = elem.curvature_radius == 0.0;

            if is_stop {
                t = (elementz - rlens.o.z) / rlens.d.z;
            } else {
                let radius = elem.curvature_radius;
                let zcenter = elementz + elem.curvature_radius;
                let cond = RealisticCamera::intersect_spherical_element(radius, zcenter, &rlens, &mut t, &mut n);

                if !cond { return false; }
            }

            assert!(t > 0.0);

            // Test intersection point against element aperture
            let phit = rlens.find_point(t);
            let r2 = phit.x * phit.x + phit.y * phit.y;

            if r2 > elem.aperture_radius * elem.aperture_radius { return false; }
            rlens.o = phit;

            // Update ray path for from-scene element interface interaction
            if !is_stop {
                let mut wt = Vector3f::default();
                let etai = if i == 0 || self.element_interface[i - 1].eta == 0.0 {
                    1.0
                } else {
                    self.element_interface[i - 1].eta
                };
                let etat = if self.element_interface[i].eta != 0.0 {
                    self.element_interface[i].eta
                } else {
                    1.0
                };

                if !refract(&(-rlens.d).normalize(), &n, etai / etat, &mut wt) { return false; }
                rlens.d = wt;
            }

            elementz += elem.thickness;
        }

        // Transform rlens from lens system space back to camera space
        if let Some(r) = rout {

            lazy_static! {
                static ref lens2cam: Transform = Transform::scale(1.0, 1.0, -1.0);
            }

            *r = lens2cam.transform_ray(&rlens);
        }

        true
    }

    fn compute_cardinal_points(&self, rin: &Ray, rout: &Ray, pz: &mut Float, fz: &mut Float) {
        let tf = -rout.o.x / rout.d.x;
        *fz = -rout.find_point(tf).z;
        let tp = (rin.o.x - rout.o.x) / rout.d.x;
        *pz = -rout.find_point(tp).z;
    }

    fn compute_thicklens_approximation(&self, pz: &mut [Float; 2], fz: &mut [Float; 2]) {
        // Find height x from optical axis for paralle rays
        let x = 0.001 * self.film.diagonal;

        // Compute cardinal points for film side of lens system
        let mut rscene = Ray::new(
            &Point3f::new(x, 0.0, self.lens_frontz() + 1.0),
        &Vector3f::new(0.0, 0.0, -1.0),
            INFINITY, 0.0, None, None);

        let  mut rfilm = Ray::default();
        assert!(self.trace_lenses_from_scene(&rscene, Some(&mut rfilm)),
                "Unable to trace ray from scene to film for thick lens \
                approximation. Is aperture extremely small?");
        self.compute_cardinal_points(&rscene, &rfilm, &mut pz[0], &mut fz[0]);

        // Compute cardinal points for scene side of lens system
        rfilm = Ray::new(
            &Point3f::new(x, 0.0, self.lens_rearz() - 1.0),
        &Vector3f::new(0.0, 0.0, 1.0),
            INFINITY, 0.0, None, None);
        assert!(self.trace_lenses_from_film(&rfilm, Some(&mut rscene)),
                "Unable to trace ray from film to scene for thick lens \
                approximation. Is aperture stop extremely small?");
        self.compute_cardinal_points(&rfilm, &rscene, &mut pz[1], &mut fz[1]);
    }

    fn focus_thicklens(&self, focus_distance: Float) -> Float {
        let mut pz = [0.0; 2];
        let mut fz = [0.0; 2];
        self.compute_thicklens_approximation(&mut pz, &mut fz);
        info!("Cardinal points: p' = {} f' = {}, p = {} f = {}.\n" ,
                pz[0], fz[0], pz[1], fz[1]);
        info!("Effective focal length {}\n", fz[0] - pz[0]);

        // Compute translation of lens, delta, to focus at focus_distance
        let f = fz[0] - pz[0];
        let z = -focus_distance;
        let c = (pz[1] - z - pz[0]) * (pz[1] - z - 4.0 * f - pz[0]);
        assert!(c > 0.0, "Coefficient must positive. It looks like focusDistance {} \
                is too short for a given lenses configuration", focus_distance);
        let delta = 0.5 * (pz[1] - z + pz[0] - c.sqrt());

        self.element_interface.last().unwrap().thickness + delta

    }

    fn bound_exit_pupil(&self, pfilm_x0: Float, pfilm_x1: Float) -> Bounds2f {
        let mut pupilbounds = Bounds2f::default();

        // Sample a collection of points on the rear lens to find exit pupil
        let nsamples = 1024 * 1024;
        let mut nexiting_rays = 0;

        // Compute bounding box of projection of rear element on sampling plane
        let rear_radius = self.rear_element_radius();
        let projrear_bounds = Bounds2f::new(
            &Point2f::new(-1.5 * rear_radius, -1.5 * rear_radius),
            &Point2f::new(1.5 * rear_radius, 1.5 * rear_radius)
        );

        for i in 0..nsamples {
            // Find location of points on x segment and rear lens element
            let x = lerp((i as Float + 0.5) / nsamples as Float, pfilm_x0, pfilm_x1);
            let pfilm = Point3f::new(x, 0.0, 0.0);
            let u = [radical_inverse(0, i as u64), radical_inverse(1, i as u64)];
            let prear = Point3f::new(
                lerp(u[0], projrear_bounds.p_min.x, projrear_bounds.p_max.x),
                lerp(u[1], projrear_bounds.p_min.y, projrear_bounds.p_max.y),
                self.lens_rearz()
            );


            // Expand pupil bounds if ray makes it through the lens system
            if pupilbounds.inside(&Point2f::new(prear.x, prear.y)) ||
               self.trace_lenses_from_film(
                   &Ray::new(&pfilm, &(prear - pfilm),
                             INFINITY, 0.0, None, None), None) {
                pupilbounds = pupilbounds.union_pointf(&Point2f::new(prear.x, prear.y));
                nexiting_rays += 1;
            }
        }

        // Return entire element bounds if no rays made it through the lens system
        if nexiting_rays == 0 {
            info!("Unable to find exit pupil in x = [{}, {}] on film", pfilm_x0, pfilm_x1);
            return projrear_bounds;
        }

        // Expand bounds to account for sample spacing
        pupilbounds = pupilbounds.expand(2.0 * projrear_bounds.diagonal().length() as Float / nsamples.sqrt() as Float);

        pupilbounds
    }

    fn focus_distance(&self, film_distance: Float) -> Float {
        // Find offset ray from film center through lens
        let bounds = self.bound_exit_pupil(0.0, 0.001 * self.film.diagonal);

        let scale_factors = [0.1, 0.01, 0.001];
        let mut lu = 0.0;
        let mut ray = Ray::default();

        // Try some different and decreasing scaling factor to find focus ray
        // more quickly when `aperturediameter` is too small.
        // (e.g. 2 [mm] for `aperturediameter` with wide.22mm.dat),
        let mut found_focusray = false;

        for scale in scale_factors.iter() {
            lu = scale * bounds.p_max[0];
            let r = Ray::new(
                &Point3f::new(0.0, 0.0, self.lens_rearz() - film_distance),
                &Vector3f::new(lu, 0.0, film_distance),
                INFINITY, 0.0, None, None);

            if self.trace_lenses_from_film(&r, Some(&mut ray)) {
                found_focusray = true;
                break;
            }
        }

        if !found_focusray {
            error!(
                "Focus ray at lens pos({}, 0) didn't make it through the lenses \
                with film distance {}?!??", lu, film_distance);
            return INFINITY;
        }

        // Compute distance zfocus where ray intersects the principal axis
        let tfocus = -ray.o.x / ray.d.x;
        let mut zfocus = ray.find_point(tfocus).z;

        if zfocus < 0.0 { zfocus = INFINITY; }

        zfocus
    }

    fn focus_binary_search(&self, focal_distance: Float) -> Float {
        // Find film_distance_lower, film_distance_upper that bound docus distance
        let mut film_distance_lower = self.focus_thicklens(focal_distance);
        let mut film_distance_upper = film_distance_lower;

        while self.focus_distance(film_distance_lower) > focal_distance {
            film_distance_lower *= 1.005;
        }
        while self.focus_distance(film_distance_upper) < focal_distance {
            film_distance_upper /= 1.005
        }

        // Do binary search on film distance to focus
        for _i in 0..20 {
            let fmid = 0.5 * (film_distance_lower + film_distance_upper);
            let mid_focus = self.focus_distance(fmid);

            if mid_focus < focal_distance {
                film_distance_lower = fmid;
            } else {
                film_distance_upper = fmid;
            }
        }

        0.5 * (film_distance_lower + film_distance_upper)
    }

    fn sample_exit_pupil(&self, pfilm: &Point2f, lens_sample: &Point2f, sba: Option<&mut Float>) -> Point3f {
        // Find exit pupil bound for sample distance from film center
        let rfilm = (pfilm.x * pfilm.x + pfilm.y * pfilm.y).sqrt();
        let mut rindex = (rfilm / (self.film.diagonal / 2.0) * self.exit_pupil_bounds.len() as Float) as usize;
        rindex = std::cmp::min(self.exit_pupil_bounds.len() - 1, rindex);
        let pupil_bounds = self.exit_pupil_bounds[rindex];

        if let Some(b) = sba {
            *b = pupil_bounds.area();
        }

        // Generate sample point inside exit pupil bound
        let plens = pupil_bounds.lerp(lens_sample);

        // Return sample point rotated by angle of pfilm with +x axis
        let sin_theta = if rfilm != 0.0 { pfilm.y } else { 0.0 };
        let cos_theta = if rfilm != 0.0 { pfilm.x } else { 1.0 };

        Point3f::new(
            cos_theta * plens.x - sin_theta * plens.y,
            sin_theta * plens.x + cos_theta * plens.y,
            self.lens_rearz())
    }
}

impl Camera for RealisticCamera {
    get_camera_data!();

    fn generate_ray(&self, sample: &CameraSample, r: &mut Ray) -> f32 {
        // TODO: ProfilePhase
        vignetted_total::inc_den();

        // Find point on film, pfilm, corresponding to sample.pfilm
        let s = Point2f::new(
            sample.pfilm.x / self.film.full_resolution.x as Float,
            sample.pfilm.y / self.film.full_resolution.y as Float);
        let pfilm2 = self.film.get_physical_extent().lerp(&s);
        let pfilm = Point3f::new(-pfilm2.x, pfilm2.y, 0.0);

        // Trace ray from pfilm through lens system
        let mut exit_pupil_boundsarea: Float = 0.0;
        let prear = self.sample_exit_pupil(
            &Point2f::new(pfilm.x, pfilm.y),
            &sample.plens,
            Some(&mut exit_pupil_boundsarea));
        let rfilm = Ray::new(
            &pfilm, &(prear - pfilm), INFINITY,
            lerp(sample.time, self.shutter_open, self.shutter_close),
            None, None);

        if !self.trace_lenses_from_film(&rfilm, Some(r)) {
            vignetted_total::inc_num();
            return 0.0;
        }

        // Finish initialization of RealisticCamera ray

        *r = self.camera_to_world.transform_ray(r);
        r.d = r.d.normalize();
        r.medium = self.medium.clone();

        // Return weighting for RealisticCamera ray
        let cos_theta = rfilm.d.normalize().z;
        let cos4_theta = (cos_theta * cos_theta) * (cos_theta * cos_theta);

        if self.simple_weighting {
            cos4_theta * exit_pupil_boundsarea / self.exit_pupil_bounds[0].area()
        } else {
            (self.shutter_close - self.shutter_open) *
            (cos4_theta * exit_pupil_boundsarea) / (self.lens_rearz() * self.lens_rearz())
        }
    }
}

pub fn create_realistic_camera(
    params: &ParamSet, cam2world: AnimatedTransform, film: Arc<Film>,
    medium: Option<Arc<Mediums>>) -> Option<Arc<Cameras>> {
    let mut shutteropen = params.find_one_float("shutteropen", 0.0);
    let mut shutterclose = params.find_one_float("shutterclose", 1.0);

    if shutterclose < shutteropen {
        warn!(
            "Shutter close time [{}] < shutter open [{}]. swapping them.",
            shutterclose, shutteropen);
        std::mem::swap(&mut shutteropen, &mut shutterclose)
    }

    // Realistic camera-specific parameters
    let lensfile = params.find_one_filename("lensfile", "");
    let aperture = params.find_one_float("aperturediameter", 1.0);
    let focusdis = params.find_one_float("focusdistance", 10.0);
    let sweighting = params.find_one_bool("simpleweighting", true);

    if lensfile.is_empty() {
        error!("No lens description file supplied");

        return None;
    }

    let mut lensdata = Vec::new();

    if let Err(e) = read_float_file(&lensfile, &mut lensdata) {
        error!("Error reading lens specification file \"{}\": {}", lensfile, e);

        return None;
    }

    if lensdata.len() % 4 != 0 {
        error!(
            "Excess values in lens specification file \"{}\"; \
            must be multiple-of-four values, read {}.",
            lensfile, lensdata.len());
    }

    let cam: Cameras = RealisticCamera::new(
        cam2world, shutteropen, shutterclose,
        aperture, focusdis, sweighting,
        &lensdata, film, medium).into();

    Some(Arc::new(cam))
}