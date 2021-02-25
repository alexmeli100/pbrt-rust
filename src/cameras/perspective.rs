use crate::core::pbrt::{Float, INFINITY, lerp, PI};
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::{Transform, AnimatedTransform};
use std::sync::Arc;
use crate::core::film::Film;
use crate::core::medium::{Mediums, MediumInterface};
use crate::core::geometry::bounds::Bounds2f;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::camera::{Camera, CameraSample, Cameras};
use crate::core::light::VisibilityTester;
use crate::core::geometry::ray::{Ray, RayDifferential};
use crate::core::interaction::{
    InteractionData, Interaction};
use crate::core::spectrum::Spectrum;
use crate::get_camera_data;
use crate::core::sampling::concentric_sample_disk;
use crate::core::paramset::ParamSet;
use log::{warn, error};
use crate::core::geometry::normal::Normal3f;

#[derive(Clone)]
pub struct PerspectiveCamera {
    pub shutter_open        : Float,
    pub shutter_close       : Float,
    pub dx_camera           : Vector3f,
    pub dy_camera           : Vector3f,
    pub camera_to_screen    : Transform,
    pub camera_to_world     : AnimatedTransform,
    pub raster_to_camera    : Transform,
    pub screen_to_raster    : Transform,
    pub raster_to_screen    : Transform,
    pub lens_radius         : Float,
    pub focal_distance      : Float,
    pub film                : Arc<Film>,
    pub medium              : Option<Arc<Mediums>>,
    pub a                   : Float
}

impl PerspectiveCamera {
    pub fn new(
        camera_to_world: AnimatedTransform, screen_window: &Bounds2f, shutter_open: Float,
        shutter_close: Float, lens_radius: Float, focal_distance: Float, fov: Float,
        film: Arc<Film>, medium: Option<Arc<Mediums>>) -> Self {
        let res = film.full_resolution;

        let camera_to_screen = Transform::perspective(fov, 1e-2, 1000.0);

        // Compute projective camera transformations

        // Compute projective camera screen transformations

        let screen_to_raster =
            Transform::scale(film.full_resolution.x as Float, film.full_resolution.y as Float, 1.0) *
            Transform::scale(
                1.0 / (screen_window.p_max.x - screen_window.p_min.x),
                1.0 / (screen_window.p_min.y - screen_window.p_max.y),
                1.0) *
            Transform::translate(&Vector3f::new(-screen_window.p_min.x, -screen_window.p_max.y, 0.0));


        let raster_to_screen = Transform::inverse(&screen_to_raster);
        let raster_to_camera = Transform::inverse(&camera_to_screen) * raster_to_screen;

        // Compute differential changes in origin for perspective camera rays
        let p1 = Point3f::new(1.0, 0.0, 0.0);
        let p2 = Point3f::new(0.0, 0.0, 0.0);
        let p2_trans = raster_to_camera.transform_point(&p2);
        let p3 = Point3f::new(0.0, 1.0, 0.0);
        let dx_camera = (raster_to_camera.transform_point(&p1)) - p2_trans;
        let dy_camera = (raster_to_camera.transform_point(&p3)) - p2_trans;

        // Compute image plane bounds at z = 1 for PerspectiveCamera

        let mut pmax = raster_to_camera.transform_point(&Point3f::new(res.x as Float, res.y as Float, 0.0));
        let mut pmin = p2_trans;
        pmin /= pmin.z;
        pmax /= pmax.z;
        let a = ((pmax.x - pmin.x) * (pmax.y - pmin.y)).abs();

        PerspectiveCamera {
            a, film, medium, shutter_open, shutter_close, camera_to_world,
            camera_to_screen, dx_camera, dy_camera, focal_distance,
            lens_radius, screen_to_raster, raster_to_camera, raster_to_screen,
        }
    }
}

impl Camera for PerspectiveCamera {
    get_camera_data!();

    fn generate_ray(&self, sample: &CameraSample, r: &mut Ray) -> f32 {
        // Compute raster and camera sample positions
        let pfilm = Point3f::new(sample.pfilm.x, sample.pfilm.y, 0.0);
        let pcamera = self.raster_to_camera.transform_point(&pfilm);
        let o = Point3f::new(0.0, 0.0, 0.0);
        let d = Vector3f::from(pcamera).normalize();
        *r = Ray::new(&o, &d, INFINITY, 0.0, None, None);

        // Modify ray for depth of field
        if self.lens_radius > 0.0 {
            // Sample point on lens
            let plens =  concentric_sample_disk(&sample.plens) * self.lens_radius;

            // Compute point on plane of focus
            let ft = self.focal_distance / r.d.z;
            let pfocus = r.find_point(ft);

            // Update ray for effect of lens
            r.o = Point3f::new(plens.x, plens.y, 0.0);
            r.d = (pfocus - r.o).normalize();
        }

        r.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        r.medium = self.medium.clone();
        *r = self.camera_to_world.transform_ray(r);

        1.0
    }

    fn generate_ray_differential(&self, sample: &CameraSample, rd: &mut Ray) -> f32 {
        // Compute main orthographic viewing ray

        // Compute raster and camera sample positions
        let pfilm = Point3f::new(sample.pfilm.x, sample.pfilm.y, 0.0);
        let pcamera = self.raster_to_camera.transform_point(&pfilm);
        let o = Point3f::new(0.0, 0.0, 0.0);
        let d = Vector3f::from(pcamera).normalize();
        *rd = Ray::new(&o, &d, INFINITY, 0.0, None, None);

        // Modify ray for depth of field
        if self.lens_radius > 0.0 {
            // Sample point on lens
            let plens =  concentric_sample_disk(&sample.plens) * self.lens_radius;

            // Compute point on plane of focus
            let ft = self.focal_distance / rd.d.z;
            let pfocus = rd.find_point(ft);

            // Update ray for effect of lens
            rd.o = Point3f::new(plens.x, plens.y, 0.0);
            rd.d = (pfocus - rd.o).normalize();
        }

        let mut diff = RayDifferential::default();
        diff.has_differentials = true;

        // Compute ray differentials for OrthographicCamera
        if self.lens_radius > 0.0 {
            // Compute OrthographicCamera ray differentials accounting for lens

            // Sample point on lens
            let plens =  concentric_sample_disk(&sample.plens) * self.lens_radius;

            let dx = Vector3f::from(pcamera + self.dx_camera).normalize();
            let mut ft = self.focal_distance / dx.z;
            let mut pfocus = Point3f::new(0.0, 0.0, 0.0) + (dx * ft);
            diff.rx_origin = Point3f::new(plens.x, plens.y, 0.0);
            diff.rx_direction = (pfocus - diff.rx_origin).normalize();

            let dy = Vector3f::from(pcamera + self.dy_camera).normalize();
            ft = self.focal_distance / dy.z;
            pfocus = Point3f::new(0.0, 0.0, 0.0) + (dy * ft);
            diff.ry_origin = Point3f::new(plens.x, plens.y, 0.0);
            diff.ry_direction = (pfocus - diff.ry_origin).normalize();

        } else {
            diff.rx_origin = rd.o;
            diff.ry_origin = rd.o;
            diff.rx_direction = (Vector3f::from(pcamera) + self.dx_camera).normalize();
            diff.ry_direction = (Vector3f::from(pcamera) + self.dy_camera).normalize();
        }

        rd.diff = Some(diff);
        rd.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        rd.medium = self.medium.clone();
        *rd = self.camera_to_world.transform_ray(rd);

        1.0
    }

    fn we(&self, ray: &Ray, praster2: Option<&mut Point2f>) -> Spectrum {
        // Interpolate camera matrix and fail if w is not foward-facing
        let mut c2w = Transform::default();
        self.camera_to_world.interpolate(ray.time, &mut c2w);
        let cos_theta = ray.d.dot(&c2w.transform_vector(&Vector3f::new(0.0, 0.0, 1.0)));
        if cos_theta <= 0.0 { return Spectrum::new(0.0); }

        // Map ray (p, w) onto the raster grid
        let lr = self.lens_radius;
        let fd = self.focal_distance;
        let pfocus = ray.find_point((if lr > 0.0 { fd } else { 1.0 }) / cos_theta);
        let praster = Transform::inverse(&self.raster_to_camera)
            .transform_point(&Transform::inverse(&c2w).transform_point(&pfocus));

        // Return raster position if requested
        if let Some (rp) = praster2 {
            *rp = Point2f::new(praster.x, praster.y);
        }

        // Return zero importance for out of bounds points
        let sbounds = self.film.get_sample_bounds();
        if praster.x < sbounds.p_min.x as Float || praster.x >= sbounds.p_max.x as Float ||
           praster.y < sbounds.p_min.y as Float || praster.y >= sbounds.p_max.y as Float {
            return Spectrum::new(0.0);
        }

        // Compute lens area of perspective camera;
        let lens_area = if lr != 0.0 {
            PI * lr * lr
        } else {
            1.0
        };

        // Return importance for point on image plane
        let cos2_theta = cos_theta * cos_theta;

        Spectrum::new(1.0 / (self.a * lens_area * cos2_theta * cos2_theta))
    }

    fn pdf_we(&self, ray: &Ray, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // Interpolate camera matrix and fail if w is not foward-facing
        let mut c2w = Transform::default();
        self.camera_to_world.interpolate(ray.time, &mut c2w);
        let cos_theta = ray.d.dot(&c2w.transform_vector(&Vector3f::new(0.0, 0.0, 1.0)));
        if cos_theta <= 0.0 {
            *pdf_pos = 0.0;
            *pdf_dir = 0.0;
            return;
        }

        // Map ray (p, w) onto the raster grid
        let lr = self.lens_radius;
        let fd = self.focal_distance;
        let pfocus = ray.find_point((if lr > 0.0 { fd } else { 1.0 }) / cos_theta);
        let praster = Transform::inverse(&self.raster_to_camera)
            .transform_point(&Transform::inverse(&c2w).transform_point(&pfocus));

        // Return zero importance for out of bounds points
        let sbounds = self.film.get_sample_bounds();
        if praster.x < sbounds.p_min.x as Float || praster.x >= sbounds.p_max.x as Float ||
            praster.y < sbounds.p_min.y as Float || praster.y >= sbounds.p_max.y as Float {
            *pdf_pos = 0.0;
            *pdf_dir = 0.0;
            return;
        }

        // Compute lens area of perspective camera;
        let lens_area = if lr != 0.0 {
            PI * lr * lr
        } else {
            1.0
        };

        *pdf_pos = 1.0 / lens_area;
        *pdf_dir = 1.0 / (self.a * cos_theta * cos_theta * cos_theta)
    }

    fn sample_wi(
        &self, r: &InteractionData, u: &Point2f, wi: &mut Vector3f, pdf: &mut Float,
        p_raster: &mut Point2f, vis: &mut VisibilityTester) -> Spectrum {
        // Uniformly sample a lens interaction lensIntr
        let plens = concentric_sample_disk(u) * self.lens_radius;
        let p = Point3f::new(plens.x, plens.y, 0.0);
        let plens_world = self.camera_to_world.transform_point(r.time, &p);
        let v = Vector3f::new(0.0, 0.0 ,1.0);
        let n = Normal3f::from(self.camera_to_world.transform_vector(r.time, &v));
        let lens_intr = InteractionData {
            n,
            p: plens_world,
            time: r.time,
            medium_interface: Some(MediumInterface::new(self.medium.clone())),
            ..Default::default()
        };

        // Populate arguments and compute the importance value

        *wi = lens_intr.p - r.p;
        let dist = wi.length();
        *wi /= dist;

        // Compute PDF for importance arriving at r

        // Compute lens area of perspective camera
        let area = if self.lens_radius != 0.0 {
            PI * self.lens_radius * self.lens_radius
        } else {
            1.0
        };

        *pdf = (dist * dist) / lens_intr.n.abs_dot_vec(wi) * area;
        let ray = lens_intr.spawn_ray(&(-*wi));
        *vis = VisibilityTester::new(r.clone(), lens_intr);

        self.we(&ray, Some(p_raster))
    }
}

pub fn create_perspective_camera(
    params: &ParamSet, cam2world: AnimatedTransform,
    film: Arc<Film>, medium: Option<Arc<Mediums>>) -> Option<Arc<Cameras>> {
    // Extract common camera parameters from ParamSet
    let mut shutteropen = params.find_one_float("shutteropen", 0.0);
    let mut shutterclose = params.find_one_float("shutterclose", 1.0);

    if shutterclose < shutteropen {
        warn!("Shutter close time [{}] < shutter open [{}]. Swapping time.",
              shutterclose, shutteropen);
        std::mem::swap(&mut shutteropen, &mut shutterclose);
    }

    let lensradius = params.find_one_float("lensradius", 0.0);
    let focaldistance = params.find_one_float("focaldistance", 1.0e30);
    let frame = params.find_one_float(
        "frameaspectratio",
        film.full_resolution.x as Float / film.full_resolution.y as Float);

    let mut screen = Bounds2f::default();

    if frame > 1.0 {
        screen.p_min.x = -frame;
        screen.p_max.x = frame;
        screen.p_min.y = -1.0;
        screen.p_max.y = 1.0;
    } else {
        screen.p_min.x = -1.0;
        screen.p_max.x = 1.0;
        screen.p_min.y = -1.0 / frame;
        screen.p_max.y = 1.0 / frame;
    }

    let mut swi = 0;
    let sw = params.find_float("screenwindow", &mut swi);

    if let Some(ref s) = sw {
        if swi == 4 {
            screen.p_min.x = s[0];
            screen.p_max.x = s[1];
            screen.p_min.y = s[2];
            screen.p_max.y = s[3];
        } else {
            error!("\"screenwindow\" should have four values");
        }
    }

    let mut fov = params.find_one_float("fov", 90.0);
    let halffov = params.find_one_float("halffov", -1.0);
    if halffov > 0.5 {
        // hack for structure synth, which exports half of the full fov
        fov = 2.0 * halffov
    }

    let cam = PerspectiveCamera::new(
        cam2world, &screen, shutteropen, shutterclose,
        lensradius, focaldistance, fov, film, medium);

    Some(Arc::new(cam.into()))
}