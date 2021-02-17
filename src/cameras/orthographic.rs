use crate::core::geometry::vector::Vector3f;
use crate::core::transform::{Transform, AnimatedTransform};
use crate::core::pbrt::{Float, INFINITY, lerp};
use crate::core::camera::{Camera, CameraSample, Cameras};
use crate::core::geometry::point::{Point3f};
use crate::core::geometry::ray::{Ray, RayDifferential};
use crate::core::geometry::bounds::Bounds2f;
use crate::core::film::Film;
use std::sync::Arc;
use crate::core::medium::Mediums;
use crate::get_camera_data;
use crate::core::sampling::concentric_sample_disk;
use crate::core::paramset::ParamSet;
use log::{warn, error};

#[derive(Clone)]
pub struct OrthographicCamera {
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
    pub medium              : Option<Arc<Mediums>>

}

impl OrthographicCamera {
    pub fn new(
        camera_to_world: AnimatedTransform, screen_window: &Bounds2f,
        shutter_open: Float, shutter_close: Float, lens_radius: Float,
        focal_distance: Float, film: Arc<Film>, medium: Option<Arc<Mediums>>
    ) -> Self {
        let camera_to_screen = Transform::orthographic(0.0, 1.0);

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

        let dx_camera = raster_to_camera.transform_vector(&Vector3f::new(1.0, 0.0, 0.0));
        let dy_camera = raster_to_camera.transform_vector(&Vector3f::new(0.0, 1.0, 0.0));

        OrthographicCamera {
            film, medium, shutter_open, shutter_close,
            dx_camera, dy_camera, camera_to_screen,
            camera_to_world, raster_to_screen, screen_to_raster,
            raster_to_camera, focal_distance, lens_radius,
        }
    }
}

impl Camera for OrthographicCamera {
    get_camera_data!();

    fn generate_ray(&self, sample: &CameraSample, r: &mut Ray) -> f32 {
        // Compute raster and camera sample positions
        let pfilm = Point3f::new(sample.pfilm.x, sample.pfilm.y, 0.0);
        let pcamera = self.raster_to_camera.transform_point(&pfilm);
        *r = Ray::new(&pcamera, &Vector3f::new(0.0, 0.0, 1.0), INFINITY, 0.0, None, None);

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
        *rd = Ray::new(&pcamera, &Vector3f::new(0.0, 0.0, 1.0), INFINITY, 0.0, None, None);

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
            let ft = self.focal_distance / rd.d.z;
            let mut pfocus = pcamera + self.dx_camera + (Vector3f::new(0.0, 0.0, 1.0) * ft);
            diff.rx_origin = Point3f::new(plens.x, plens.y, 0.0);
            diff.rx_direction = (pfocus - diff.rx_origin).normalize();

            pfocus = pcamera + self.dy_camera + (Vector3f::new(0.0, 0.0, 0.1) * ft);
            diff.ry_origin = Point3f::new(plens.x, plens.y, 0.0);
            diff.ry_direction = (pfocus - diff.ry_origin).normalize();

        } else {
            diff.rx_origin = rd.o + self.dx_camera;
            diff.ry_origin = rd.o + self.dy_camera;
            diff.rx_direction = rd.d;
            diff.ry_direction = rd.d;
        }

        rd.diff = Some(diff);
        rd.time = lerp(sample.time, self.shutter_open, self.shutter_close);
        rd.medium = self.medium.clone();
        *rd = self.camera_to_world.transform_ray(rd);

        1.0
    }
}

pub fn create_orthographic_camera(
    params: &ParamSet, cam2world: AnimatedTransform,
    film: Arc<Film>, medium: Option<Arc<Mediums>>) -> Option<Arc<Cameras>> {
    // Extract common camera parameters from ParamSet
    let mut shutteropen = params.find_one_float("shutteropen", 0.0);
    let mut shutterclose = params.find_one_float("shutterclose", 0.0);

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

    let cam = OrthographicCamera::new(
        cam2world, &screen, shutteropen, shutterclose,
        lensradius, focaldistance, film, medium);

    Some(Arc::new(cam.into()))
}