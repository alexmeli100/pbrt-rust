use crate::core::pbrt::{Float, INFINITY, lerp};
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::{Transform, AnimatedTransform};
use std::sync::Arc;
use crate::init_projective_camera;
use crate::init_camera;
use crate::core::film::Film;
use crate::core::medium::Mediums;
use crate::core::geometry::bounds::Bounds2f;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::camera::{Camera, CameraSample};
use crate::core::light::VisibilityTester;
use crate::core::geometry::ray::{Ray, RayDifferential};
use crate::core::interaction::Interactions;
use crate::core::spectrum::Spectrum;
use crate::core::sampling::concentric_sample_disk;

#[derive(Default, Clone)]
pub struct PerspectiveCamera {
    pub shutter_open: Float,
    pub shutter_close: Float,
    pub dx_camera: Vector3f,
    pub dy_camera: Vector3f,
    pub camera_to_screen: Transform,
    pub camera_to_world: AnimatedTransform,
    pub raster_to_camera: Transform,
    pub screen_to_raster: Transform,
    pub raster_to_screen: Transform,
    pub lens_radius: Float,
    pub focal_distance: Float,
    pub film: Option<Arc<Film>>,
    pub medium: Option<Arc<Mediums>>,
    pub a: Float
}

impl PerspectiveCamera {
    fn new(
        camera_to_world: AnimatedTransform,
        screen_window: &Bounds2f,
        shutter_open: Float,
        shutter_close: Float,
        lens_radius: Float,
        focal_distance: Float,
        fov: Float,
        film: Option<Arc<Film>>,
        medium: Option<Arc<Mediums>>
    ) -> Self {
        let res = film.as_ref().unwrap().full_resolution;
        let mut camera = PerspectiveCamera::default();
        let camera_to_screen = Transform::perspective(fov, 1e-2, 1000.0);

        // Initialize projective camera data
        init_projective_camera!(
            camera, camera_to_world, camera_to_screen, screen_window,
            shutter_open, shutter_close, lens_radius, focal_distance, film, medium
        );

        // Compute differential changes in origin for perspective camera rays
        let p1 = Point3f::new(1.0, 0.0, 0.0);
        let p2 = Point3f::new(0.0, 0.0, 0.0);
        let p2_trans = camera.raster_to_camera.transform_point(&p2);
        let p3 = Point3f::new(0.0, 1.0, 0.0);
        camera.dx_camera = (camera.raster_to_camera.transform_point(&p1)) - p2_trans;
        camera.dy_camera = (camera.raster_to_camera.transform_point(&p3)) - p2_trans;

        // Compute image plane bounds at z = 1 for PerspectiveCamera

        let mut pmax = camera.raster_to_camera.transform_point(&Point3f::new(res.x as Float, res.y as Float, 0.0));
        let mut pmin = p2_trans;
        pmin /= pmin.z;
        pmax /= pmax.z;
        camera.a = ((pmax.x - pmin.x) * (pmax.y - pmin.y)).abs();

        camera
    }
}

impl Camera for PerspectiveCamera {
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

            let dx = Vector3f::from(pcamera + self.dx_camera);
            let mut ft = self.focal_distance / dx.z;
            let mut pfocus = Point3f::new(0.0, 0.0, 0.0) + (dx * ft);
            diff.rx_origin = Point3f::new(plens.x, plens.y, 0.0);
            diff.rx_direction = (pfocus - diff.rx_origin).normalize();

            let dy = Vector3f::from(pcamera + self.dy_camera);
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

    fn we(&self, ray: &Ray, p_raster2: Option<&[Point2f]>) {
        unimplemented!()
    }

    fn pdf_we(&self, ray: &Ray, pdf_pos: &[f32], pdf_dir: &[f32]) {
        unimplemented!()
    }

    fn sample_wi(&self, r: &Interactions, u: &Point2f, wi: &mut Vector3f, pdf: &mut [f32], p_raster: &mut [Point2f], vis: &mut VisibilityTester) -> Spectrum {
        unimplemented!()
    }
}