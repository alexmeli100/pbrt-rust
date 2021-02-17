use crate::core::transform::AnimatedTransform;
use crate::core::pbrt::{Float, PI, INFINITY, lerp};
use std::sync::Arc;
use log::{warn, error};
use crate::core::film::Film;
use crate::core::medium::Mediums;
use crate::core::camera::{Camera, CameraSample, Cameras};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::point::Point3f;
use crate::core::paramset::ParamSet;
use crate::core::geometry::bounds::Bounds2f;
use crate::get_camera_data;

pub struct EnvironmentCamera {
    pub camera_to_world     : AnimatedTransform,
    pub shutter_open        : Float,
    pub shutter_close       : Float,
    pub film                : Arc<Film>,
    pub medium              : Option<Arc<Mediums>>
}

impl EnvironmentCamera {
    pub fn new(
        camera_to_world: AnimatedTransform, shutter_open: Float,
        shutter_close: Float, film: Arc<Film>, medium: Option<Arc<Mediums>>) -> Self {
        Self {
            camera_to_world, shutter_open, shutter_close, film, medium
        }
    }
}

impl Camera for EnvironmentCamera {
    get_camera_data!();

    fn generate_ray(&self, sample: &CameraSample, r: &mut Ray) -> Float {
        // TODO: ProfilePhase
        let theta = PI * sample.pfilm.y / self.film.full_resolution.y as Float;
        let phi = 2.0 * PI * sample.pfilm.x / self.film.full_resolution.x as Float;
        let dir = Vector3f::new(
            theta.sin() * phi.cos(),
            theta.cos(),
            theta.sin() * phi.sin()
        );
        *r = Ray::new(&Point3f::new(0.0, 0.0, 0.0), &dir, INFINITY,
            lerp(sample.time, self.shutter_open, self.shutter_close), self.medium.clone(), None);

        *r = self.camera_to_world.transform_ray(r);

        1.0
    }
}

pub fn create_environment_camera(
    params: &ParamSet, cam2world: AnimatedTransform, film: Arc<Film>,
    medium: Option<Arc<Mediums>>) -> Option<Arc<Cameras>> {
    let mut shutteropen = params.find_one_float("shutteropen", 1.0);
    let mut shutterclose = params.find_one_float("shutterclose", 1.0);

    if shutterclose < shutteropen {
        warn!("Shutter close time [{}] < shutter open [{}]. Swapping time.",
            shutterclose, shutteropen);
        std::mem::swap(&mut shutteropen, &mut shutterclose);
    }

    let _lensradius = params.find_one_float("lensradius", 0.0);
    let _focaldistance = params.find_one_float("focaldistance", 1.0e30);
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

    let cam = EnvironmentCamera::new(
        cam2world, shutteropen,
        shutterclose, film, medium);

    Some(Arc::new(cam.into()))
}