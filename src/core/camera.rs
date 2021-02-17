use enum_dispatch::enum_dispatch;
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::Float;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{InteractionData};
use crate::core::geometry::vector::Vector3f;
use crate::core::light::VisibilityTester;
use crate::core::spectrum::Spectrum;
use crate::cameras::orthographic::OrthographicCamera;
use crate::cameras::perspective::PerspectiveCamera;
use crate::cameras::environment::EnvironmentCamera;
use crate::cameras::realistic::RealisticCamera;
use bumpalo::core_alloc::sync::Arc;
use crate::core::film::Film;
use crate::core::medium::Mediums;
use std::fmt::{Display, Result, Formatter};

#[macro_export]
macro_rules! get_camera_data {
    () => {
        fn film(&self) -> Arc<Film> {
            self.film.clone()
        }

        fn shutter_open(&self) -> Float {
            self.shutter_open
        }

        fn shutter_close(&self) -> Float {
            self.shutter_close
        }

        fn medium(&self) -> Option<Arc<Mediums>> {
            self.medium.clone()
        }
    }
}

#[enum_dispatch(Cameras)]
pub trait Camera {
    fn generate_ray(&self, sample: &CameraSample, r: &mut Ray) -> Float;

    fn generate_ray_differential(&self, sample: &CameraSample, rd: &mut Ray) -> Float {
        let wt = self.generate_ray(sample, rd);

        if wt == 0.0 { return 0.0; }
        let mut wtx = 0.0;
        let mut wty = 0.0;

        // find camera ray after shifting a fraction of a pixel in the x direction
        for eps in [0.05, -0.05].iter() {
            let mut sshift = *sample;
            sshift.pfilm.x += eps;
            let mut rx = Ray::default();
            wtx = self.generate_ray(&sshift, &mut rx);

            if let Some(ref mut diff) = rd.diff {
                diff.rx_origin = rd.o + (rx.o - rd.o) / *eps;
                diff.rx_direction = rd.d + (rx.d - rd.d) / *eps;

                if wtx != 0.0 {
                    break;
                }
            }
        }

        if wtx == 0.0 {
            return 0.0;
        }

        // find camera ray after shifting a fraction of a pixel in the y direction
        for eps in [0.05, -0.05].iter() {
            let mut sshift = *sample;
            sshift.pfilm.y += eps;
            let mut rx = Ray::default();
            wty = self.generate_ray(&sshift, &mut rx);

            if let Some(ref mut diff) = rd.diff {
                diff.ry_origin = rd.o + (rx.o - rd.o) / *eps;
                diff.ry_direction = rd.d + (rx.d - rd.d) / *eps;

                if wtx != 0.0 {
                    break;
                }
            }
        }

        if wty == 0.0 {
            return 0.0;
        }

        if let Some(ref mut diff) = rd.diff {
            diff.has_differentials = true;
        }

        wt
    }

    fn we(&self, _ray: &Ray, _p_raster2: Option<&mut Point2f>) -> Spectrum {
        panic!("Camera::we() is not implemented")
    }

    fn pdf_we(&self, _ray: &Ray, _pdf_pos: &mut Float, _pdf_dir: &mut Float) {
        panic!("Camera::pdf_we() is not implemented");
    }

    fn sample_wi(
        &self, _r: &InteractionData, _u: &Point2f, _wi: &mut Vector3f, _pdf: &mut Float,
        _p_raster: &mut Point2f, _vis: &mut VisibilityTester) -> Spectrum {
        panic!("Camera::sample_wi() is not implemeted");
    }

    fn film(&self) -> Arc<Film>;

    fn shutter_open(&self) -> Float;

    fn shutter_close(&self) -> Float;

    fn medium(&self) -> Option<Arc<Mediums>>;

}

#[derive(Default, Debug, Copy, Clone)]
pub struct CameraSample {
    pub pfilm: Point2f,
    pub plens: Point2f,
    pub time : Float
}

impl Display for CameraSample {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f,
            "[ pFilm: {}, pLens: {}, time {}]",
            self.pfilm, self.plens, self.time
        )
    }
}

#[enum_dispatch]
pub enum Cameras {
    OrthographicCamera,
    PerspectiveCamera,
    RealisticCamera,
    EnvironmentCamera
}