use enum_dispatch::enum_dispatch;
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::Float;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::Interactions;
use crate::core::geometry::vector::Vector3f;
use crate::core::light::VisibilityTester;
use crate::core::spectrum::Spectrum;
use crate::cameras::orthographic::OrthographicCamera;
use crate::cameras::perspective::PerspectiveCamera;
use crate::core::transform::Transform;

#[macro_export]
macro_rules! init_camera {
    ($c:ident, $c_to_world:ident, $sopen:ident, $sclose:ident, $f:ident, $m:ident) => {{
        $c.camera_to_world = $c_to_world;
        $c.shutter_open = $sopen;
        $c.shutter_close = $sclose;
        $c.film = $f;
        $c.medium = $m;
    }}
}

#[macro_export]
macro_rules! init_projective_camera {
    (
        $c:ident, $c_to_world:ident, $c_to_screen:ident, $s_window:ident,
        $s_open:ident, $s_close:ident, $lensr:ident, $focald:ident, $f:ident, $m:ident
    ) => {{
        init_camera!($c, $c_to_world, $s_open, $s_close, $f, $m);

        // Initialize depth of field parameters
        $c.lens_radius = $lensr;
        $c.focal_distance = $focald;

        // Compute projective camera transformations

        // Compute projective camera screen transformations

        if let Some(ref f) = $c.film {
            $c.screen_to_raster =
                Transform::scale(f.full_resolution.x as Float, f.full_resolution.y as Float, 1.0) *
                Transform::scale(
                    1.0 / ($s_window.p_max.x - $s_window.p_min.x),
                    1.0 / ($s_window.p_min.y - $s_window.p_max.y),
                    1.0) *
                Transform::translate(&Vector3f::new(-$s_window.p_min.x, -$s_window.p_max.y, 0.0));
        }

        $c.raster_to_screen = Transform::inverse(&$c.screen_to_raster);
        $c.raster_to_camera = Transform::inverse(&$c_to_screen) * $c.raster_to_screen;
    }}
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

    fn we(&self, ray: &Ray, p_raster2: Option<&[Point2f]>);

    fn pdf_we(&self, ray: &Ray, pdf_pos: &[Float], pdf_dir: &[Float]);

    fn sample_wi(&self, r: &Interactions, u: &Point2f, wi: &mut Vector3f, pdf: &mut [Float], p_raster: &mut [Point2f], vis: &mut VisibilityTester) -> Spectrum;
}

#[derive(Default, Debug, Copy, Clone)]
pub struct CameraSample {
    pub pfilm: Point2f,
    pub plens: Point2f,
    pub time: Float
}

#[enum_dispatch]
pub enum Cameras {
    OrthographicCamera,
    PerspectiveCamera
}