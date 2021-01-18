use crate::core::spectrum::Spectrum;
use crate::core::shape::{Shape, Shapes};
use std::sync::Arc;
use crate::core::pbrt::{Float, PI};
use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::light::{AreaLight, Light, VisibilityTester, LightFlags};
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::point::Point2f;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interactions, Interaction};
use crate::core::geometry::normal::Normal3f;
use crate::init_light_data;
use log::warn;

#[derive(Clone)]
pub struct DiffuseAreaLight {
    lemit            : Spectrum,
    shape            : Arc<Shapes>,
    two_sided        : bool,
    area             : Float,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl DiffuseAreaLight {
    pub fn new(
        l2w: &Transform, mi: MediumInterface, lemit: &Spectrum,
        nsamples: usize, shape: Arc<Shapes>, two_sided: bool) -> Self {
        let area = shape.area();
        let mut dl = DiffuseAreaLight {
            area,
            shape,
            nsamples,
            two_sided,
            lemit           : *lemit,
            flags           : Default::default(),
            medium_interface: Default::default(),
            light_to_world  : Default::default(),
            world_to_light  : Default::default()
        };

        // Initalize Light data
        let flags = LightFlags::Area as u8;
        init_light_data!(dl, flags, 1, mi, l2w);

        // Warn if light has transformation with non-uniform scale, though not
        // for Triangles, since this doesn't matter for them.
        match dl.shape.as_ref() {
            Shapes::Triangle(_)                => (),
            _ if dl.world_to_light.has_scale() => {
                warn!("Scaling detected in world to light transformation! \
                The system has numerous assumptions, implicit and explicit, \
                that this transform will have no scale factors in it. \
                Proceed at your own risk; your image may have errors.")
            }
            _                                  => ()
        }

        dl
    }
}

impl AreaLight for DiffuseAreaLight {
    fn l<I: Interaction>(&self, intr: &I, w: &Vector3f) -> Spectrum {
        if self.two_sided || intr.n().dot_vec(w) > 0.0 {
            self.lemit
        } else {
            Spectrum::new(0.0)
        }
    }
}

impl Light for DiffuseAreaLight {
    fn power(&self) -> Spectrum {
        self.lemit * self.area * PI
    }

    fn le(&self, r: &Ray) -> Spectrum {
        unimplemented!()
    }

    fn pdf_li(&self, re: &Interactions, wi: &Vector3f) -> f32 {
        unimplemented!()
    }

    fn sample_li(
        &self, re: &Interactions, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut f32, vis: &mut VisibilityTester) -> Spectrum {
        unimplemented!()
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, time: f32, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) -> Spectrum {
        unimplemented!()
    }

    fn pdf_le(&self, ray: &Ray, nlight: &Normal3f, pdf_pos: &mut f32, pdf_dir: &mut f32) {
        unimplemented!()
    }
}