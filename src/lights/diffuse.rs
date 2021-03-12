use crate::core::spectrum::Spectrum;
use crate::core::shape::{Shape, Shapes};
use std::sync::Arc;
use crate::core::pbrt::{Float, PI, Options};
use crate::core::medium::{MediumInterface};
use crate::core::transform::Transform;
use crate::core::light::{AreaLight, Light, VisibilityTester, LightFlags, Lights};
use crate::core::geometry::vector::{Vector3f, vec3_coordinate_system};
use crate::core::geometry::point::Point2f;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{Interaction, InteractionData};
use crate::core::geometry::normal::Normal3f;
use crate::init_light_data;
use log::warn;
use crate::core::paramset::ParamSet;
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::sampling::{cosine_sample_hemisphere, cosine_hemisphere_pdf};

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

    fn le(&self, _r: &Ray) -> Spectrum {
        Spectrum::new(0.0)
    }

    fn pdf_li(&self, re: &InteractionData, wi: &Vector3f) -> Float {
        // TODO: ProfilePhase
        self.shape.pdf_wi(re, wi)
    }

    fn sample_li(
        &self, re: &InteractionData, u: &Point2f, wi: &mut Vector3f,
        pdf: &mut Float, vis: &mut VisibilityTester) -> Spectrum {
        // TODO: ProfilePhase
        let mut pshape: InteractionData = self.shape.sample_interaction(re, u, pdf);
        pshape.medium_interface = Some(self.medium_interface.clone());

        if *pdf == 0.0 || (pshape.p - re.p).length_squared() == 0.0 {
            *pdf = 0.0;
            return Spectrum::new(0.0);
        }

        *wi = (pshape.p - re.p).normalize();
        let l = AreaLight::l(self, &pshape, &(-*wi));
        *vis = VisibilityTester::new(re.clone(), pshape);

        l
    }

    fn sample_le(
        &self, u1: &Point2f, u2: &Point2f, _time: Float, ray: &mut Ray,
        nlight: &mut Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) -> Spectrum {
        // TODO: ProfilePhase
        // Sample a point on the area light's Shape pshape
        let mut pshape: InteractionData = self.shape.sample(u1, pdf_pos);
        pshape.medium_interface = Some(self.medium_interface.clone());
        *nlight = pshape.n;

        // Sample a cosine-weighted outgoing direction w for area light
        let mut w: Vector3f;
        if self.two_sided {
            let mut u = *u2;
            // Choose a side to sample and then remap u[0] to [0,1] before
            // applying cosine-weighted hemisphere sampling for the chosen side.
            if u[0] < 0.5 {
                u[0] = (u[0] * 2.0).min(ONE_MINUS_EPSILON);
                w = cosine_sample_hemisphere(&u);
            } else {
                u[0] = ((u[0] - 0.5) * 2.0).min(ONE_MINUS_EPSILON);
                w = cosine_sample_hemisphere(&u);
                w.z *= -1.0;
            }

            *pdf_dir = 0.5 * cosine_hemisphere_pdf(w.z.abs());
        } else {
            w = cosine_sample_hemisphere(u2);
            *pdf_dir = cosine_hemisphere_pdf(w.z);
        }

        let mut v1 = Vector3f::default();
        let mut v2 = Vector3f::default();
        let n = Vector3f::from(pshape.n);
        vec3_coordinate_system(&n, &mut v1, &mut v2);
        w = v1 * w.x + v2 * w.y + n * w.z;
        *ray = pshape.spawn_ray(&w);

        AreaLight::l(self, &pshape, &w)
    }

    fn pdf_le(&self, ray: &Ray, n: &Normal3f, pdf_pos: &mut Float, pdf_dir: &mut Float) {
        // TODO: ProfilePhase
        let it = InteractionData::new(
            ray.o, *n, Default::default(), Vector3f::from(*n),
            ray.time, Some(self.medium_interface.clone()));
        *pdf_pos = self.shape.pdf(&it);
        *pdf_dir = if self.two_sided {
            0.5 * cosine_hemisphere_pdf(n.abs_dot_vec(&ray.d))
        } else {
            cosine_hemisphere_pdf(n.dot_vec(&ray.d))
        }
    }

    fn nsamples(&self) -> usize { self.nsamples }

    fn flags(&self) -> u8 {
        self.flags
    }

    fn l<I: Interaction>(&self, intr: &I, wi: &Vector3f) -> Spectrum {
        AreaLight::l(self, intr, wi)
    }
}

pub fn create_diffuse_arealight(
    l2w: &Transform, medium: MediumInterface,
    params: &ParamSet, shape: Arc<Shapes>,
    opts: &Options) -> Option<Arc<Lights>> {
    let L = params.find_one_spectrum("L", Spectrum::new(1.0));
    let sc = params.find_one_spectrum("scale", Spectrum::new(1.0));
    let ns = params.find_one_int("nsamples", 1);
    let mut nsamples = params.find_one_int("samples", ns);
    let twosided = params.find_one_bool("twosided", false);

    if opts.quick_render {
        nsamples = std::cmp::max(1, nsamples / 4);
    }

    let l = DiffuseAreaLight::new(
        l2w, medium, &(L * sc), nsamples as usize, shape, twosided);

    Some(Arc::new(l.into()))
}