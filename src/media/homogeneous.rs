use crate::core::spectrum::Spectrum;
use crate::core::pbrt::{Float};
use crate::core::medium::Medium;
use crate::core::sampler::Samplers;
use crate::core::interaction::MediumInteraction;
use crate::core::geometry::ray::Ray;

#[derive(Debug)]
pub struct HomogeneousMedium {
    g       : Float,
    sigma_t : Spectrum,
    sigma_a : Spectrum,
    sigma_s : Spectrum
}

impl HomogeneousMedium {
    pub fn new(sigma_a: &Spectrum, sigma_s: &Spectrum, g: Float) -> Self {
        Self {
            g,
            sigma_a: *sigma_a,
            sigma_s: *sigma_s,
            sigma_t: *sigma_a + *sigma_s
        }
    }
}

impl Medium for HomogeneousMedium {
    fn tr(&self, ray: &Ray, sampler: &mut Samplers) -> Spectrum {
        // TODO: ProfilePhase
        (-self.sigma_t * (ray.t_max * ray.d.length()).min(f32::MAX)).exp()
    }

    fn sample(&self, ray: &Ray, sampler: &mut Samplers, mi: &mut MediumInteraction) -> Spectrum {
        unimplemented!()
    }
}