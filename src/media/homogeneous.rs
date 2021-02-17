use crate::core::spectrum::Spectrum;
use crate::core::pbrt::{Float};
use crate::core::medium::{Medium, Mediums, PhaseFunctions, HenyeyGreenstein, MediumInterface};
use crate::core::sampler::{Sampler};
use crate::core::interaction::MediumInteraction;
use crate::core::geometry::ray::Ray;
use std::sync::Arc;

#[derive(Debug, Clone, Copy)]
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
    fn tr<S: Sampler>(&self, ray: &Ray, _sampler: &mut S) -> Spectrum {
        // TODO: ProfilePhase
        (-self.sigma_t * (ray.t_max * ray.d.length()).min(f32::MAX)).exp()
    }

    fn sample<S: Sampler>(
        &self, ray: &Ray, sampler: &mut S,
        mi: &mut MediumInteraction) -> Spectrum {
        // Todo: ProfilePhase
        // Sample a channel and distance along the ray;
        let channel = std::cmp::min(
            sampler.get_1d() as usize * Spectrum::n(),
            Spectrum::n() - 1);
        let dist = -((1.0 - sampler.get_1d()).ln()) / self.sigma_t[channel];
        let t = (dist / ray.d.length()).min(ray.t_max);
        let sampled_medium = t < ray.t_max;

        if sampled_medium {
            let medium: Option<Arc<Mediums>> = Some(Arc::new((*self).into()));
            let pf: Option<PhaseFunctions> = Some(HenyeyGreenstein::new(self.g).into());
            let minter = Some(MediumInterface::new(medium));
            *mi = MediumInteraction::new(
                &ray.find_point(t), &(-ray.d),
                ray.time, minter, pf);
        }

        // Compute the transmittance and sampling for scattering from homogeneous medium
        let Tr = (-self.sigma_t * t.min(f32::MAX) * ray.d.length()).exp();

        // Return weighting factor for scattering from homogeneous medium
        let density = if sampled_medium { self.sigma_t * Tr } else { Tr };
        let mut pdf = 0.0;
        for i in 0..Spectrum::n() { pdf += density[i]; }
        pdf *= 1.0 / Spectrum::n() as Float;
        if pdf == 0.0 {
            assert!(Tr.is_black());
            pdf = 1.0;
        }

        if sampled_medium { Tr * self.sigma_s / pdf } else { Tr / pdf }
    }
}