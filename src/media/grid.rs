use crate::core::spectrum::Spectrum;
use crate::{stat_memory_counter, stat_ratio};
use crate::core::pbrt::{Float, lerp};
use crate::core::transform::Transform;
use crate::core::sampler::Sampler;
use std::sync::Arc;
use log::error;
use crate::core::medium::Medium;
use crate::core::sampler::Samplers;
use crate::core::interaction::MediumInteraction;
use crate::core::geometry::ray::Ray;
use crate::core::geometry::point::{Point3f, Point3i};
use crate::core::geometry::bounds::{Bounds3i, Bounds3f};
use crate::core::geometry::vector::Vector3i;

stat_memory_counter!("Memory/Volume density grid", density_bytes);
stat_ratio!("Media/Grid steps per Tr() call", ntr_steps_per_calls);

pub fn init_stats() {
    density_bytes::init();
    ntr_steps_per_calls::init();
}

pub struct GridDensityMedium {
    sigma_a         : Spectrum,
    sigma_s         : Spectrum,
    g               : Float,
    nx              : usize,
    ny              : usize,
    nz              : usize,
    w2m             : Transform,
    density         : Arc<Vec<Float>>,
    sigma_t         : Float,
    inv_max_density : Float

}

impl GridDensityMedium {
    pub fn new(
        sigma_a: &Spectrum, sigma_s: &Spectrum, g: Float,
        nx: usize, ny: usize, nz: usize,
        m2w: &Transform, d: Arc<Vec<Float>>) -> Self {
        density_bytes::add((nx * ny * nz * std::mem::size_of::<Float>()) as u64);

        // Precompute values for Monte Carlo sampling of GridDensityMedium
        let s = *sigma_a + *sigma_s;
        let sigma_t = s[0];

        if Spectrum::new(sigma_t) != s {
            error!("GridDensityMedium requires spectrally uniform \
            attenuation coefficient");
        }

        let mut maxd = 0 as Float;

        for i in 0.. nx * ny * nz {
            maxd = maxd.max(d[i]);
        }

        let inv_maxd = 1.0 / maxd;

        Self {
            g,
            nx, ny, nz,
            sigma_t,
            density         : d,
            sigma_a         : *sigma_a,
            sigma_s         : *sigma_s,
            inv_max_density : inv_maxd,
            w2m             : Transform::inverse(m2w)
        }
    }

    pub fn density(&self, p: &Point3f) -> Float {
        // Compute voxel coordinate and offsets for p
        let psamples = Point3f::new(
            p.x * self.nx as Float - 0.5,
            p.y * self.ny as Float - 0.5,
            p.z * self.nz as Float - 0.5
        );
        let pi = Point3i::from(psamples);
        let d = psamples - Point3f::from(pi);

        // Trilinearly interpolate density values to compute local density
        let d00 = lerp(d.x, self.d(&pi), self.d(&(pi + Vector3i::new(1, 0, 0))));
        let d10 = lerp(d.x, self.d(&(pi + Vector3i::new(0, 1, 0))), self.d(&(pi + Vector3i::new(1, 1, 0))));
        let d01 = lerp(d.x, self.d(&(pi + Vector3i::new(0, 0, 1))), self.d(&(pi + Vector3i::new(1, 0, 1))));
        let d11 = lerp(d.x, self.d(&(pi + Vector3i::new(0, 1, 1))), self.d(&(pi + Vector3i::new(1, 1, 1))));
        let d0 = lerp(d.y, d00, d10);
        let d1 = lerp(d.y, d01, d11);

        lerp(d.z, d0, d1)


    }

    fn d(&self, p: &Point3i) -> Float {
        let bounds = Bounds3i::new(
            Point3i::new(0, 0, 0),
            Point3i::new(self.nx as isize, self.ny as isize, self.nz as isize));

        if !bounds.inside_exclusive(p) { return 0.0; }

        self.density[(p.z as usize * self.ny + p.y as usize) * self.nx + p.x as usize]
    }
}

impl Medium for GridDensityMedium {
    fn tr(&self, ray: &Ray, sampler: &mut Samplers) -> Spectrum {
        // TODO: ProfilePhase
        ntr_steps_per_calls::inc_den();

        let r = self.w2m.transform_ray(
            &Ray::new(&ray.o, &ray.d.normalize(), ray.t_max * ray.d.length(), 0.0, None, None)
        );

        // Compute [tmin, tmax] interval of ray's overlap with medium bounds
        let b = Bounds3f::from_points(Point3f::new(0.0, 0.0, 0.0), Point3f::new(1.0, 1.0, 1.0));
        let mut tmin = 0.0;
        let mut tmax = 0.0;

        if !b.intersect_p(&r, &mut tmin, &mut tmax) { return Spectrum::new(1.0) }

        // Perform ratio tracking to estimate transmittance value
        let mut tr = 1.0;
        let mut t = tmin;

        loop {
            ntr_steps_per_calls::inc_num();
            t -= (1.0 - sampler.get_1d()).ln() * self.inv_max_density / self.sigma_t;

            if t >= tmax { break; }

            let density = self.density(&r.find_point(t));
            tr *= 1.0 - (0.0 as Float).max(density * self.inv_max_density);

            // When transmittance gets low, start applying Russian roulette to terminate sampling
            let rr_threshold = 0.1;

            if tr < rr_threshold {
                let q = (0.05 as Float).max(1.0 - tr);

                if sampler.get_1d() < q { return Spectrum::new(0.0); }
                tr /= 1.0 - q;
            }
        }

        Spectrum::new(tr)
    }

    fn sample(&self, ray: &Ray, sampler: &mut Samplers, mi: &mut MediumInteraction) -> Spectrum {
        unimplemented!()
    }
}