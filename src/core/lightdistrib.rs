use crate::core::geometry::point::{Point3f, Point3i, Point2f};
use crate::core::sampling::Distribution1D;
use std::sync::Arc;
use atom::AtomSetOnce;
use log::info;
use std::sync::atomic::{AtomicU64, Ordering};
use crate::core::scene::Scene;
use crate::core::integrator::compute_light_power_distribution;
use crate::core::pbrt::{Float, clamp};
use crate::{stat_counter, stat_ratio, stat_int_distribution};
use crate::core::geometry::bounds::Bounds3f;
use crate::core::lowdiscrepancy::radical_inverse;
use crate::core::interaction::InteractionData;
use crate::core::geometry::normal::Normal3f;
use crate::core::medium::MediumInterface;
use crate::core::geometry::vector::Vector3f;
use crate::core::light::{VisibilityTester, Light};
use enum_dispatch::enum_dispatch;
use log::error;

pub fn create_light_sample_distribution(name: &str, scene: &Scene) -> Option<Arc<LightDistributions>> {
        if name == "uniform" || scene.lights.len() == 1 {
            Some(Arc::new(UniformLightDistribution::new(scene).into()))
        } else if name == "power" {
            Some(Arc::new(PowerLightDistribution::new(scene).into()))
        } else if name == "spatial" {
            Some(Arc::new(SpatialLightDistribution::new(scene, 64).into()))
        } else {
            error!("Light sample distribution type \"{}\" unknown. Using \"spatial\".", name);
            Some(Arc::new(SpatialLightDistribution::new(scene, 64).into()))
        }
}

#[enum_dispatch]
pub trait LightDistribution {
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D>;
}

#[enum_dispatch(LightDistribution)]
pub enum LightDistributions {
    UniformLightDistribution,
    PowerLightDistribution,
    SpatialLightDistribution,
}

pub struct UniformLightDistribution {
    distrib: Arc<Distribution1D>
}

impl UniformLightDistribution {
    pub fn new(scene: &Scene) -> Self {
        let prob = vec![1.0; scene.lights.len()];
        let distrib = Arc::new(Distribution1D::new(prob));

        Self { distrib }
    }
}

impl LightDistribution for UniformLightDistribution {
    fn lookup(&self, _p: &Point3f) -> Arc<Distribution1D> {
        self.distrib.clone()
    }
}

pub struct  PowerLightDistribution {
    distrib: Option<Arc<Distribution1D>>
}

impl PowerLightDistribution {
    pub fn new(scene: &Scene) -> Self {
        Self {
            distrib: compute_light_power_distribution(scene)
        }
    }
}

impl LightDistribution for PowerLightDistribution {
    fn lookup(&self, _p: &Point3f) -> Arc<Distribution1D> {
        match self.distrib {
            Some(ref d) => d.clone(),
            _       => Arc::new(Distribution1D::new(Vec::new()))
        }
    }
}

stat_counter!("SpatialLightDistribution/Distributions created", ncreated);
stat_ratio!("SpatialLightDistribution/Lookups per distribution", nlookups_distrib);
stat_int_distribution!("SpatialLightDistribution/Hash probes per lookup", nprobes_per_lookup);

pub fn init_stats() {
    ncreated::init();
    nlookups_distrib::init();
    nprobes_per_lookup::init();
}

// Voxel coordinates are packed into a uint64_t for hash table lookups;
// 10 bits are allocated to each coordinate.  invalidPackedPos is an impossible
// packed coordinate value, which we use to represent
const INVALID_PACKED_POS: u64 = 0xffff_ffff_ffff_ffff;

struct HashEntry {
    packed_pos      : AtomicU64,
    distribution    : AtomSetOnce<Arc<Distribution1D>>
}

pub struct SpatialLightDistribution {
    scene       : Scene,
    nvoxels     : [usize; 3],
    hash_table  : Box<[HashEntry]>,
    table_size  : usize,
}

impl SpatialLightDistribution {
    pub fn new(scene: &Scene, max_voxels: usize) -> Self {
        // Compute the number of voxels so that the widest scene bounding box
        // dimension has maxVoxels voxels and the other dimensions have a number
        // of voxels so that voxels are roughly cube shaped.
        let b = scene.wb;
        let diag = b.diagonal();
        let bmax = diag[b.maximum_extent()];
        let mut nvoxels = [0; 3];

        for i in 0..3 {
            nvoxels[i] = std::cmp::max(1, (diag[i] / bmax * max_voxels as Float).round() as usize);
            // In the Lookup() method, we require that 20 or fewer bits be
            // sufficient to represent each coordinate value. It's fairly hard
            // to imagine that this would ever be a problem.
            assert!(nvoxels[i] < 1 << 20);
        }

        let table_size = 4 * nvoxels[0] * nvoxels[1] * nvoxels[2];
        let mut hash_table = Vec::with_capacity(table_size);

        for _ in 0..table_size {
            let entry = HashEntry{
                packed_pos  : AtomicU64::new(INVALID_PACKED_POS),
                distribution: AtomSetOnce::empty()
            };
            hash_table.push(entry)
        }

        info!(
            "SpatialLightDistribution: scene bounds {} , voxel res ({}, {}, {})",
            b, nvoxels[0], nvoxels[1], nvoxels[2]);

        Self {
            nvoxels,
            table_size,
            scene: scene.clone(),
            hash_table: hash_table.into_boxed_slice(),
        }
    }

    fn compute_dsitribution(&self, pi: Point3i) -> Arc<Distribution1D> {
        // TODO: ProfilePhase
        ncreated::inc();
        nlookups_distrib::inc_den();

        // Compute the world-space bounding box of the voxel corresponding to
        // |pi|.
        let p0 = Point3f::new(
            pi[0] as Float / self.nvoxels[0] as Float,
            pi[1] as Float / self.nvoxels[1] as Float,
            pi[2] as Float / self.nvoxels[2] as Float);
        let p1 = Point3f::new(
            (pi[0] + 1) as Float / self.nvoxels[0] as Float,
            (pi[1] + 1) as Float / self.nvoxels[1] as Float,
            (pi[2] + 1) as Float / self.nvoxels[2] as Float);
        let voxel_bounds = Bounds3f::from_points(
            self.scene.wb.lerp(&p0),
            self.scene.wb.lerp(&p1));

        // Compute the sampling distribution. Sample a number of points inside
        // voxelBounds using a 3D Halton sequence; at each one, sample each
        // light source and compute a weight based on Li/pdf for the light's
        // sample (ignoring visibility between the point in the voxel and the
        // point on the light source) as an approximation to how much the light
        // is likely to contribute to illumination in the voxel.
        let nsamples = 128;
        let mut light_contrib = vec![0.0; self.scene.lights.len()];

        for i in 0..nsamples {
            let p = Point3f::new(
                radical_inverse(0, i),
                radical_inverse(1, i),
                radical_inverse(2, i));
            let po = voxel_bounds.lerp(&p);
            let intr = InteractionData::new(
                po, Normal3f::default(), Vector3f::default(),
                Vector3f::new(1.0, 0.0, 0.0), 0.0, Some(MediumInterface::default()));

            // Use the next two Halton dimensions to sample a point on the
            // light source.
            let u = Point2f::new(radical_inverse(2, i), radical_inverse(4, i));
            for j in 0..self.scene.lights.len() {
                let mut pdf = 0.0;
                let mut wi = Vector3f::default();
                let mut vis = VisibilityTester::default();
                let Li = self.scene.lights[j].sample_li(&intr, &u, &mut wi, &mut pdf, &mut vis);

                if pdf > 0.0 {
                    // TODO: look at tracing shadow rays / computing beam
                    // transmittance.  Probably shouldn't give those full weight
                    // but instead e.g. have an occluded shadow ray scale down
                    // the contribution by 10 or something.
                    light_contrib[j] += Li.y() / pdf;
                }
            }
        }

        // We don't want to leave any lights with a zero probability; it's
        // possible that a light contributes to points in the voxel even though
        // we didn't find such a point when sampling above.  Therefore, compute
        // a minimum (small) weight and ensure that all lights are given at
        // least the corresponding probability.
        let sum_contrib: Float = light_contrib.iter().sum();
        let avg_contrib = sum_contrib / (nsamples as Float * light_contrib.len() as Float);
        let min_contrib = if avg_contrib > 0.0 { 0.001 * avg_contrib } else { 1.0};

        for (i, l) in light_contrib.iter_mut().enumerate() {
            info!("Voxel pi = {}, light = {}, contrib = {}", pi, i, l);
            *l = (*l).max(min_contrib);
        }

        info!(
            "Initialized light distribution in voxel \
             pi= {}, avgContrib = {}", pi, avg_contrib);

        Arc::new(Distribution1D::new(light_contrib))
    }
}

impl LightDistribution for SpatialLightDistribution {
    fn lookup(&self, p: &Point3f) -> Arc<Distribution1D> {
        // TODO: ProfilePhase
        nlookups_distrib::inc_num();

        // First, compute integer voxel coordinates for the given point |p|
        // with respect to the overall voxel grid.
        let offset = self.scene.wb.offset(p); // offset in [0, 1]
        let mut pi = Point3i::default();

        for i in 0..3 {
            // The clamp should almost never be necessary, but is there to be
            // robust to computed intersection points being slightly outside
            // the scene bounds due to floating-point roundoff error.
            pi[i] = clamp((offset[i] * self.nvoxels[i] as Float) as isize, 0, (self.nvoxels[i] - 1) as isize);
        }

        // Pack the 3D integer voxel coordinates into a single 64-bit value.
        let packedpos = (pi[0] << 40) as u64 | (pi[1] << 20) as u64 | pi[2] as u64;
        assert_ne!(packedpos, INVALID_PACKED_POS);

        // Compute a hash value from the packed voxel coordinates.  We could
        // just take packedPos mod the hash table size, but since packedPos
        // isn't necessarily well distributed on its own, it's worthwhile to do
        // a little work to make sure that its bits values are individually
        // fairly random. For details of and motivation for the following, see:
        // http://zimbry.blogspot.ch/2011/09/better-bit-mixing-improving-on.html
        let mut hash = packedpos;
        hash ^= hash >> 31;
        hash = hash.wrapping_mul(0x7fb5d329728ea185);
        hash ^= hash >> 27;
        hash = hash.wrapping_mul(0x81dadef4bc2dd44d);
        hash ^= hash >> 33;
        hash %= self.table_size as u64;

        // Now, see if the hash table already has an entry for the voxel. We'll
        // use quadratic probing when the hash table entry is already used for
        // another value; step stores the square root of the probe step.
        let mut step = 1;
        let mut nprobes = 0;

        loop {
            nprobes += 1;
            let entry = &self.hash_table[hash as usize];
            let entry_packedpos = entry.packed_pos.load(Ordering::Acquire);

            if entry_packedpos == packedpos {
                // Yes! Most of the time, there should already by a light
                // sampling distribution available.
                let mut dist = entry.distribution.dup(Ordering::Acquire);
                if dist.is_none() {
                    // Rarely, another thread will have already done a lookup
                    // at this point, found that there isn't a sampling
                    // distribution, and will already be computing the
                    // distribution for the point.  In this case, we spin until
                    // the sampling distribution is ready.  We assume that this
                    // is a rare case, so don't do anything more sophisticated
                    // than spinning.
                    // TODO: ProfilePhase
                    loop {
                        dist = entry.distribution.dup(Ordering::Acquire);
                        if dist.is_some() { break; }
                    }
                }
                // We have a valid sampling distribution
                nprobes_per_lookup::report_value(nprobes);
                return dist.as_ref().unwrap().clone();
            } else if entry_packedpos != INVALID_PACKED_POS {
                // The hash table entry we're checking has already been
                // allocated for another voxel. Advance to the next entry with
                // quadratic probing.
                hash += step * step;
                if hash >= self.table_size as u64 {
                    hash %= self.table_size as u64
                }

                step += 1u64;
            } else {
                // We have found an invalid entry. (Though this may have
                // changed since the load into entryPackedPos above.)  Use an
                // atomic compare/exchange to try to claim this entry for the
                // current position.
                let invalid = INVALID_PACKED_POS;
                let succ = entry
                    .packed_pos
                    .compare_exchange_weak(invalid, packedpos, Ordering::SeqCst, Ordering::Relaxed)
                    .is_ok();

                if succ {
                    // Success; we've claimed this position for this voxel's
                    // distribution. Now compute the sampling distribution and
                    // add it to the hash table. As long as packedPos has been
                    // set but the entry's distribution pointer is nullptr, any
                    // other threads looking up the distribution for this voxel
                    // will spin wait until the distribution pointer is
                    // written.
                    let dist = self.compute_dsitribution(pi);
                    entry.distribution.set_if_none(dist.clone(), Ordering::Release);
                    nprobes_per_lookup::report_value(nprobes);

                    return dist;
                }
            }

        }


    }
}

