use crate::core::pbrt::{Float, SQRT2, erf_inv, Options};
use crate::core::rng::RNG;
use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::sampler::{Sampler, Samplers};
use crate::core::camera::{CameraSample, Cameras, Camera};
use crate::*;
use std::sync::Arc;
use crate::core::scene::Scene;
use bumpalo_herd::{Member, Herd};
use crate::core::sampling::Distribution1D;
use crate::integrators::bdpt::{LightIndexMap, generate_camera_subpath, generate_light_subpath, connect_bdpt, LightKey};
use crate::core::spectrum::Spectrum;
use crate::core::geometry::bounds::Bounds2f;
use crossbeam::crossbeam_channel::bounded;
use crate::core::integrator::{Integrator, compute_light_power_distribution, Integrators};
use std::collections::HashMap;
use rayon::prelude::*;
use crate::core::film::Film;
use crate::stat_percent;
use crate::core::paramset::ParamSet;

const CAMERA_STREAM_INDEX       : usize = 0;
const LIGHT_STREAM_INDEX        : usize = 1;
const CONNECTION_STREAM_INDEX   : usize = 2;
const NSAMPLE_STREAMS           : usize = 3;

stat_percent!("Integrator/Acceptance rate", accepted_total);

pub fn init_stats() {
    accepted_total::init();
}

#[derive(Default, Clone)]
struct PrimarySample {
    value                       : Float,
    last_modification_iteration : u64,
    value_backup                : Float,
    modify_backup               : u64
}

impl PrimarySample {
    pub fn backup(&mut self) {
        self.value_backup = self.value;
        self.modify_backup = self.last_modification_iteration;
    }

    pub fn restore(&mut self) {
        self.value = self.value_backup;
        self.last_modification_iteration = self.modify_backup;
    }
}

#[derive(Default, Clone)]
pub struct MLTSampler {
    rng                         : RNG,
    sigma                       : Float,
    large_step_probability: Float,
    stream_count                : usize,
    current_iteration           : u64,
    large_step                  : bool,
    last_large_step_iteration   : u64,
    stream_index                : usize,
    sample_index                : usize,
    X                           : Vec<PrimarySample>,
    // Sampler data
    current_pixel               : Point2i,
    current_pixel_sample_index  : u64,
    samples_1d_array_sizes      : Vec<usize>,
    samples_2d_array_sizes      : Vec<usize>,
    sample_array_1d             : Vec<Vec<Float>>,
    sample_array_2d             : Vec<Vec<Point2f>>,
    array_1d_offset             : usize,
    array_2d_offset             : usize,
    samples_per_pixel           : u64,

}

impl MLTSampler {
    pub fn new(
        mutations_per_pixel: u64, rng_seq_index: u64,
        sigma: Float, large_step_probability: Float,
        stream_count: usize) -> Self {
        Self {
            sigma,
            large_step_probability,
            stream_count,
            rng: RNG::new(rng_seq_index),
            samples_per_pixel: mutations_per_pixel,
            ..Default::default()
        }
    }

    fn ensure_ready(&mut self, index: usize) {
        // Enlarge MLTSampler::X if necessary and get currect Vec{x}
        if index >= self.X.len() {
            self.X.resize_with(index + 1, Default::default);
        }
        let mut Xi = &mut self.X[index];

        // Reset Vec{x} if a large step took place in the meantime
        if Xi.last_modification_iteration < self.last_large_step_iteration {
            Xi.value = self.rng.uniform_float();
            Xi.last_modification_iteration = self.last_large_step_iteration;
        }

        // Apply remaining sequence of mutations to sample
        Xi.backup();
        if self.large_step {
            Xi.value = self.rng.uniform_float();
        } else {
            let nsmall = self.current_iteration - Xi.last_modification_iteration;
            // Apply nsmall step mutations

            // Sample the standard normal distribution N(0, 1)
            let normal_sample = SQRT2 * erf_inv(2.0 * self.rng.uniform_float() - 1.0);

            // Compute the effective standard deviation and apply pertubation to Vec{x}
            let effsigma = self.sigma * (nsmall as Float).sqrt();
            Xi.value += normal_sample * effsigma;
            Xi.value -= Xi.value.floor();
        }

        Xi.last_modification_iteration = self.current_iteration;
    }

    pub fn start_iteration(&mut self) {
        self.current_iteration += 1;
        self.large_step = self.rng.uniform_float() < self.large_step_probability
    }

    pub fn accept(&mut self) {
        if self.large_step {
            self.last_large_step_iteration = self.current_iteration;
        }
    }

    pub fn reject(&mut self) {
        for Xi in self.X.iter_mut() {
            if Xi.last_modification_iteration == self.current_iteration {
                Xi.restore();
            }
        }

        self.current_iteration -= 1;
    }

    pub fn get_next_index(&mut self) -> usize {
        let ret = self.stream_index * self.stream_count * self.sample_index;
        self.sample_index += 1;

        ret
    }

    pub fn start_stream(&mut self, index: usize) {
        assert!(index < self.stream_count);
        self.stream_index = index;
        self.sample_index = 0;
    }
}

impl Sampler for MLTSampler {
    get_sampler_data!();

    fn start_pixel(&mut self, p: &Point2i) {
        start_pixel_default!(self, *p)
    }

    fn get_1d(&mut self) -> f32 {
        // TODO: ProfilePhase
        let index = self.get_next_index();
        self.ensure_ready(index);

        self.X[index].value
    }

    fn get_2d(&mut self) -> Point2f {
        Point2f::new(self.get_1d(), self.get_1d())
    }

    get_camera_sample_default!();

    request_1d_array_default!();

    request_2d_array_default!();

    get_1d_array_default!();

    get_2d_array_default!();

    fn start_next_sample(&mut self) -> bool {
        start_next_sample_default!(self)
    }

    fn set_sample_number(&mut self, sample_num: u64) -> bool {
        set_sample_number_default!(self, sample_num)
    }

    current_sample_number_default!();

    fn clone(&self, _seed: isize) -> Samplers {
        panic!("MLTSampler::Clone() is not implemented")
    }
}

pub struct MLTIntegrator {
    camera: Arc<Cameras>,
    max_depth               : usize,
    nbootstrap              : usize,
    nchains                 : usize,
    mutations_per_pixel     : usize,
    sigma                   : Float,
    large_step_probability  : Float
}

impl MLTIntegrator {
    pub fn new(
        camera: Arc<Cameras>, max_depth: usize, nbootstrap: usize,
        nchains: usize, mutations_per_pixel: usize, sigma: Float,
        large_step_probability: Float) -> Self {
        Self {
            camera, max_depth, nbootstrap,
            nchains, mutations_per_pixel,
            sigma, large_step_probability
        }
    }

    pub fn l(
        &self, scene: &Scene, arena: &Member,
        lightdis: &Arc<Distribution1D>, lti: &LightIndexMap,
        sampler: &mut MLTSampler, depth: usize,
        praster: &mut Point2f) -> Spectrum {
        sampler.start_stream(CAMERA_STREAM_INDEX);
        // Determine the number of available strategies and pick a specific one
        let (s, t, nstrategies) = if depth == 0 {
            (0, 2, 1)
        } else {
            let ns = depth + 2;
            let s = std::cmp::min(sampler.get_1d() as usize * ns, ns - 1);
            (s, ns - s, ns)
        };

        // Generate a camera subpath with exactly t vertices;
        let mut cvertices = Vec::with_capacity(t);
        let sbounds = Bounds2f::from(self.camera.film().get_sample_bounds());
        *praster = sbounds.lerp(&sampler.get_2d());

        if generate_camera_subpath(
            scene, sampler, arena, t, &self.camera,
            praster, &mut cvertices) != t {
            return Spectrum::new(0.0);
        }

        // Generate a light subpath with exactly s vertices
        sampler.start_stream(LIGHT_STREAM_INDEX);
        let mut lvertices = Vec::with_capacity(s);

        if generate_light_subpath(
            scene, sampler, arena, s, cvertices[0].time(),
            &lightdis, lti, &mut lvertices) != s {
            return Spectrum::new(0.0);
        }

        // Execute connection strategy and return the radiance estimate
        sampler.start_stream(CONNECTION_STREAM_INDEX);

        connect_bdpt(
            scene, &mut lvertices, &mut cvertices,
            s, t, &lightdis, lti, &self.camera,
            sampler, praster, None) * nstrategies as Float
    }
}

impl Integrator for MLTIntegrator {
    fn render(&mut self, scene: &Scene) {
        let lightdistr = compute_light_power_distribution(scene).unwrap();
        let mut herd = Herd::new();

        // Compute a reverse mapping from light pointers to offsets into the
        // scene lights vector (and, equivalently, offsets into
        // lightDistr). Added after book text was finalized; this is critical
        // to reasonable performance with 100s+ of light sources.
        let mut lti: LightIndexMap = HashMap::new();
        for (i, light) in scene.lights.iter().enumerate() {
            lti.insert(LightKey::new(light), i);
        }

        // Generate bootstrap samples and compute normalization constant b
        let nbootrap_samples = self.nbootstrap * (self.max_depth + 1);
        let mut bootstrap_weights = vec![0.0; nbootrap_samples];
        let (sendl, recvl) = bounded(self.nbootstrap);

        if !scene.lights.is_empty() {
            // TODO ProgressReporter
            (0..self.nbootstrap).into_par_iter().for_each_init(|| herd.get(), | arena, i | {
                let sendlx = sendl.clone();
                // Generate ith bootstrap sample
                for depth in 0..=self.max_depth {
                    let rng_idx = i * (self.max_depth + 1) + depth;
                    let mut sampler = MLTSampler::new(
                        self.mutations_per_pixel as u64,
                        rng_idx as u64, self.sigma,
                        self.large_step_probability,
                        NSAMPLE_STREAMS);
                    let mut praster = Point2f::default();
                    let l = self.l(
                        scene, arena, &lightdistr, &lti,
                        &mut sampler, depth, &mut praster).y();
                    sendlx.send((rng_idx, l)).unwrap()
                }
                    // TODO: Progress
            });

            for _ in 0..self.nbootstrap {
                let (i , l) = recvl.recv().unwrap();
                bootstrap_weights[i] = l;
            }
        }

        herd.reset();

        let bootstrap = Distribution1D::new(bootstrap_weights);
        let b = bootstrap.func_int * (self.max_depth + 1) as Float;

        // Run nchains Markov chains in parallel
        let film: Arc<Film> = self.camera.film();
        let ntotal_mutations = self.mutations_per_pixel as u64 * film.get_sample_bounds().area() as u64;

        herd.reset();

        if !scene.lights.is_empty() {
            // TODO Progress
            (0..self.nchains).into_par_iter().for_each_init(|| herd.get(), | arena, i | {
                let nchain_mutations = std::cmp::min(
                    (i + 1) as u64 * ntotal_mutations / self.nchains as u64,
                    ntotal_mutations);

                // Select initial state from the set of bootstrap samples
                let mut rng = RNG::new(i as u64);
                let bootstrap_index = bootstrap.sample_discrete(rng.uniform_float(), None, None);
                let depth = bootstrap_index % (self.max_depth + 1);

                // Initialize local variables for selected state
                let mut sampler = MLTSampler::new(
                    self.mutations_per_pixel as u64, bootstrap_index as u64,
                    self.sigma, self.large_step_probability, NSAMPLE_STREAMS);
                let mut pcurrent = Point2f::default();
                let mut lcurrent = self.l(scene, arena, &lightdistr, &lti, &mut sampler, depth,&mut pcurrent);

                // Run the Markov chain for nChainMutations steps
                for _j in 0..nchain_mutations {
                    sampler.start_iteration();
                    let mut pproposed = Point2f::default();
                    let lproposed = self.l(scene, arena, &lightdistr, &lti, &mut sampler, depth, &mut pproposed);
                    // Compute acceptance probability for proposed sample
                    let accept = (lproposed.y() / lcurrent.y()).min(1.0);

                    // Splat both current and proposed samples to film
                    if accept > 0.0 {
                        film.add_splat(&pproposed, lproposed* accept / lproposed.y());
                    }
                    film.add_splat(&pcurrent, lcurrent * (1.0 - accept) / lcurrent.y());

                    // Accept or reject the proposal
                    if rng.uniform_float() < accept {
                        pcurrent = pproposed;
                        lcurrent = lproposed;
                        sampler.accept();
                        accepted_total::inc_num()
                    } else {
                        sampler.reject();
                    }
                    accepted_total::inc_den()

                    // TODO: Progress
                }
            })
        }

        // Store final image computed with MLT
        film.write_image(b / self.mutations_per_pixel as Float).unwrap();
    }
}

pub fn create_mlt_integrator(params: &ParamSet, camera: Arc<Cameras>, opts: &Options) -> Option<Integrators> {
    let maxdepth = params.find_one_int("maxdepth", 5) as usize;
    let mut nbootstrap = params.find_one_int("bootstrapsamples", 100000) as usize;
    let nchains = params.find_one_int("chains", 1000) as usize;
    let mut mutations_per_pixel = params.find_one_int("mutationsperpixel", 100) as usize;
    let large_step_prob = params.find_one_float("largestepprobability", 0.3);
    let sigma = params.find_one_float("sigma", 0.01);

    if opts.quick_render {
        mutations_per_pixel = std::cmp::max(1, mutations_per_pixel / 16);
        nbootstrap = std::cmp::max(1, nbootstrap / 16);
    }

    Some(MLTIntegrator::new(
            camera, maxdepth, nbootstrap, nchains,
            mutations_per_pixel, sigma, large_step_prob).into())
}