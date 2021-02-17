use crate::core::interaction::{SurfaceInteraction, InteractionData, Interaction};
use crate::core::geometry::vector::Vector3f;
use crate::core::spectrum::Spectrum;
use crate::core::scene::Scene;
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::{Float, PI, INFINITY, clamp};
use crate::core::geometry::normal::Normal3f;
use crate::core::material::{Materials, TransportMode};
use crate::core::primitive::Primitive;
use std::sync::Arc;
use rayon::prelude::*;
use enum_dispatch::enum_dispatch;
use crate::core::reflection::{fr_dielectric, cos_theta, BSDF, BxDFType, BxDF, BxDFs};
use crate::core::interpolation::{catmull_rom_weights, integrate_catmull_rom, invert_catmull_rom, sample_catmull_rom_2d};
use crate::core::mipmap::Clampable;
use crate::core::medium::phase_hg;
use crate::{matches_flags_bxdf, get_type};
use std::fmt::{Formatter, Display, Result};
use bumpalo_herd::Member;
use crate::materials::disney::DisneyBSSRDF;

pub fn fresnel_moment1(eta: Float) -> Float {
    let eta2 = eta * eta;
    let eta3 = eta2 * eta;
    let eta4 = eta3 * eta;
    let eta5 = eta4 * eta;

    if eta < 1.0 {
        0.45966 - 1.73965 * eta + 3.37668 * eta2 - 3.904945 * eta3 +
        2.49277 * eta4 - 0.68441 * eta5
    } else {
        -4.61686 + 11.1136 * eta - 10.4646 * eta2 + 5.11455 * eta3 -
        1.27198 * eta4 + 0.12746 * eta5
    }
}

pub fn fresnel_moment2(eta: Float) -> Float {
    let eta2 = eta * eta;
    let eta3 = eta2 * eta;
    let eta4 = eta3 * eta;
    let eta5 = eta4 * eta;

    if eta < 1.0 {
        0.27614 - 0.87350 * eta + 1.12077 * eta2 - 0.65095 * eta3 +
        0.07883 * eta4 + 0.04860 * eta5
    } else {
        let r_eta = 1.0 / eta;
        let r_eta2 = r_eta * r_eta;
        let r_eta3 = r_eta2 * r_eta;

        -547.033 + 45.3087 * r_eta3 - 218.725 * r_eta2 +
        458.843 * r_eta + 404.557 * eta - 189.519 * eta2 +
        54.9327 * eta3 - 9.00603 * eta4 + 0.63942 * eta5
    }
}

fn beam_diffusion_ss(sigma_s: Float, sigma_a: Float, g: Float, eta: Float, r: Float) -> Float {
    // Compute material parameters and minimum t below the criticala angle
    let sigma_t = sigma_a + sigma_s;
    let rho = sigma_s / sigma_t;
    let tcrit = r * (eta * eta - 1.0).sqrt();
    let mut ess = 0.0;
    let nsamples = 100;

    for i in 0..nsamples {
        // Evaluate single scattering integrand and add to Ess
        let ti = tcrit - (1.0 - (i as Float + 0.5) / nsamples as Float).ln() / sigma_t;

        // Determine length d of connecting segment and cos theta o
        let d = (r * r + ti * ti).sqrt();
        let cos_thetao = ti / d;

        // Add contribution of single scattering at depth t
        ess += rho * (-sigma_t * (d + tcrit)).exp() / (d * d) *
               phase_hg(cos_thetao, g) * (1.0 - fr_dielectric(-cos_thetao, 1.0, eta)) *
               cos_thetao.abs();
    }

    ess / nsamples as Float
}

pub fn compute_beam_diffusion_bssrdf(g: Float, eta: Float, t: &mut BSSRDFTable) {
    // Choose radius of the diffusion profile discretization
    t.radius_samples[0] = 0.0;
    t.radius_samples[1] = 2.5e-3;

    for i in 2..t.nradius_samples {
        t.radius_samples[i] = t.radius_samples[i - 1] * 1.2;
    }

    // choose albedo values of diffusion profile discretization
    for i in 0..t.nrho_samples {
        t.rho_samples[i] =
            (1.0 - (-8.0 * i as Float / (t.nrho_samples - 1) as Float).exp()) /
            (1.0 - (-8 as Float).exp())
    }

    // Compute the diffusion profile for the ith albedo sample

    // Compute scattering profile for chosen albedo rho
    let d = (0..(t.nrho_samples * t.nradius_samples))
        .into_par_iter()
        .map(|idx| {
        let i = idx / t.nradius_samples;
        let j = idx % t.nradius_samples;

        (t.rho_samples[i], t.radius_samples[j])
    }).collect::<Vec<(Float, Float)>>();

    t.profile
        .par_iter_mut()
        .zip(d)
        .for_each(|(p, (rho, r))| {
            *p = 2.0 * PI * r *
                (beam_diffusion_ss(rho, 1.0 - rho, g, eta, r) +
                 beam_diffusion_ss(rho, 1.0 - rho, g, eta, r));
        });

    // Compute effective albedo rho and CDF for importance sampling
    let samples= &t.radius_samples;
    let d = t.profile
        .par_chunks(t.nradius_samples)
        .zip(t.profile_cdf.par_chunks_mut(t.nradius_samples));
    let nsamples = t.nradius_samples;

    t.rhoeff
        .par_iter_mut()
        .zip(d)
        .for_each(|(rho, (p, cdf))| {
            *rho = integrate_catmull_rom(nsamples, samples, p, cdf);
        })

}

pub fn subsurface_from_diffuse(t: &BSSRDFTable, rho_eff: &Spectrum, mfp: &Spectrum) -> (Spectrum, Spectrum) {
    let mut sigma_a = Spectrum::default();
    let mut sigma_s = Spectrum::default();

    for c in 0..Spectrum::n() {
        let rho = invert_catmull_rom(t.nrho_samples, &t.rho_samples, &t.rhoeff, rho_eff[c]);
        sigma_s[c] = rho / mfp[c];
        sigma_a[c] = (1.0 - rho) / mfp[c];
    }

    (sigma_a, sigma_s)
}

#[enum_dispatch(BSSRDFs)]
pub trait BSSRDF{
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum;
    fn sample_s<'a: 'a>(
        &self, scene: &Scene, u1: Float, u2: &Point2f,
        arena: &Member<'a>, si: &mut SurfaceInteraction<'a>, pdf: &mut Float) -> Spectrum;
}

#[enum_dispatch]
pub trait SeparableBSSRDF: BSSRDF {
    fn sw(&self, w: &Vector3f) -> Spectrum;
    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum;
    fn sample_sp<'a: 'a>(
        &self, scene: &Scene, u1: Float, u2: &Point2f,
        si: &mut SurfaceInteraction<'a>, pdf: &mut Float) -> Spectrum;
    fn pdf_sp(&self, si: &SurfaceInteraction) -> Float;

    fn sr(&self, d: Float) -> Spectrum;
    fn sample_sr(&self, ch: usize, u: Float) -> Float;
    fn pdf_sr(&self, ch: usize, r: Float) -> Float;

    fn mode(&self) -> TransportMode;
    fn eta(&self) -> Float;
}

#[enum_dispatch]
pub enum BSSRDFs {
    DisneyBSSRDF(DisneyBSSRDF),
    TabulatedBSSRDF(TabulatedBSSRDF)
}

#[enum_dispatch(SeparableBSSRDF, BSSRDF)]
pub enum SeparableBSSRDFs {
    TabulatedBSSRDF,
    DisneyBSSRDF
}

pub struct BSSRDFTable {
    nrho_samples    : usize,
    nradius_samples : usize,
    rho_samples     : Vec<Float>,
    radius_samples  : Vec<Float>,
    profile         : Vec<Float>,
    rhoeff          : Vec<Float>,
    profile_cdf     : Vec<Float>
}

impl BSSRDFTable {
    pub fn new(nrho_samples: usize, nradius_samples: usize) -> Self {
        Self {
            nrho_samples, nradius_samples,
            rho_samples     : vec![0.0; nrho_samples],
            radius_samples  : vec![0.0; nradius_samples],
            profile         : vec![0.0; nradius_samples * nrho_samples],
            rhoeff          : vec![0.0; nrho_samples],
            profile_cdf     : vec![0.0; nradius_samples * nrho_samples]
        }
    }

    #[inline(always)]
    pub fn eval_profile(&self, rho_index: usize, radius_index: usize) -> Float {
        self.profile[rho_index * self.nradius_samples + radius_index]
    }
}

#[derive(Clone)]
pub struct TabulatedBSSRDF {
    table       : Arc<BSSRDFTable>,
    sigma_t     : Spectrum,
    rho         : Spectrum,
    // SeperableBSSRDF data
    ns          : Normal3f,
    ss          : Vector3f,
    ts          : Vector3f,
    material    : Arc<Materials>,
    mode        : TransportMode,
    // BSSRDF data
    eta         : Float,
    po          : InteractionData
}

impl TabulatedBSSRDF {
    pub fn new(
        s: &SurfaceInteraction, mat: Option<Arc<Materials>>,
        mode: TransportMode, eta: Float, sigma_a: &Spectrum,
        sigma_s: &Spectrum, table: Arc<BSSRDFTable>) -> Self {
        let sigma_t = *sigma_a + *sigma_s;
        let mut rho = Spectrum::default();
        let ns = s.shading.n;
        let ss = s.shading.dpdu.normalize();
        let ts = ns.cross_vec(&ss);

        for i in 0..Spectrum::n() {
            rho[i] = if sigma_t[i] != 0.0 {
                sigma_s[i] / sigma_t[i]
            } else {
                0.0
            }
        }

        let po = InteractionData {
            p: s.p,
            time: s.time,
            p_error: s.p_error,
            wo: s.wo,
            n: s.n,
            medium_interface: None
        };

        let material = mat.unwrap();

        Self {
            ns, ss, ts, mode, material,
            table, sigma_t, rho, eta, po
        }
    }
}

impl SeparableBSSRDF for TabulatedBSSRDF {
    fn sw(&self, w: &Vector3f) -> Spectrum {
        let c = 1.0 - 2.0 * fresnel_moment1(1.0 / self.eta);

        Spectrum::new((1.0 - fr_dielectric(cos_theta(w), 1.0, self.eta)) / (c * PI))
    }

    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum {
        self.sr(self.po.p.distance(&pi.p))
    }

    fn sample_sp<'b: 'b>(
        &self, scene: &Scene, u1: f32, u2: &Point2f,
        si: &mut SurfaceInteraction<'b>, pdf: &mut f32) -> Spectrum {
        // TODO: ProfilePhase
        // Choose projection axis for BSSRDF sampling
        let (vx, vy, vz, mut u1n) = if u1 < 0.5 {
            (self.ss, self.ts, Vector3f::from(self.ns), u1 * 2.0)
        } else if u1 < 0.75 {
            (self.ts, Vector3f::from(self.ns), self.ss, (u1 - 0.5) * 4.0)
        } else {
            (Vector3f::from(self.ns), self.ss, self.ts, (u1 - 0.75) * 4.0)
        };

        // Choose spectral channel for BSSRDF sampling
        let ch = clamp((u1n * Spectrum::n() as Float) as usize, 0, Spectrum::n() - 1);
        u1n = u1n * Spectrum::n() as Float - ch as Float;

        // Sample BSSRDF profile in polar coordinates
        let r = self.sample_sr(ch, u2[0]);
        if r < 0.0 { return Spectrum::new(0.0); }
        let phi = 2.0 * PI * u2[1];

        // Compute BSSRDF profile bounds and intersection height
        let rmax = self.sample_sr(ch, 0.999);
        if r >= rmax { return Spectrum::new(0.0); }
        let l = 2.0 * (rmax * rmax - r * r).sqrt();

        // Compute BSSRDF sampling ray segment
        let p = self.po.p + (vx * phi.cos() + vy * phi.sin()) * r - vz * l * 0.5;
        let mut base = InteractionData {
            p,
            time: self.po.time,
            ..Default::default()
        };
        let ptarget = base.p + vz * l;

        // Intersect BSSRDF sampling ray against the scene geometry

        // Accumulate chain of intersections along ray
        let mut nfound = 0;

        let mut chain: Vec<SurfaceInteraction> = vec![];

        loop {
            let mut r = base.spawn_rayto_point(&ptarget);
            let mut si = SurfaceInteraction::default();
            if r.d == Default::default() || !scene.intersect(&mut r, &mut si) {
                break;
            }

            base = si.get_data();

            // Append admissible intersection to IntersectionChain
            let mat = si.primitive.as_ref().unwrap().get_material();

            if let Some(ref m) = mat {
                if Arc::ptr_eq(m, &self.material) {
                    chain.push(si);

                    nfound += 1;
                }
            }
        }

        // Randomly choose one of several intersections during BSSRDF sampling
        if nfound == 0 { return Spectrum::new(0.0); }
        let selected = clamp((u1n * nfound as Float) as usize, 0, nfound - 1);

        *si = chain.remove(selected);

        // Compute sample PDF and return the spatial BSSRDF term Sp
        *pdf = self.pdf_sp(si) / nfound as Float;

        self.sp(si)
    }

    fn pdf_sp(&self, pi: &SurfaceInteraction) -> f32 {
        // Express pti-pto and ni with respect to local coordinates at pto
        let d = self.po.p - pi.p;
        let dlocal = Vector3f::new(
            self.ss.dot(&d),
            self.ts.dot(&d),
            self.ns.dot_vec(&d));
        let nlocal = Normal3f::new(
            self.ss.dot_norm(&pi.n),
            self.ts.dot_norm(&pi.n),
            self.ns.dot(&pi.n));

        // Compute BSSRDF profile radius under projection along each axis
        let rproj = [
            (dlocal.y * dlocal.y + dlocal.z * dlocal.z).sqrt(),
            (dlocal.z * dlocal.z + dlocal.x * dlocal.x).sqrt(),
            (dlocal.x * dlocal.x + dlocal.y * dlocal.y).sqrt()
        ];

        // Return combined probability from all BSSRDF sampling strategies
        let mut pdf = 0.0;
        let axisprob = [0.25, 0.25, 0.5];
        let chprob = 1.0 / Spectrum::n() as Float;

        for axis in 0..3 {
            for ch in 0..Spectrum::n() {
                pdf += self.pdf_sr(ch, rproj[axis]) *
                       nlocal[axis].abs() *
                       chprob * axisprob[axis];
            }
        }

        pdf
    }

    fn sr(&self, r: Float) -> Spectrum {
        let mut Sr = Spectrum::new(0.0);

        for ch in 0..Spectrum::n() {
            // Convert r into unitless optical radius rOptical
            let roptical = r * self.sigma_t[ch];

            // Compute spline weights to interpolate BSSRDF on channel ch
            let (mut rho_off, mut rad_off) = (0, 0);
            let mut rho_weights = [0.0; 4];
            let mut radius_weights = [0.0; 4];

            let x = !catmull_rom_weights(
                self.table.nrho_samples as i32, &self.table.rho_samples,
                     self.rho[ch], &mut rho_off, &mut rho_weights);
            let y = !catmull_rom_weights(
                self.table.nradius_samples as i32, &self.table.radius_samples,
                roptical, &mut rad_off, &mut radius_weights);

            if !x || !y { continue; }

            // Set BSSRDF r[ch] using tensor spline interpolation
            let mut sr = 0.0;

            for i in 0..4 {
                for j in 0..4 {
                    let weight = rho_weights[i] * radius_weights[j];

                    if weight != 0.0 {
                        sr += weight * self.table.eval_profile(
                            rho_off as usize + i, rad_off as usize + j)
                    }
                }
            }

            // Cancel marginal PDF factor from tabulated BSSRDF profile
            if roptical != 0.0 { sr /= 2.0 * PI * roptical; }
            Sr[ch] = sr;
        }

        // Transform BSSRDF value into world space units
        Sr *= self.sigma_t * self. sigma_t;

        Sr.clamp(0.0, INFINITY)
    }

    fn sample_sr(&self, ch: usize, u: Float) -> Float{
        if self.sigma_t[ch] == 0.0 { return -1.0; }

        sample_catmull_rom_2d(
            self.table.nrho_samples as i32, self.table.nradius_samples as i32,
            &self.table.rho_samples, &self.table.radius_samples,
            &self.table.profile, &self.table.profile_cdf, self.rho[0], u, None, None
        ) / self.sigma_t[ch]
    }

    fn pdf_sr(&self, ch: usize, r: Float) -> Float {
        // convert r into unitless optical radius roptical
        let roptical = r * self.sigma_t[ch];
        let table = &self.table;

        // Compute spline weights to interpolate BSSRDF density on channel ch
        let mut rho_offset = 0;
        let mut radius_offset = 0;
        let mut rho_weights = [0.0; 4];
        let mut radius_weights = [0.0; 4];

        if !catmull_rom_weights(table.nrho_samples as i32, &table.rho_samples,
                self.rho[ch], &mut rho_offset, &mut rho_weights) ||
           !catmull_rom_weights(table.nradius_samples as i32, &table.radius_samples,
                roptical, & mut radius_offset, &mut radius_weights) {
            return 0.0;
        }

        // Return BSSRDF profile density for channel ch
        let mut sr = 0.0;
        let mut rho_eff = 0.0;

        for i in 0..4 {
            if rho_weights[i] == 0.0 { continue; }
            rho_eff += table.rhoeff[rho_offset as usize + i] * rho_weights[i];

            for j in 0..4 {
                if radius_weights[i] == 0.0 { continue; }
                sr += table.eval_profile(
                    rho_offset as usize + i,
                    radius_offset as usize + j) *
                    rho_weights[i] * radius_weights[j];
            }
        }

        // Cancel marginal PDF factor from tabulated BSSRDF profile
        if roptical != 0.0 { sr /= 2.0 * PI * roptical; }

        (sr * self.sigma_t[ch] * self.sigma_t[0] / rho_eff).max(0.0)
    }

    fn mode(&self) -> TransportMode {
        self.mode
    }

    fn eta(&self) -> Float {
        self.eta
    }
}

impl BSSRDF for TabulatedBSSRDF {
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum {
        let ft = fr_dielectric(cos_theta(&self.po.wo), 1.0, self.eta);

        self.sp(pi) * self.sw(wi) * (1.0 - ft)
    }


    fn sample_s<'b: 'b>(
        &self, scene: &Scene, u1: f32, u2: &Point2f, arena: &Member<'b>,
        si: &mut SurfaceInteraction<'b>, pdf: &mut f32) -> Spectrum {
        // TODO ProfilePhase
        let Sp = self.sample_sp(scene, u1, u2, si, pdf);

        if !Sp.is_black() {
            // Initialize material model at sampled surface interaction
            let bsdf = arena.alloc(BSDF::new(si, 1.0));
            let bxdf: &mut BxDFs = arena.alloc(
                SeparableBSSRDFAdapter::new(self.clone().into()).into());
            bsdf.add(bxdf);
            si.bsdf = Some(bsdf);
            si.wo = Vector3f::from(si.shading.n);
        }

        Sp
    }
}

pub struct SeparableBSSRDFAdapter {
    bssrdf  : SeparableBSSRDFs,
    bx_type : u8
}

impl SeparableBSSRDFAdapter {
    pub fn new(bssrdf: SeparableBSSRDFs) -> Self {
        let bx_type = (BxDFType::Reflection as u8) | (BxDFType::Diffuse as u8);

        Self { bssrdf, bx_type }
    }
}

impl BxDF for SeparableBSSRDFAdapter {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let mut f = self.bssrdf.sw(wi);
        //Update BSSRDF transmission term to account for adjoint light transport
        if self.bssrdf.mode() == TransportMode::Radiance{
            f *= self.bssrdf.eta() * self.bssrdf.eta();
        }

        f
    }
}

impl Display for SeparableBSSRDFAdapter {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ SeparableBSSRDFAdapter ]")
    }
}