use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::Vector3f;
use crate::core::spectrum::Spectrum;
use crate::core::scene::Scene;
use crate::core::geometry::point::Point2f;
use crate::core::pbrt::{Float, PI, INFINITY};
use bumpalo::Bump;
use crate::core::geometry::normal::Normal3f;
use crate::core::material::{Materials, TransportMode};
use std::sync::Arc;
use crate::core::reflection::{fr_dielectric, cos_theta};
use crate::core::cie::N_SPECTRAL_SAMPLES;
use crate::core::interpolation::catmull_rom_weights;
use crate::core::mipmap::Clampable;

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

pub trait BSSRDF{
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum;
    fn sample_s<'a>(
        &self, scene: &Scene, u1: Float, u2: &Point2f,
        arena: &'a Bump, si: &mut SurfaceInteraction<'a>, pdf: &mut Float) -> Spectrum;
}

pub trait SeparableBSSRDF: BSSRDF {
    fn sw(&self, w: &Vector3f) -> Spectrum;
    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum;
    fn sample_sp<'a>(
        &self, scene: &Scene, u1: Float, u2: &Point2f,
        arena: &'a Bump, si: &mut SurfaceInteraction<'a>, pdf: &mut Float) -> Spectrum;

    fn sr(&self, d: Float) -> Spectrum;
    fn sample_sr(&self, ch: usize, u: Float) -> Float;
    fn pdf_sr(&self, ch: usize, r: Float) -> Float;
}

pub enum BSSRDFs {

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

pub struct TabulatedBSSRDF<'a> {
    table       : &'a BSSRDFTable,
    sigma_t     : Spectrum,
    rho         : Spectrum,
    // SeperableBSSRDF data
    ns          : Normal3f,
    ss          : Vector3f,
    ts          : Vector3f,
    material    : Option<Arc<Materials>>,
    mode        : TransportMode,
    // BSSRDF data
    eta         : Float,
    po          : &'a SurfaceInteraction<'a>
}

impl<'a> TabulatedBSSRDF<'a> {
    pub fn new(
        po: &'a SurfaceInteraction<'a>, material: Option<Arc<Materials>>,
        mode: TransportMode, eta: Float, sigma_a: &Spectrum,
        sigma_s: &Spectrum, table: &'a BSSRDFTable) -> Self {
        let sigma_t = *sigma_a + *sigma_s;
        let mut rho = Spectrum::default();
        let ns = po.shading.n;
        let ss = po.shading.dpdu.normalize();
        let ts = ns.cross_vec(&ss);

        for i in 0..Spectrum::n() {
            rho[i] = if sigma_t[i] != 0.0 {
                sigma_s[i] / sigma_t[i]
            } else {
                0.0
            }
        }

        Self {
            ns, ss, ts, mode, material,
            table, sigma_t, rho, eta, po
        }
    }
}

impl<'a> SeparableBSSRDF for TabulatedBSSRDF<'a> {
    fn sw(&self, w: &Vector3f) -> Spectrum {
        let c = 1.0 - 2.0 * fresnel_moment1(1.0 / self.eta);

        Spectrum::new((1.0 - fr_dielectric(cos_theta(w), 1.0, self.eta)) / (c * PI))
    }

    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum {
        self.sr(self.po.p.distance(&pi.p))
    }

    fn sample_sp<'b>(
        &self, scene: &Scene, u1: f32, u2: &Point2f,
        arena: &'b Bump, si: &mut SurfaceInteraction<'b>, pdf: &mut f32) -> Spectrum {
        unimplemented!()
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
                        sr += weight * self.table.eval_profile(rho_off as usize + i, rad_off as usize + j)
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
        unimplemented!()
    }

    fn pdf_sr(&self, ch: usize, r: Float) -> Float {
        unimplemented!()
    }
}

impl<'a> BSSRDF for TabulatedBSSRDF<'a> {
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum {
        let ft = fr_dielectric(cos_theta(&self.po.wo), 1.0, self.eta);

        self.sp(pi) * self.sw(wi) * (1.0 - ft)
    }


    fn sample_s<'b>(
        &self, scene: &Scene, u1: f32, u2: &Point2f, arena: &'b Bump,
        si: &mut SurfaceInteraction<'b>, pdf: &mut f32) -> Spectrum {
        unimplemented!()
    }
}
