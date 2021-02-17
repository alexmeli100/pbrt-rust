use std::sync::Arc;
use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use crate::core::pbrt::{Float, radians, clamp, PI, INFINITY};
use crate::core::spectrum::{Spectrum, SpectrumType};
use crate::core::reflection::{BxDFType, BxDF, fr_dielectric, abs_cos_theta, BSDF, BxDFs};
use static_assertions::_core::fmt::{Display, Formatter};
use crate::core::geometry::point::Point2f;
use crate::core::geometry::vector::Vector3f;
use bumpalo_herd::Member;
use crate::{matches_flags_bxdf, get_type};
use crate::core::material::{Material, TransportMode, Materials};
use log::warn;
use crate::core::interaction::SurfaceInteraction;
use crate::core::paramset::TextureParams;
use crate::textures::constant::ConstantTexture;

const PMAX: usize = 3;
const SQRT_PI_OVER8: Float = 0.626657069;

pub fn mp(
    cos_thetai: Float, cos_thetao: Float,
    sin_thetai: Float, sin_thetao: Float, v: Float) -> Float {
    let a = cos_thetai * cos_thetao / v;
    let b = sin_thetai * sin_thetao / v;

    let mp = if v <= 0.1 {
        (log_i0(a) - b - 1.0 / v + 0.6931 + (1.0 / (2.0 * v)).ln()).exp()
    } else {
        ((-b).exp() * i0(a)) / ((1.0 / v).sinh() * 2.0 * v)
    };
    assert!(!mp.is_infinite() && !mp.is_nan());

    mp
}

#[inline]
fn i0(x: Float) -> Float {
    let mut val = 0.0;
    let mut x2i = 1.0;
    let mut ifact = 1.0;
    let mut i4 = 1;

    for i in 0..10 {
        if i > 1 { ifact *= i as Float; }

        val += x2i / (i4 as Float * sqr(ifact));
        x2i *= x * x;
        i4 *= 4;
    }

    val
}

#[inline]
fn log_i0(x: Float) -> Float {
    if x > 12.0 {
        x + 0.5 * -((2.0 * PI).ln()) + (1.0 / x).ln() + 1.0 / (8.0 * x)
    } else {
        i0(x).ln()
    }
}

pub fn ap(cos_thetao: Float, eta: Float, h: Float, t: &Spectrum) -> [Spectrum; PMAX + 1] {
    let mut ap = [Spectrum::default(); PMAX + 1];

    // Compute p=0 attenuation at initial cylinder intersection
    let cos_gammao = safe_sqrt(1.0 - h * h);
    let cos_theta = cos_thetao * cos_gammao;
    let f = fr_dielectric(cos_theta, 1.0, eta);
    ap[0] = Spectrum::new(f);

    // Compute p=1 attenuation term;
    ap[1] = t * sqr(1.0 - f);

    // Compute attenuation terms up to p=PMAX
    for p in 2..PMAX {
        ap[p] = ap[p - 1] * t * f;
    }

    // Compute attenuation term accounting for remaining orders of scattering
    ap[PMAX] = ap[PMAX - 1] * t * f / (Spectrum::new(1.0) - t * f);

    ap
}

#[inline]
fn phi(p: isize, gammao: Float, gammat: Float) -> Float {
    2.0 * p as Float * gammat - 2.0 * gammao + p as Float * PI
}

#[inline]
fn logistic(mut x: Float, s: Float) -> Float {
    x = x.abs();

    (-x / s).exp() / (s * sqr(1.0 + (-x / s).exp()))
}

#[inline]
fn logistic_cdf(x: Float, s: Float) -> Float {
    1.0 / (1.0 + (-x / s).exp())
}

#[inline]
fn trimmed_logistic(x: Float, s: Float, a: Float, b: Float) -> Float {
    assert!(a < b);

    logistic(x , s) / (logistic_cdf(b, s) - logistic_cdf(a, s))
}

#[inline]
fn np(ph: Float, p: isize, s: Float, gammao: Float, gammat: Float) -> Float {
    let mut dphi = ph - phi(p, gammao, gammat);

    while dphi > PI { dphi -= 2.0 * PI; }
    while dphi < -PI { dphi += 2.0 * PI; }

    trimmed_logistic(dphi, s, -PI, PI)
}

fn sample_trimmed_logistic(u: Float, s: Float, a: Float, b: Float) -> Float {
    assert!(a < b);

    let k = logistic_cdf(b, s) - logistic_cdf(a, s);
    let x = -s * (1.0 / (u * k + logistic_cdf(a, s)) - 1.0).ln();
    assert!(!x.is_nan());

    clamp(x, a, b)
}

pub struct HairMaterial {
    sigma_a     : Option<Arc<TextureSpec>>,
    color       : Option<Arc<TextureSpec>>,
    eumelanin   : Option<Arc<TextureFloat>>,
    phemelanin  : Option<Arc<TextureFloat>>,
    eta         : Arc<TextureFloat>,
    beta_m      : Arc<TextureFloat>,
    beta_n      : Arc<TextureFloat>,
    alpha       : Arc<TextureFloat>
}

impl HairMaterial {
    pub fn new(
        sigma_a: Option<Arc<TextureSpec>>, color: Option<Arc<TextureSpec>>,
        eumelanin: Option<Arc<TextureFloat>>, phemelanin: Option<Arc<TextureFloat>>,
        eta: Arc<TextureFloat>, beta_m: Arc<TextureFloat>, beta_n: Arc<TextureFloat>,
        alpha: Arc<TextureFloat>) -> Self {
        Self {
            sigma_a, color, eumelanin, phemelanin,
            eta, beta_m, beta_n, alpha
        }
    }
}

impl Material for HairMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, _mode: TransportMode, _allow_multiple_lobes: bool) {
        let bm = self.beta_m.evaluate(si);
        let bn = self.beta_n.evaluate(si);
        let a = self.alpha.evaluate(si);
        let e = self.eta.evaluate(si);

        let bsdf = arena.alloc(BSDF::new(si, e));

        let siga = if let Some(ref s) = self.sigma_a {
            s.evaluate(si).clamps(0.0, INFINITY)
        } else if let Some(ref c) = self.color {
            let x = c.evaluate(si).clamps(0.0, INFINITY);
            HairBSDF::sigmaa_from_reflectance(&x, bn)
        } else {
            assert!(self.eumelanin.is_some() || self.phemelanin.is_some());
            let ce = if let Some(ref e) = self.eumelanin {
                e.evaluate(si)
            } else {
                0 as Float
            };
            let cp = if let Some(ref p) = self.phemelanin {
                p.evaluate(si)
            } else {
                0 as Float
            };

            HairBSDF::sigmaa_from_concentration(ce.max(0.0), cp.max(0.0))
        };

        // Offset along width
        let h = -1.0 + 2.0 * si.uv[1];
        let bxdf: &mut BxDFs = arena.alloc(HairBSDF::new(h, e, &siga, bm, bn, a).into());
        bsdf.add(bxdf);

        si.bsdf = Some(bsdf)

    }
}

pub struct HairBSDF {
    h           : Float,
    gammao      : Float,
    eta         : Float,
    sigma_a     : Spectrum,
    betam       : Float,
    betan       : Float,
    v           : [Float; PMAX + 1],
    s           : Float,
    sin2k_alpha : [Float; 3],
    cos2k_alpha : [Float; 3],
    bx_type     : u8
}

impl HairBSDF {
    pub fn new(
        h: Float, eta: Float, sig: &Spectrum, betam: Float,
        betan: Float, alpha: Float) -> Self {
        assert!(h >= -1.0 && h <= 1.0);
        assert!(betam >= 0.0 && betam <= 1.0);
        assert!(betan >= 0.0 && betam <= 0.0);

        // Compute longitudinal variance from beta
        static_assertions::const_assert!(PMAX >= 3);
        let mut v = [0 as Float; PMAX + 1];
        v[0] = sqr(0.726 * betam + 0.812 * sqr(betam) + 3.7 * pow(betam, 20));
        v[1] = 0.25 * v[0];
        v[2] = 4.0 * v[0];

        for p in 3..=PMAX { v[p] = v[2]; }

        // Compute azimuthal logistic scale factor from beta_n
        let s = SQRT_PI_OVER8 * (0.265 * betan + 1.194 * sqr(betan) + 5.372 * pow(betan, 22));
        assert!(!s.is_nan());

        // Compute alpha terms for hair scales
        let mut sin2k_alpha = [radians(alpha).sin(), 0.0, 0.0];
        let mut cos2k_alpha = [safe_sqrt(1.0 - sqr(sin2k_alpha[0])), 0.0, 0.0];

        for i in 0..3 {
            sin2k_alpha[i] = 2.0 * cos2k_alpha[i - 1] * sin2k_alpha[i - 1];
            cos2k_alpha[i] = sqr(cos2k_alpha[i - 1]) - sqr(sin2k_alpha[i - 1]);
        }

        let gammao = safe_asin(h);
        let sigma_a = *sig;
        let bx_type = BxDFType::Glossy as u8 | BxDFType::Reflection as u8 | BxDFType::Transmission as u8;

        Self {
            h, gammao, eta, sigma_a, betan, betam,
            v, s, cos2k_alpha, sin2k_alpha, bx_type
        }

    }

    fn compute_ap_pdf(&self, cos_thetao: Float) -> [Float; PMAX + 1] {
        // Compute array of A_P values for cos_thetao
        let sin_thetao = safe_sqrt(1.0 - cos_thetao * cos_thetao);

        // Compute cos thetat for refracted ray
        let sin_thetat = sin_thetao / self.eta;
        let cos_thetat = safe_sqrt(1.0 - sqr(sin_thetat));

        // Compute gammat for refracted ray
        let etap = (self.eta * self.eta - sqr(sin_thetao)) / cos_thetao;
        let sin_gammat = self.h / etap;
        let cos_gammat = safe_sqrt(1.0 - sqr(sin_gammat));

        // Compute the transmittance T of a single path through the cylinder
        let t = (-self.sigma_a * (2.0 * cos_gammat / cos_thetat)).exp();
        let ap = ap(cos_thetao, self.eta, self.h, &t);

        // Compute A_p PDF from individual A_p terms
        let mut ap_pdf = [0 as Float; PMAX + 1];
        let sumy = ap.iter().fold(0 as Float, |s, ap| s + ap.y());

        for (i, p) in ap_pdf.iter_mut().enumerate() {
            *p = ap[i].y() / sumy;
        }

        ap_pdf
    }

    fn sigmaa_from_concentration(ce: Float, cp: Float) -> Spectrum {
        let mut sigma_a = [0 as Float; 3];
        let eumelanin_sigmaa = [0.419, 0.697, 1.37];
        let pheomelanin_sigmaa = [0.187, 0.4, 1.05];

        for (i, s) in sigma_a.iter_mut().enumerate() {
            *s = ce * eumelanin_sigmaa[i] + cp * pheomelanin_sigmaa[i];
        }

        Spectrum::from_rgb(sigma_a, SpectrumType::Reflectance)
    }

    fn sigmaa_from_reflectance(c: &Spectrum, betan: Float) -> Spectrum {
        let mut sigma_a = Spectrum::default();

        for i in 0..Spectrum::n() {
            sigma_a[i] = sqr(c[i].ln() /
                (5.969 - 0.215 * betan + 2.532 * sqr(betan) -
                 10.73 * pow(betan, 3) + 5.574 * pow(betan, 4) +
                 0.245 * pow(betan, 5)))
        }

        sigma_a
    }
}

impl BxDF for HairBSDF {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        // Compute hair coordinate system terms related to wo
        let sin_thetao = wo.x;
        let cos_thetao = safe_sqrt(1.0 - sqr(sin_thetao));
        let phio = wo.z.atan2(wo.y);

        // Compute hair coordinate system terms related to wi
        let sin_thetai = wi.x;
        let cos_thetai = safe_sqrt(1.0 - sqr(sin_thetai));
        let phii = wi.z.atan2(wi.y);

        // Compute cos thetat for refracted ray
        let sin_thetat = sin_thetao / self.eta;
        let cos_thetat = safe_sqrt(1.0 - sqr(sin_thetat));

        // Compute gammat for refracted ray
        let etap = (self.eta * self.eta - sqr(sin_thetao)).sqrt() / cos_thetao;
        let sin_gammat = self.h / etap;
        let cos_gammat = safe_sqrt(1.0 - sqr(sin_gammat));
        let gammat = safe_asin(sin_gammat);

        // Compute the transmittance T of a single path through the cylinder
        let t = (-self.sigma_a * (2.0 * cos_gammat / cos_thetat)).exp();

        // Evaluate hair BSDF
        let phi = phii - phio;
        let ap = ap(cos_thetao, self.eta, self.h, &t);
        let mut fsum = Spectrum::new(0.0);

        for p in 0..PMAX {
            // Compute sin thetao and cos thetao accounting for scales
            let mut cos_theta_op: Float;
            let sin_theta_op: Float;

            if p == 0 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[1] - cos_thetao * self.sin2k_alpha[1];
                cos_theta_op = cos_thetao * self.cos2k_alpha[1] + sin_thetao * self.sin2k_alpha[1];
            }

            // Handle remainder of $p$ values for hair scale tilt
            else if p == 1 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[0] + cos_thetao * self.sin2k_alpha[0];
                cos_theta_op = cos_thetao * self.cos2k_alpha[0] - sin_thetao * self.sin2k_alpha[0];
            } else if p == 2 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[2] + cos_thetao * self.sin2k_alpha[2];
                cos_theta_op = cos_thetao * self.cos2k_alpha[2] - sin_thetao * self.sin2k_alpha[2];
            } else {
                sin_theta_op = sin_thetao;
                cos_theta_op = cos_thetao;
            }

            // Handle out-of-range cos thetao from scale adjusment
            cos_theta_op = cos_theta_op.abs();
            fsum += ap[p] * mp(cos_thetai, cos_theta_op, sin_thetai, sin_theta_op, self.v[p]) *
                    np(phi, p as isize, self.s, self.gammao, gammat);
        }

        // Compute contribution of remaining terms after PMAX
        fsum += ap[PMAX] * mp(cos_thetai, cos_thetao, sin_thetai, sin_thetao, self.v[PMAX]) / (2.0 * PI);
        let c = abs_cos_theta(wi);

        if c > 0.0 { fsum /= c; }
        let y = fsum.y();
        assert!(!y.is_infinite() && !y.is_nan());

        fsum
    }

    fn sample_f(
        &self, wo: &Vector3f, wi: &mut Vector3f, u2: &Point2f,
        pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        // Compute hair coordinate system terms related to wo
        let sin_thetao = wo.x;
        let cos_thetao = safe_sqrt(1.0 - sqr(sin_thetao));
        let phio = wo.z.atan2(wo.y);

        // Derive four random samples from u2
        let mut u = [demux_float(u2[0]), demux_float(u2[1])];

        // Determine which term p to sample for hair scattering
        let ap_pdf = self.compute_ap_pdf(cos_thetao);
        let mut p = 0;

        for  i in 0..PMAX {
            p = i;
            if u[0][0] < ap_pdf[i] { break }

            u[0][0] -= ap_pdf[i];
        }

        // Rotate sin thetao and cos thetao to account for hair scale tilt
        let sin_theta_op: Float;
        let cos_theta_op: Float;

        if p == 0 {
            sin_theta_op = sin_thetao * self.cos2k_alpha[1] - cos_thetao * self.sin2k_alpha[1];
            cos_theta_op = cos_thetao * self.cos2k_alpha[1] + sin_thetao * self.sin2k_alpha[1];
        } else if p == 1 {
            sin_theta_op = sin_thetao * self.cos2k_alpha[0] + cos_thetao * self.sin2k_alpha[0];
            cos_theta_op = cos_thetao * self.cos2k_alpha[0] - sin_thetao * self.sin2k_alpha[0];
        } else if p == 2 {
            sin_theta_op = sin_thetao * self.cos2k_alpha[2] + cos_thetao * self.sin2k_alpha[2];
            cos_theta_op = cos_thetao * self.cos2k_alpha[2] - sin_thetao * self.sin2k_alpha[2];
        } else {
            sin_theta_op = sin_thetao;
            cos_theta_op = cos_thetao;
        }

        // Sample M_p to compute thetai
        u[1][0] = (u[1][0]).max(1.0e-5);
        let cos_theta = 1.0 + self.v[p] * (u[1][0] + (1.0 - u[1][0]) * (-2.0 / self.v[p]).exp()).ln();
        let sin_theta = safe_sqrt(1.0 - sqr(cos_theta));
        let cos_phi = (2.0 * PI * u[1][1]).cos();
        let sin_thetai = -cos_theta * sin_theta_op + sin_theta * cos_phi * cos_theta_op;
        let cos_thetai = safe_sqrt(1.0 - sqr(sin_thetai));

        // Sample N_p to compute Delta phi
        let etap = (self.eta * self.eta - sqr(sin_thetao)).sqrt() / cos_thetao;
        let sin_gammat = self.h / etap;
        let gammat = safe_asin(sin_gammat);
        let dphi = if p < PMAX {
            phi(p as isize, self.gammao, gammat) + sample_trimmed_logistic(u[0][1], self.s, -PI, PI)
        } else {
            2.0 * PI * u[0][1]
        };

        // compute wi from sampled hair scattering angles
        let phii = phio + dphi;
        *wi = Vector3f::new(
            sin_thetai,
            cos_thetai * phii.cos(),
            cos_thetai * phii.sin()
        );

        // Compute PDF for sampled hair scattering direction wi
        *pdf = 0.0;

        for p in 0..PMAX {
            // Compute sin thetao and cos theta terms accounting for scales
            let sin_theta_op: Float;
            let mut cos_theta_op: Float;

            if p == 0 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[1] - cos_thetao * self.sin2k_alpha[1];
                cos_theta_op = cos_thetao * self.cos2k_alpha[1] + sin_thetao * self.sin2k_alpha[1];
            } else if p == 1 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[0] + cos_thetao * self.sin2k_alpha[0];
                cos_theta_op = cos_thetao * self.cos2k_alpha[0] - sin_thetao * self.sin2k_alpha[0];
            } else if p == 2 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[2] + cos_thetao * self.sin2k_alpha[2];
                cos_theta_op = cos_thetao * self.cos2k_alpha[2] - sin_thetao * self.sin2k_alpha[2];
            } else {
                sin_theta_op = sin_thetao;
                cos_theta_op = cos_thetao;
            }

            //handle out-of-range cos thetao from scale adjustment
            cos_theta_op = cos_theta_op.abs();
            *pdf += mp(cos_thetai, cos_theta_op, sin_thetai, sin_theta_op, self.v[PMAX]) *
                    ap_pdf[p] * np(dphi, p as isize, self.s, self.gammao, gammat);
        }

        *pdf += mp(cos_thetai, cos_thetao, sin_thetai, sin_thetao, self.v[PMAX]) *
                ap_pdf[PMAX] * (1.0 / (2.0 * PI));

        self.f(wo, &*wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        // Compute hair coordinate system terms related to wo
        let sin_thetao = wo.x;
        let cos_thetao = safe_sqrt(1.0 - sqr(sin_thetao));
        let phio = wo.z.atan2(wo.y);

        // Compute hair coordinate system terms related to wi
        let sin_thetai = wi.x;
        let cos_thetai = safe_sqrt(1.0 - sqr(sin_thetai));
        let phii = wi.z.atan2(wi.y);

        // Compute gammat for refracted ray
        let etap = (self.eta * self.eta - sqr(sin_thetao)).sqrt() / cos_thetao;
        let sin_gammat = self.h / etap;
        let gammat = safe_asin(sin_gammat);

        // Compute PDF for A_p terms
        let ap_pdf = self.compute_ap_pdf(cos_thetao);

        // Compute PDF sum for hair scattering events
        let phi = phii - phio;
        let mut pdf = 0 as Float;

        for p in 0..PMAX {
            // Compute sin thetao and cos theta terms accounting for scales
            let sin_theta_op: Float;
            let mut cos_theta_op: Float;

            if p == 0 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[1] - cos_thetao * self.sin2k_alpha[1];
                cos_theta_op = cos_thetao * self.cos2k_alpha[1] + sin_thetao * self.sin2k_alpha[1];
            } else if p == 1 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[0] + cos_thetao * self.sin2k_alpha[0];
                cos_theta_op = cos_thetao * self.cos2k_alpha[0] - sin_thetao * self.sin2k_alpha[0];
            } else if p == 2 {
                sin_theta_op = sin_thetao * self.cos2k_alpha[2] + cos_thetao * self.sin2k_alpha[2];
                cos_theta_op = cos_thetao * self.cos2k_alpha[2] - sin_thetao * self.sin2k_alpha[2];
            } else {
                sin_theta_op = sin_thetao;
                cos_theta_op = cos_thetao;
            }

            //handle out-of-range cos thetao from scale adjustment
            cos_theta_op = cos_theta_op.abs();
            pdf += mp(cos_thetai, cos_theta_op, sin_thetai, sin_theta_op, self.v[PMAX]) *
                ap_pdf[p] * np(phi, p as isize, self.s, self.gammao, gammat);
        }

        pdf += mp(cos_thetai, cos_thetao, sin_thetai, sin_thetao, self.v[PMAX]) *
            ap_pdf[PMAX] * (1.0 / (2.0 * PI));

        pdf
    }

}

impl Display for HairBSDF {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "[ Hair h: {} gammaO: {} eta: {} beta_m: {} \
                  beta_n: {} v[0]: {} s: {} sigma_a: {}]",
               self.h, self.gammao, self.eta, self.betam,
               self.betan, self.v[0], self.s, self.sigma_a)
    }
}

// Utility functions
#[inline(always)]
fn sqr(v: Float) -> Float { v * v }

fn pow(v: Float, n: isize) -> Float {
    assert!(n > 0, "Power can't be negative");

    if n == 0 {
        return 1.0;
    }

    if n == 1 {
        return v;
    }

    let n2 = pow(v, n /2);
    n2 * n2 * pow(v, n & 1)
}

fn safe_asin(x: Float) -> Float {
    assert!(x >= -1.0001 && x <= 1.0001);

    clamp(x, -1.0, 1.0).asin()
}

fn safe_sqrt(x: Float) -> Float {
    assert!(x >= -1.0e-4);

    x.max(0.0).sqrt()
}

fn compact_1by1(mut x: u32) -> u32 {
    // TODO: as of Haswell, the PEXT instruction could do all this in a
    // single instruction.
    // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
    x &= 0x55555555;
    // x = --fe --dc --ba --98 --76 --54 --32 --10
    x = (x ^ (x >> 1)) & 0x33333333;
    // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
    x = (x ^ (x >> 2)) & 0x0f0f0f0f;
    // x = ---- ---- fedc ba98 ---- ---- 7654 3210
    x = (x ^ (x >> 4)) & 0x00ff00ff;
    // x = ---- ---- ---- ---- fedc ba98 7654 3210
    x = (x ^ (x >> 8)) & 0x0000ffff;

    x
}

fn demux_float(f: Float) -> Point2f {
    assert!(f >= 0.0 && f < 1.0);

    let v = (f * (1u64 << 32) as Float) as u64;
    assert!(v < 0x100000000);
    let bits = [compact_1by1(v as u32), compact_1by1((v >> 1) as u32)];

    Point2f::new(
        bits[0] as Float / (1 << 16) as Float,
        bits[1] as Float / (1 << 16) as Float
    )
}

pub fn create_hair_material(mp: &mut TextureParams) -> Materials {
    let mut sigmaa = mp.get_spectrumtexture_ornull("sigma_a");
    let color = mp.get_spectrumtexture_ornull("color");
    let eumelanin = mp.get_floattexture_ornull("eumelanin");
    let pheomelanin = mp.get_floattexture_ornull("pheomelanin");

    if sigmaa.is_some() {
        if color.is_some() {
            warn!("Ignoring \"color\" parameter since \"sigma_a\" was provided.");
        }
        if eumelanin.is_some() {
            warn!("Ignoring \"eumelanin\" paramter since \"sigma_a\" was provided.");
        }
        if pheomelanin.is_some() {
            warn!("Ignoring \"pheomelanin\" parameter since \"sigma_a\" was provided.");
        }
    } else if color.is_some() {
        if sigmaa.is_some() {
            warn!("Ignoring \"sigma_a\" parameter since \"color\" was provided.");
        }
        if eumelanin.is_some() {
            warn!("Ignoring \"eumelanin\" paramter since \"color\" was provided.");
        }
        if pheomelanin.is_some() {
            warn!("Ignoring \"pheomelanin\" parameter since \"color\" was provided.");
        }
    } else if eumelanin.is_some() || pheomelanin.is_some() {
        if sigmaa.is_some() {
            warn!("Ignoring \"sigma_a\" paramter since \"eumelanin\"/\"pheomelanin\" was provided");
        }
        if color.is_some() {
            warn!("Ignoring \"color\" paramter since \"eumelanin\"/\"pheomelanin\" was provided");
        }
    } else {
        let sig: TextureSpec = ConstantTexture::new(HairBSDF::sigmaa_from_concentration(1.3, 0.0)).into();
        sigmaa = Some(Arc::new(sig))
    }

    let eta = mp.get_floattexture("eta", 1.55);
    let betam = mp.get_floattexture("beta_m", 0.3);
    let betan = mp.get_floattexture("beta_n", 0.3);
    let alpha = mp.get_floattexture("alpha", 2.0);

    HairMaterial::new(
        sigmaa, color, eumelanin, pheomelanin,
        eta, betam, betan, alpha
    ).into()
}