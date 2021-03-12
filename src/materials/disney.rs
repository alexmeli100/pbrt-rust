use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, Materials, TransportMode, bump};
use crate::core::interaction::{SurfaceInteraction, InteractionData, Interaction};
use bumpalo_herd::Member;
use crate::core::pbrt::{Float, clamp, lerp, INV_PI, PI, INFINITY};
use crate::core::spectrum::Spectrum;
use crate::core::reflection::{BxDFType, BxDF, abs_cos_theta, same_hemisphere, reflect, Fresnel, fr_dielectric, tan2_theta, cos2_theta, cos2_phi, sin2_phi, tan_theta, BSDF, BxDFs, cos_theta, SpecularTransmission, Fresnels, MicrofacetReflection, MicrofacetTransmission, LambertianTransmission};
use crate::core::geometry::point::Point2f;
use crate::core::geometry::vector::Vector3f;
use crate::{get_type, matches_flags_bxdf};
use std::fmt::{Display, Formatter, Result};
use crate::core::primitive::Primitive;
use crate::core::geometry::geometry::spherical_direction;
use crate::core::microfacet::{MicrofacetDistribution, trowbridge_reitz_sample, MicrofacetDistributions, TrowbridgeReitzDistribution};
use crate::pdf;
use crate::core::geometry::normal::Normal3f;
use crate::core::bssrdf::{SeparableBSSRDF, BSSRDF, SeparableBSSRDFAdapter, fresnel_moment1};
use crate::core::scene::Scene;
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::paramset::TextureParams;

macro_rules! sqr {
    ($x:expr) => {{ $x * $x }}
}

fn schlick_weight(cos_theta: Float) -> Float {
    let m = clamp(1.0 - cos_theta, 0.0, 1.0);

    (m * m) * (m * m) * m
}

fn fr_schlick(r0: Float, cos_theta: Float) -> Float {
    lerp(schlick_weight(cos_theta), r0, 1.0)
}

fn fr_schlicks(r0: &Spectrum, cos_theta: Float) -> Spectrum {
    lerp(schlick_weight(cos_theta), *r0, Spectrum::new(1.0))
}

fn schlickr0_from_eta(eta: Float) -> Float {
    sqr!(eta - 1.0) / sqr!(eta + 1.0)
}

pub struct DisneyDiffuse {
    r       : Spectrum,
    bx_type : u8
}

impl DisneyDiffuse {
    pub fn new(r: Spectrum) -> Self {
        let bx_type = BxDFType::Reflection as u8 | BxDFType::Diffuse as u8;

        Self { r, bx_type }
    }
}

impl BxDF for DisneyDiffuse {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let fo = schlick_weight(abs_cos_theta(wo));
        let fi = schlick_weight(abs_cos_theta(wi));

        // Diffues fresnel -go from 1 at normal incidence to 0.5 at grazing
        // Burley 2015, eq (4)
        self.r * INV_PI * (1.0 - fo / 2.0) * (1.0 - fi / 2.0)

    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.r
    }

    fn rho2(&self, _nsamples: usize, _u1: &[Point2f], _u2: &[Point2f]) -> Spectrum {
        self.r
    }
}

impl Display for DisneyDiffuse {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneyDiffuse R: {} ]", self.r)
    }
}

pub struct DisneyFakeSS {
    r           : Spectrum,
    roughness   : Float,
    bx_type     : u8
}

impl DisneyFakeSS {
    pub fn new(r: Spectrum, roughness: Float) -> Self {
        let bx_type = BxDFType::Reflection as u8 | BxDFType::Diffuse as u8;

        Self { r, roughness, bx_type }
    }
}

impl BxDF for DisneyFakeSS {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let mut wh = *wi + *wo;
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }

        wh = wh.normalize();
        let cos_thetad = wi.dot(&wh);

        // Fss90 used to "flatten" retroreflection based on roughness
        let fss90 = cos_thetad * cos_thetad * self.roughness;
        let fo = schlick_weight(abs_cos_theta(wo));
        let fi = schlick_weight(abs_cos_theta(wi));
        let fss = lerp(fo, 1.0, fss90) * lerp(fi, 1.0, fss90);
        // 1.25 scale is used to (roughly) preserve albedo
        let ss = 1.25 * (fss * (1.0 / (abs_cos_theta(wo) + abs_cos_theta(wi)) - 0.5) + 0.5);

        self.r / INV_PI * ss
    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.r
    }

    fn rho2(&self, _nsamples: usize, _u1: &[Point2f], _u2: &[Point2f]) -> Spectrum {
        self.r
    }
}

impl Display for DisneyFakeSS {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneyFakeSS R: {} roughness: {} ]", self.r, self.roughness)
    }
}

pub struct DisneyRetro {
    r           : Spectrum,
    roughness   : Float,
    bx_type     : u8
}

impl DisneyRetro {
    pub fn new(r: Spectrum, roughness: Float) -> Self {
        let bx_type = BxDFType::Reflection as u8 | BxDFType::Diffuse as u8;

        Self { r, roughness, bx_type }
    }
}

impl BxDF for DisneyRetro {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let mut wh = *wi + *wo;
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }

        wh = wh.normalize();
        let cos_thetad = wi.dot(&wh);

        let fo = schlick_weight(abs_cos_theta(&wo));
        let fi = schlick_weight(abs_cos_theta(&wi));
        let Rr = 2.0 * self.roughness * cos_thetad * cos_thetad;

        // Burley 2015, eq (4).
        self.r * INV_PI * Rr * (fo + fi + fo * fi * (Rr - 1.0))
    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.r
    }

    fn rho2(&self, _nsamples: usize, _u1: &[Point2f], _u2: &[Point2f]) -> Spectrum {
        self.r
    }
}

impl Display for DisneyRetro {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneyRetro R: {} roughness: {} ]", self.r, self.roughness)
    }
}

pub struct DisneySheen {
    r       : Spectrum,
    bx_type : u8
}

impl DisneySheen {
    pub fn new(r: Spectrum) -> Self {
        let bx_type = BxDFType::Reflection as u8 | BxDFType::Diffuse as u8;

        Self { r, bx_type }
    }
}

impl BxDF for DisneySheen {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let mut wh = *wi + *wo;
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }

        wh = wh.normalize();
        let cos_thetad = wi.dot(&wh);

        self.r * schlick_weight(cos_thetad)
    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.r
    }

    fn rho2(&self, _nsamples: usize, _u1: &[Point2f], _u2: &[Point2f]) -> Spectrum {
        self.r
    }
}

impl Display for DisneySheen {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneySheen R: {} ]", self.r)
    }
}

pub struct DisneyClearcoat {
    gloss   : Float,
    weight  : Float,
    bx_type : u8
}

impl DisneyClearcoat {
    pub fn new(gloss: Float, weight: Float) -> Self {
        let bx_type = BxDFType::Reflection as u8 | BxDFType::Glossy as u8;

        Self { gloss, weight, bx_type }
    }
}

fn gtr1(cos_theta: Float, alpha: Float) -> Float {
    let alpha2 = alpha * alpha;

    (alpha2 - 1.0) / (PI * alpha2.ln() * (1.0 + (alpha2 - 1.0) * cos_theta * cos_theta))
}

fn smithg_ggx(cos_theta: Float, alpha: Float) -> Float {
    let alpha2 = alpha * alpha;
    let cos_theta2 = cos_theta * cos_theta;

    1.0 / (cos_theta + (alpha2 + cos_theta2 - alpha2 * cos_theta2))
}

impl BxDF for DisneyClearcoat {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let mut wh = *wi + *wo;
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }
        wh = wh.normalize();

        // Clearcoat has ior = 1.5 hardcoded -> F0 = 0.04. It then uses the
        // GTR1 distribution, which has even fatter tails than Trowbridge-Reitz
        // (which is GTR2).
        let Dr = gtr1(abs_cos_theta(&wh), self.gloss);
        let Fr = fr_schlick(0.04, wo.dot(&wh));
        // The geometric term always based on alpha = 0.25
        let Gr = smithg_ggx(abs_cos_theta(wo), 0.25) * smithg_ggx(abs_cos_theta(wi), 0.25);

        Spectrum::new(Fr * self.weight * Gr * Dr / 4.0)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, u: &Point2f, pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        if wo.z == 0.0 { return Spectrum::new(0.0); }

        let alpha2 = self.gloss * self.gloss;
        let cos_theta = (((1.0 - alpha2.powf(1.0 - u[0])) / (1.0 - alpha2)).max(0.0)).sqrt();
        let sin_theta = ((1.0 - cos_theta * cos_theta).max(0.0)).sqrt();
        let phi = 2.0 * PI * u[1];
        let mut wh = spherical_direction(sin_theta, cos_theta, phi);
        if !same_hemisphere(wo, &wh) { wh = -wh; }

        *wi = reflect(wo, &wh);
        if !same_hemisphere(wo, &wh) { return Spectrum::new(0.0); }
        *pdf = self.pdf(wo, wi);

        self.f(wo, wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        if !same_hemisphere(wo, wi) { return 0.0 }

        let mut wh = *wi + *wi;
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return 0.0; }
        wh = wh.normalize();

        // The sampling routine samples wh exactly from the GTR1 distribution.
        // Thus, the final value of the PDF is just the value of the
        // distribution for wh converted to a mesure with respect to the
        // surface normal.
        let Dr = gtr1(abs_cos_theta(&wh), self.gloss);

        Dr * abs_cos_theta(&wh) / (4.0 * wo.dot(&wh))
    }
}

impl Display for DisneyClearcoat {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneyClearcoat weight: {} gloss: {} ]", self.weight, self.gloss)
    }
}

pub struct DisneyFresnel {
    r0      : Spectrum,
    metallic: Float,
    eta     : Float
}

impl DisneyFresnel {
    pub fn new(r0: Spectrum, metallic: Float, eta: Float) -> Self {
        Self { r0, metallic, eta }
    }
}

impl Fresnel for DisneyFresnel {
    fn evaluate(&self, cosi: f32) -> Spectrum {
        lerp(
            self.metallic,
            Spectrum::new(fr_dielectric(cosi, 1.0, self.eta)),
            fr_schlicks(&self.r0, cosi))
    }
}

impl Display for DisneyFresnel {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f,
            "[ DisneyFresnel R0: {} metallic: {} eta: {} ]",
            self.r0, self.metallic, self.eta)
    }
}

pub struct DisneyMicrofacetDistribution {
    alphax      : Float,
    alphay      : Float,
    samplevis   : bool
}

impl DisneyMicrofacetDistribution {
    pub fn new(alphax: Float, alphay: Float, samplevis: bool) -> Self {
        Self { alphax, alphay, samplevis }
    }
}

impl MicrofacetDistribution for DisneyMicrofacetDistribution {
    fn d(&self, wh: &Vector3f) -> f32 {
        let tan2_theta = tan2_theta(wh);

        if tan2_theta.is_infinite() { return 0.0; }

        let cos4_theta = cos2_theta(wh) * cos2_theta(wh);
        let e = (cos2_phi(wh) / (self.alphax * self.alphax) +
            sin2_phi(wh) / (self.alphay * self.alphay)) *
            tan2_theta;

        1.0 / (PI * self.alphax * self.alphay * cos4_theta * (1.0 + e) * (1.0 + e))
    }

    fn lambda(&self, w: &Vector3f) -> f32 {
        let abs_tan_theta = tan_theta(w).abs();
        if abs_tan_theta.is_infinite() { return 0.0; }

        // Compute alpha for direction w
        let alpha = (cos2_phi(w) * self.alphax * self.alphax +
            sin2_phi(w) * self.alphay * self.alphay).sqrt();
        let alpha2_tan2_theta = (alpha * abs_tan_theta) * (alpha * abs_tan_theta);

        (-1.0 + (1.0 + alpha2_tan2_theta).sqrt()) / 2.0
    }

    fn g(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        // Disney uses the separable masking-shadowing model
        self.g1(wo) * self.g1(wi)
    }

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f {
        let mut wh: Vector3f;

        if !self.samplevis {
            let cos_theta: Float;
            let mut phi = (2.0 * PI) * u[1];
            if self.alphax == self.alphay {
                let tan_theta2 = self.alphax * self.alphax * u[0] / (1.0 - u[0]);
                cos_theta = 1.0 / (1.0 + tan_theta2).sqrt();
            } else {
                phi = (self.alphay / self.alphax * (2.0 * PI * u[1] + 0.5 * PI).tan()).tan();
                if u[1] > 0.5 { phi += PI; }
                let sin_phi = phi.sin();
                let cos_phi = phi.cos();
                let alphax2 = self.alphax * self.alphax;
                let alphay2 = self.alphay * self.alphay;
                let alpha2 = 1.0 / (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
                let tan_theta2 = alpha2 * u[0] / (1.0 - u[0]);
                cos_theta = 1.0 / (1.0 + tan_theta2).sqrt();
            }

            let sin_theta = ((1.0 - cos_theta * cos_theta).max(0.0)).sqrt();
            wh = spherical_direction(sin_theta, cos_theta, phi);
            if !same_hemisphere(wo, &wh) { wh = -wh; }
        } else {
            let flip = wo.z < 0.0;
            let mwo = -*wo;
            wh = trowbridge_reitz_sample(
                if flip { &mwo } else { wo },
                self.alphax, self.alphay, u[0], u[1]);
            if flip { wh = -wh }
        }

        wh
    }

    pdf!();
}

impl Display for DisneyMicrofacetDistribution {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ DisneyMicrofacetDistribution alphax: {} alphay: {} ]",
               self.alphax, self.alphay)
    }
}

#[derive(Clone)]
pub struct DisneyBSSRDF {
    r           : Spectrum,
    d           : Spectrum,
    n           : Normal3f,
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

impl DisneyBSSRDF {
    pub fn new(
        s: &SurfaceInteraction, mat: Option<Arc<Materials>>,
        mode: TransportMode, eta: Float, r: Spectrum,
        d: Spectrum) -> Self {
        let ns = s.shading.n;
        let ss = s.shading.dpdu.normalize();
        let ts = ns.cross_vec(&ss);



        let po = InteractionData {
            p: s.p,
            time: s.time,
            p_error: s.p_error,
            wo: s.wo,
            n: s.n,
            medium_interface: None
        };

        let n = s.shading.n;

        let material = mat.unwrap();

        Self {
            ns, ss, ts, mode, material,
             eta, po, r, d, n
        }
    }
}

impl BSSRDF for DisneyBSSRDF {
    fn s(&self, pi: &SurfaceInteraction, wi: &Vector3f) -> Spectrum {
        let a = (pi.p - self.po.p).normalize();
        let mut fade = 1.0;
        let n = Vector3f::from(self.n);
        let cos_theta = a.dot(&n);
        if cos_theta < 0.0 {
            // Point on or above surface plane
            let sin_theta = ((1.0 - cos_theta - cos_theta).max(0.0)).sqrt();
            let a2 = n * sin_theta - (a -n * cos_theta) * cos_theta / sin_theta;
            fade = pi.shading.n.dot_vec(&a2).max(0.0);
        }

        let fo = schlick_weight(abs_cos_theta(&self.po.wo));
        let fi = schlick_weight(abs_cos_theta(wi));

        self.sp(pi) * fade * (1.0 - fo / 2.0) * (1.0 - fi / 2.0) / PI
    }

    fn sample_s<'a: 'a>(
        &self, scene: &Scene, u1: f32, u2: &Point2f, arena: &Member<'a>,
        si: &mut SurfaceInteraction<'a>, pdf: &mut f32) -> Spectrum {
        // TODO ProfilePhase
        let Sp = self.sample_sp(scene, u1, u2, si, pdf);

        if !Sp.is_black() {
            // Initialize material model at sampled surface interaction
            let mut bsdf = BSDF::new(si, 1.0);
            let bxdf: &mut BxDFs = arena.alloc(
                SeparableBSSRDFAdapter::new(self.clone().into()).into());
            bsdf.add(bxdf);
            si.bsdf = Some(bsdf);
            si.wo = Vector3f::from(si.shading.n);
        }

        Sp
    }
}

impl SeparableBSSRDF for DisneyBSSRDF {
    fn sw(&self, w: &Vector3f) -> Spectrum {
        let c = 1.0 - 2.0 * fresnel_moment1(1.0 / self.eta);

        Spectrum::new((1.0 - fr_dielectric(cos_theta(w), 1.0, self.eta)) / (c * PI))
    }

    fn sp(&self, pi: &SurfaceInteraction) -> Spectrum {
        self.sr(self.po.p.distance(&pi.p))
    }

    fn sample_sp<'a: 'a>(
        &self, scene: &Scene, u1: Float, u2: &Point2f,
        si: &mut SurfaceInteraction<'a>, pdf: &mut Float) -> Spectrum {
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

    fn pdf_sp(&self, pi: &SurfaceInteraction) -> Float {
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

    fn sr(&self, mut r: Float) -> Spectrum {
        // TODO: ProfilePhase
        if r < 1.0e-6 { r = 1.0e-6; }

        self.r * ((-Spectrum::new(r) / self.d).exp() + (-Spectrum::new(r) / (self.d * 3.0)).exp()) /
            (self.d * 8.0 * PI  * r)
    }

    fn sample_sr(&self, ch: usize, mut u: Float) -> Float {
        if u < 0.25 {
            // Sample the first exponential
            u = (u * 4.0).min(ONE_MINUS_EPSILON);

            self.d[ch] * (1.0 / (1.0 - u)).ln()
        } else {
            u = ((u - 0.25) / 0.75).min(ONE_MINUS_EPSILON);

            3.0 * self.d[ch] * (1.0 / (1.0 - u)).ln()
        }
    }

    fn pdf_sr(&self, ch: usize, r: Float) -> Float {
        0.25 * (-r / self.d[ch]).exp() / (2.0 * PI * self.d[ch] * r) +
        0.75 * (-r / (3.0 * self.d[ch])).exp() / (6.0 * PI * self.d[ch] * r)
    }

    fn mode(&self) -> TransportMode {
        self.mode
    }

    fn eta(&self) -> Float {
        self.eta
    }
}

#[derive(Clone)]
pub struct DisneyMaterial {
    color           : Arc<TextureSpec>,
    eta             : Arc<TextureFloat>,
    metallic        : Arc<TextureFloat>,
    roughness       : Arc<TextureFloat>,
    specular_tint   : Arc<TextureFloat>,
    anisotropic     : Arc<TextureFloat>,
    sheen           : Arc<TextureFloat>,
    sheen_tint      : Arc<TextureFloat>,
    clearcoat       : Arc<TextureFloat>,
    clearcoat_gloss : Arc<TextureFloat>,
    spec_trans      : Arc<TextureFloat>,
    flatness        : Arc<TextureFloat>,
    diff_trans      : Arc<TextureFloat>,
    bumpmap         : Option<Arc<TextureFloat>>,
    scatter_distance: Arc<TextureSpec>,
    thin            : bool
}

impl DisneyMaterial {
    pub fn new(
        color: Arc<TextureSpec>, metallic: Arc<TextureFloat>,
        eta: Arc<TextureFloat>, roughness: Arc<TextureFloat>,
        specular_tint: Arc<TextureFloat>, anisotropic: Arc<TextureFloat>,
        sheen: Arc<TextureFloat>, sheen_tint: Arc<TextureFloat>,
        clearcoat: Arc<TextureFloat>, clearcoat_gloss: Arc<TextureFloat>,
        spec_trans: Arc<TextureFloat>, scatter_distance: Arc<TextureSpec>,
        thin: bool, flatness: Arc<TextureFloat>, diff_trans: Arc<TextureFloat>,
        bumpmap: Option<Arc<TextureFloat>>) -> Self {
        Self {
            color, metallic, eta, roughness, specular_tint,
            spec_trans, anisotropic, sheen, sheen_tint,
            clearcoat, clearcoat_gloss, scatter_distance,
            thin, flatness, diff_trans, bumpmap
        }
    }
}

impl Material for DisneyMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>,
        arena: &Member<'b>, mat: Option<Arc<Materials>>,
        mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        // Evaluate textures for DisneyMaterial and allocate BRDF
        let mut bsdf = BSDF::new(si, 1.0);

        // Diffuse
        let c = self.color.evaluate(si).clamps(0.0, INFINITY);
        let mweight = self.metallic.evaluate(si);
        let e = self.eta.evaluate(si);
        let strans = self.spec_trans.evaluate(si);
        let dweight = (1.0 - mweight) * (1.0 - strans);
        let dt = self.diff_trans.evaluate(si) / 2.0;
        let rough = self.roughness.evaluate(si);
        let lum = c.y();
        // Normalize lum to isoluate hue+sat
        let ctint = if lum > 0.0 { c / lum } else { Spectrum::new(1.0) };

        let sheen_weight = self.sheen.evaluate(si);
        let mut csheen = Spectrum::default();
        if sheen_weight > 0.0 {
            let stint = self.sheen_tint.evaluate(si);
            csheen = lerp(stint, Spectrum::new(1.0), ctint);
        }

        if dweight > 0.0 {
            if self.thin {
                let flat = self.flatness.evaluate(si);
                // Blend between DisneyDiffuse and fake subsurface based on
                // flatness.  Additionally, weight using diffTrans.
                let bx: &mut BxDFs = arena.alloc(
                    DisneyDiffuse::new(c * dweight * flat * (1.0 - dt)).into());
                bsdf.add(bx);
                let bxt: &mut BxDFs = arena.alloc(
                    DisneyFakeSS::new(c * (1.0 - dt) * flat * dweight, rough).into());
                bsdf.add(bxt);
            } else {
                let sd = self.scatter_distance.evaluate(si);

                if sd.is_black() {
                    // No subsurface scattering; use regular (Fresnel modified)
                    // diffuse.
                    let bxdf: &mut BxDFs = arena.alloc(DisneyDiffuse::new(c * dweight).into());
                    bsdf.add(bxdf);
                } else {
                    // Use a BSSRDF instead
                    let bxdf: &mut BxDFs = arena.alloc(
                        SpecularTransmission::new(&Spectrum::new(1.0), 1.0, e, mode).into());
                    bsdf.add(bxdf);
                    let bssrdf = DisneyBSSRDF::new(si, mat, mode,e, c * dweight, sd ).into();
                    si.bssrdf = Some(bssrdf);
                }
            }

            // Retro-relfection
            let bxdf: &mut BxDFs = arena.alloc(DisneyRetro::new(c * dweight, rough).into());
            bsdf.add(bxdf);

            // Sheen if enabled
            if sheen_weight > 0.0 {
                let bxdf: &mut BxDFs = arena.alloc(DisneySheen::new(csheen * sheen_weight * dweight).into());
                bsdf.add(bxdf);
            }
        }

        // Create the microfacet distribution for metallic and/or specular
        // transmission.
        let aspect = (1.0 - self.anisotropic.evaluate(si) * 0.9).sqrt();
        let ax = (sqr!(rough) / aspect).max(0.001);
        let ay = (sqr!(rough) * aspect).max(0.001);
        let dis: &mut MicrofacetDistributions =
            arena.alloc(DisneyMicrofacetDistribution::new(ax, ay, true).into());

        // Specular is Trowbridge-Reitz with a modified Fresnel function
        let spec_tint = self.specular_tint.evaluate(si);
        let cspec0 =
            lerp(mweight,
                 lerp(spec_tint, Spectrum::new(1.0) * schlickr0_from_eta(e), ctint), c);
        let fresnel: &mut Fresnels = arena.alloc(DisneyFresnel::new(cspec0, mweight, e).into());
        let bxdf: &mut BxDFs =
            arena.alloc(MicrofacetReflection::new(&Spectrum::new(1.0), dis, fresnel).into());
        bsdf.add(bxdf);

        // Clearcoat
        let cc = self.clearcoat.evaluate(si);
        if cc > 0.0 {
            let bxdf: &mut BxDFs = arena.alloc(
                DisneyClearcoat::new(cc, lerp(self.clearcoat_gloss.evaluate(si), 0.1, 0.001)).into());
            bsdf.add(bxdf);
        }

        // BTDF
        if strans > 0.0 {
            let T = c.sqrt() * strans;
            if self.thin {
                // Scale roughness based on IOR
                let rscaled = (0.65 * e - 0.35) * rough;
                let ax = (sqr!(rscaled) / aspect).max(0.001);
                let ay = (sqr!(rscaled) * aspect).max(0.001);
                let sdistrib: &mut MicrofacetDistributions = arena.alloc(
                    TrowbridgeReitzDistribution::new(ax, ay, true).into());
                let bxdf: &mut BxDFs = arena.alloc(
                    MicrofacetTransmission::new(&T, sdistrib, 1.0, e, mode).into());
                bsdf.add(bxdf);
            } else {
                let bxdf: &mut BxDFs = arena.alloc(
                    MicrofacetTransmission::new(&T, dis, 1.0, e, mode).into());
                bsdf.add(bxdf);
            }
        }

        if self.thin {
            // Lambertian, weighted by (1 - diffTrans
            let bxdf: &mut BxDFs = arena.alloc(
                LambertianTransmission::new(&(c * dt)).into());
            bsdf.add(bxdf);
        }

        si.bsdf = Some(bsdf);
    }
}

pub fn create_disney_material(mp: &mut TextureParams) -> Materials {
    let color =
        mp.get_spectrumtexture("color", Spectrum::new(0.5));
    let metallic =
        mp.get_floattexture("metallic", 0.0);
    let eta =
        mp.get_floattexture("eta", 1.5);
    let roughness =
        mp.get_floattexture("roughness", 0.5);
    let specular_tint =
        mp.get_floattexture("speculartint", 0.0);
    let anisotropic =
        mp.get_floattexture("anisotropic", 0.0);
    let sheen =
        mp.get_floattexture("sheen", 0.0);
    let sheen_tint =
        mp.get_floattexture("sheentint", 0.5);
    let clearcoat =
        mp.get_floattexture("clearcoat", 0.0);
    let clearcoat_gloss =
        mp.get_floattexture("clearcoatgloss", 1.0);
    let spectrans =
        mp.get_floattexture("spectrans", 0.0);
    let scatterdis =
        mp.get_spectrumtexture("scatterdistance", Spectrum::new(0.0));
    let thin =
        mp.find_bool("thin", false);
    let flatness =
        mp.get_floattexture("flatness", 0.0);
    let difftrans =
        mp.get_floattexture("difftrans", 0.0);
    let bumpmap =
        mp.get_floattexture_ornull("bumpmap");

    DisneyMaterial::new(
        color, metallic, eta, roughness, specular_tint,
        anisotropic, sheen, sheen_tint, clearcoat,
        clearcoat_gloss, spectrans, scatterdis,
        thin, flatness, difftrans, bumpmap).into()
}