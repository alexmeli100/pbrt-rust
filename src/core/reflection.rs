
use enum_dispatch::enum_dispatch;
use std::fmt::{Display, Result, Formatter};
use std::path::Path;
use log::{error, debug};
use byteorder::{ReadBytesExt, LittleEndian};
use crate::core::geometry::vector::{Vector3f};
use crate::core::pbrt::{Float, clamp, INV_PI, PI, radians, INFINITY};
use crate::core::geometry::point::Point2f;
use crate::core::spectrum::{Spectrum, SpectrumType};
use crate::core::material::TransportMode;
use crate::core::geometry::normal::Normal3f;
use crate::core::sampling::{cosine_sample_hemisphere, uniform_sample_hemisphere, uniform_hemisphere_pdf};
use crate::core::microfacet::{MicrofacetDistributions, MicrofacetDistribution};
use std::fs::File;
use std::io::Read;
use std::sync::Arc;
use crate::core::interpolation::{catmull_rom_weights, fourier, sample_catmull_rom_2d, sample_fourier};
use smallvec::{SmallVec, smallvec};
use crate::core::interaction::{SurfaceInteraction};
use crate::core::rng::ONE_MINUS_EPSILON;
use crate::core::mipmap::Clampable;
use crate::materials::hair::HairBSDF;
use crate::core::bssrdf::SeparableBSSRDFAdapter;
use crate::materials::disney::{DisneyFakeSS, DisneyDiffuse, DisneyRetro, DisneySheen, DisneyClearcoat, DisneyFresnel};

const MAX_BXDFS: usize = 8;

pub fn fr_dielectric(mut cos_thetai: Float, mut etai: Float, mut etat: Float) -> Float {
    cos_thetai = clamp(cos_thetai, -1.0, 1.0);

    // Potentially swap indices of refraction
    let entering = cos_thetai > 0.0;
    if !entering {
        std::mem::swap(&mut etai, &mut etat);
        cos_thetai = cos_thetai.abs();
    }

    // Compute cosThetaT using snell's law
    let sin_thetai = ((1.0 - cos_thetai * cos_thetai).max(0.0)).sqrt();
    let sin_thetat = etai / etat * sin_thetai;

    // Handle total internal reflection
    if sin_thetat >= 1.0 { return 1.0; }
    let cos_thetat = ((1.0 - sin_thetat * sin_thetat).max(0.0)).sqrt();
    let r_parl = ((etat * cos_thetai) - (etai * cos_thetat)) /
                 ((etat * cos_thetai) + (etai * cos_thetat));
    let r_perp = ((etai * cos_thetai) - (etat * cos_thetat)) /
                 ((etai * cos_thetai) + (etat * cos_thetat));

    (r_parl * r_parl + r_perp * r_perp) / 2.0
}

fn fr_conductor(mut cos_thetai: Float, etai: &Spectrum, etat: &Spectrum, k: &Spectrum) -> Spectrum {
    cos_thetai = clamp(cos_thetai, -1.0, 1.0);
    let eta = etat / etai;
    let etak = k / etai;

    let cos_thetai2 = cos_thetai * cos_thetai;
    let sin_thetai2 = 1.0 - cos_thetai2;
    let eta2 = eta * eta;
    let etak2 = etak * etak;

    let t0 = eta2 - etak2 - Spectrum::new(sin_thetai2);
    let a2plusb2 = (t0 * t0 + eta2 * etak2 * 4.0).sqrt();
    let t1 = a2plusb2 + Spectrum::new(cos_thetai2);
    let a = ((a2plusb2 + t0) * 0.5).sqrt();
    let t2 = a * cos_thetai * 2.0;
    let Rs = (t1 - t2) / (t1 + t2);

    let t3 = a2plusb2 * cos_thetai2 + Spectrum::new(sin_thetai2 * sin_thetai2);
    let t4 = t2 * sin_thetai2;
    let Rp = Rs * (t3 - t4) / (t3 + t4);

    (Rp + Rs) * 0.5
}

#[inline(always)]
pub fn cos_theta(w: &Vector3f) -> Float {
    w.z
}

#[inline(always)]
pub fn cos2_theta(w: &Vector3f) -> Float {
    w.z * w.z
}

#[inline(always)]
pub fn abs_cos_theta(w: &Vector3f) -> Float {
    w.z.abs()
}

#[inline(always)]
pub fn sin2_theta(w: &Vector3f) -> Float {
    (1.0 - cos2_theta(w)).max(0.0)
}

#[inline(always)]
pub fn sin_theta(w: &Vector3f) -> Float {
    sin2_theta(w).sqrt()
}

#[inline(always)]
pub fn tan_theta(w: &Vector3f) -> Float {
    sin_theta(w) / cos_theta(w)
}

#[inline(always)]
pub fn tan2_theta(w: &Vector3f) -> Float {
    sin2_theta(w) / cos2_theta(w)
}

#[inline(always)]
pub fn cos_phi(w: &Vector3f) -> Float {
    let stheta = sin_theta(w);

    if stheta == 0.0 {
        1.0
    } else {
        clamp(w.x / stheta, -1.0, 1.0)
    }
}

#[inline(always)]
pub fn cos2_phi(w: &Vector3f) -> Float {
    cos_phi(w) * cos_phi(w)
}

#[inline(always)]
pub fn sin_phi(w: &Vector3f) -> Float {
    let stheta = sin_theta(w);

    if stheta == 0.0 {
        0.0
    } else {
        clamp(w.y / stheta, -1.0, 1.0)
    }
}

#[inline(always)]
pub fn sin2_phi(w: &Vector3f) -> Float {
    sin_phi(w) * sin_phi(w)
}

#[inline(always)]
pub fn cos_d_phi(wa: &Vector3f, wb: &Vector3f) -> Float {
    let waxy = wa.x * wa.x + wa.y * wa.y;
    let wbxy = wb.x * wb.x + wb.y * wb.y;

    if waxy == 0.0 || wbxy == 0.0 { return 1.0; }

    clamp((wa.x * wb.x + wa.y * wb.y) / (waxy * wbxy).sqrt(), -1.0, 1.0)
}

#[inline(always)]
pub fn reflect(wo: &Vector3f, n: &Vector3f) -> Vector3f {
    -(*wo)  + *n * 2.0 * wo.dot(n)
}

pub fn refract(wi: &Vector3f, n: &Normal3f, eta: Float, wt: &mut Vector3f) -> bool {
    // Compute costheta using Snell's law
    let cos_thetai = n.dot_vec(wi);
    let sin2_thetai = (1.0 - cos_thetai * cos_thetai).max(0.0);
    let sin2_thetat = eta * eta * sin2_thetai;

    // Handle internal reflection for transmission
    if sin2_thetat >= 1.0 { return false; }

    let cos_thetat = (1.0 - sin2_thetat).sqrt();
    *wt = Vector3f::from(*n) * (eta * cos_thetai - cos_thetat) + -*wi * eta;

    true

}

#[inline(always)]
pub fn same_hemisphere(w: &Vector3f, wp: &Vector3f) -> bool {
    w.z * wp.z > 0.0
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum BxDFType {
    Reflection      = 1 << 0,
    Transmission    = 1 << 1,
    Diffuse         = 1 << 2,
    Glossy          = 1 << 3,
    Specular        = 1 << 4,
    All             = 31
}

#[derive(Default)]
pub struct FourierBSDFTable {
    eta             : Float,
    nmax            : i32,
    pub nchannels   : i32,
    nmu             : i32,
    mu              : Vec<Float>,
    m               : Vec<i32>,
    aoffset         : Vec<i32>,
    a               : Vec<Float>,
    a0              : Vec<Float>,
    cdf             : Vec<Float>,
    recip           : Vec<Float>
}

#[inline(always)]
fn read32(file: &mut File, buf: &mut [i32]) -> bool {
    file.read_i32_into::<LittleEndian>(buf).is_ok()
}

#[inline(always)]
fn readfloat(file: &mut File, buf: &mut [f32]) -> bool {
    file.read_f32_into::<LittleEndian>(buf).is_ok()
}

impl FourierBSDFTable {
    pub fn get_ak(&self, offseti: i32, offseto: i32, mptr: &mut i32) -> i32 {
        let offset = (offseto * self.nmu + offseti) as usize;
        *mptr = self.m[offset];

        self.aoffset[offset]
    }

    pub fn get_weights_and_offset(&self, cos_theta: Float, offset: &mut i32, weights: &mut [Float; 4]) -> bool {
        catmull_rom_weights(self.nmu, &self.mu, cos_theta, offset, weights)
    }

    pub fn read<P>(filename: P, table: &mut FourierBSDFTable) -> bool
    where P: AsRef<Path>  {
        macro_rules! error_read {
            () => {
                error!("Tabulated BSDF file \"{}\" has incompatible file \
                   format or version.", filename.as_ref().to_str().unwrap());
                return false;
            }
        }

        table.nchannels = 0;
        let f = File::open(filename.as_ref());

        if f.is_err() {
            error!("Unable to open tabulated BSDF file \"{}\"",
                   filename.as_ref().to_str().unwrap());
            return false;
        }

        let mut file = f.unwrap();
        let header_exp = [b'S', b'C', b'A', b'T', b'F', b'U', b'N', b'\x01'];
        let mut buf = [0; 8];
        let res = file.read_exact(&mut buf);

        if res.is_err() || header_exp != buf {
            error_read!();
        }


        // last 3 integers are unused
        let mut buf = [0_i32; 9];

        if !read32(&mut file, &mut buf) {
            error_read!();
        }

        let flags = buf[0];
        table.nmu = buf[1];
        let ncoeffs = buf[2];
        table.nmax = buf[3];
        table.nchannels = buf[4];
        let nbases = buf[5];

        let mut buf = [0.0_f32; 1];

        if !readfloat(&mut file, &mut buf) {
            error_read!();
        }

        table.eta = buf[0];

        // 4 32 bit integers are unused
        let mut buf = [0_i32; 4];

        if !read32(&mut file,&mut buf) {
            error_read!();
        }

        /* Only a subset of BSDF files are supported for simplicity, in particular:
           monochromatic and
           RGB files with uniform (i.e. non-textured) material properties */
        if flags != 1 ||
            (table.nchannels != 1 && table.nchannels != 3) ||
            nbases != 1 {
            error_read!();
        }


        table.mu = vec![0.0; table.nmu as usize];
        table.cdf = vec![0.0; (table.nmu * table.nmu) as usize];
        table.a0 = vec![0.0; (table.nmu * table.nmu) as usize];
        table.aoffset = vec![0_i32; (table.nmu * table.nmu) as usize];
        table.m = vec![0_i32; (table.nmu * table.nmu) as usize];
        table.a = vec![0.0; ncoeffs as usize];
        let mut offset_and_length = vec![0_i32; (table.nmu * table.nmu) as usize * 2];

        if !readfloat(&mut file, &mut table.mu) ||
           !readfloat(&mut file, &mut table.cdf) ||
           !read32(&mut file, &mut offset_and_length) ||
           !readfloat(&mut file, &mut table.a) {
            error_read!();
        }

        for i in 0..(table.nmu * table.nmu) as usize {
            let offset = offset_and_length[2 * i];
            let length = offset_and_length[2 * i + 1];

            table.aoffset[i] = offset;
            table.m[i] = length;

            table.a0[i] = if length > 0 {
                table.a[offset as usize]
            } else {
                0.0
            }
        }

        table.recip = vec![0.0; table.nmax as usize];

        for i in 0..table.nmax as usize {
            table.recip[i] = 1.0 / i as Float;
        }

        true
    }
}

#[enum_dispatch]
pub enum BxDFs<'a> {
    DisneyDiffuse(DisneyDiffuse),
    DisneyFakeSS(DisneyFakeSS),
    DisneyRetro(DisneyRetro),
    DisneySheen(DisneySheen),
    DisneyClearcoat(DisneyClearcoat),
    OrenNayar(OrenNayar),
    HairBSDF(HairBSDF),
    FourierBSDF(FourierBSDF),
    ScaledBxDF(ScaledBxDF<'a>),
    FresnelBlend(FresnelBlend<'a>),
    FresnelSpecular(FresnelSpecular),
    SeparableBSSRDFAdapter(SeparableBSSRDFAdapter),
    SpecularTransmission(SpecularTransmission),
    LambertianReflection(LambertianReflection),
    LambertianTransmission(LambertianTransmission),
    SpecularReflection(SpecularReflection<'a>),
    MicrofacetReflection(MicrofacetReflection<'a>),
    MicrofacetTransmission(MicrofacetTransmission<'a>)
}

impl<'a> Display for BxDFs<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self {
            BxDFs::OrenNayar(ref b) => write!(f, "{}", b),
            BxDFs::HairBSDF(ref b) => write!(f, "{}", b),
            BxDFs::FourierBSDF(ref b) => write!(f, "{}", b),
            BxDFs::ScaledBxDF(ref b) => write!(f, "{}", b),
            BxDFs::FresnelBlend(ref b) => write!(f, "{}", b),
            BxDFs::FresnelSpecular(ref b) => write!(f, "{}", b),
            BxDFs::SpecularTransmission(ref b) => write!(f, "{}", b),
            BxDFs::SpecularReflection(ref b) => write!(f, "{}", b),
            BxDFs::LambertianTransmission(ref b) => write!(f, "{}", b),
            BxDFs::LambertianReflection(ref b) => write!(f, "{}", b),
            BxDFs::MicrofacetTransmission(ref b) => write!(f, "{}", b),
            BxDFs::MicrofacetReflection(ref b) => write!(f, "{}", b),
            BxDFs::SeparableBSSRDFAdapter(ref b) => write!(f, "{}", b),
            BxDFs::DisneyDiffuse(ref b) => write!(f, "{}", b),
            BxDFs::DisneyClearcoat(ref b) => write!(f, "{}", b),
            BxDFs::DisneySheen(ref b) => write!(f, "{}", b),
            BxDFs::DisneyRetro(ref b) => write!(f, "{}", b),
            BxDFs::DisneyFakeSS(ref b) => write!(f, "{}", b),

        }
    }
}

#[enum_dispatch(BxDFs)]
pub trait BxDF {
    fn matches_flags(&self, t: u8) -> bool;

    fn get_type(&self) -> u8;

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum;

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f,
                pdf: &mut Float, _sampled_type: &mut u8) -> Spectrum {
        *wi = cosine_sample_hemisphere(sample);

        if wo.z < 0.0 {
            wi.z *= -1.0;
        }

        *pdf = self.pdf(wo, wi);

        self.f(wo, wi)
    }

    fn rho(&self, wo: &Vector3f, nsamples: usize, samples: &[Point2f]) -> Spectrum {
        let mut r = Spectrum::new(0.0);

        for i in 0..nsamples {
            // Estimate one term of rho
            let mut wi = Vector3f::default();
            let mut pdf = 0.0;
            let f = self.sample_f(wo, &mut wi, &samples[i], &mut pdf, &mut 0);

            if pdf > 0.0 {
                r *= f * abs_cos_theta(&wi) / pdf
            }
        }

        r / nsamples as Float
    }
    fn rho2(&self, nsamples: usize, u1: &[Point2f], u2: &[Point2f]) -> Spectrum {
        let mut r = Spectrum::new(0.0);

        for i in 0..nsamples {
            // Estimate one term of rho
            let wo = uniform_sample_hemisphere(&u1[i]);
            let mut wi = Vector3f::default();
            let pdfo = uniform_hemisphere_pdf();
            let mut pdfi = 0.0;
            let f = self.sample_f(&wo, &mut wi, &u2[i], &mut pdfi, &mut 0);

            if pdfi > 0.0 {
                r *= f * abs_cos_theta(&wi) * abs_cos_theta(&wo) / (pdfo * pdfi);
            }
        }

        r / PI * nsamples as Float
    }
    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if same_hemisphere(wo, wi) {
            abs_cos_theta(wi) * INV_PI
        } else {
            0.0
        }
    }
}

#[macro_export]
macro_rules! matches_flags_bxdf {
    () => {
        fn matches_flags(&self, t: u8) -> bool {
            (self.bx_type & t) == self.bx_type
        }
    }
}

#[macro_export]
macro_rules! get_type {
    () => {
        fn get_type(&self) -> u8 {
            self.bx_type
        }
    }
}

pub struct ScaledBxDF<'a> {
    bxdf    : &'a BxDFs<'a>,
    scale   : Spectrum,
    //bx_type: u8
}

impl<'a> ScaledBxDF<'a> {
    pub fn new(bxdf: &'a BxDFs<'a>, scale: &Spectrum) -> Self {
        Self {
            bxdf,
            scale: *scale,
        }
    }
}

impl<'a> Display for ScaledBxDF<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ ScaledBxDF bxdf: {} scale {} ]", self.bxdf, self.scale)
    }
}

impl<'a> BxDF for ScaledBxDF<'a> {
    fn matches_flags(&self, t: u8) -> bool {
        self.bxdf.matches_flags(t)
    }

    fn get_type(&self) -> u8 {
        self.bxdf.get_type()
    }

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        self.scale * self.bxdf.f(wo, wi)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f,
                pdf: &mut Float, sampled_type: &mut u8) -> Spectrum {
        let f = self.bxdf.sample_f(wo, wi, sample, pdf, sampled_type);

        self.scale * f
    }

    fn rho(&self, wo: &Vector3f, nsamples: usize, samples: &[Point2f]) -> Spectrum {
        self.scale * self.bxdf.rho(wo, nsamples, samples)
    }

    fn rho2(&self, nsamples: usize, samples1: &[Point2f], samples2: &[Point2f]) -> Spectrum {
        self.scale * self.bxdf.rho2(nsamples, samples1, samples2)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        self.bxdf.pdf(wo, wi)
    }
}

#[enum_dispatch]
pub trait Fresnel: Display {
    fn evaluate(&self, cosi: Float) -> Spectrum;
}


#[enum_dispatch(Display, Fresnel)]
pub enum Fresnels {
    FresnelConductor(FresnelConductor),
    FresnelDielectric(FresnelDielectric),
    DisneyFresnel(DisneyFresnel),
    FresnelNoOp(FresnelNoOp)
}

// TODO: Fix display for fresnels
impl Display for Fresnels {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self {
            Fresnels::FresnelConductor(ref fr) => write!(f, "{}", fr),
            Fresnels::FresnelDielectric(ref fr) => write!(f, "{}", fr),
            Fresnels::FresnelNoOp(ref fr) => write!(f, "{}", fr),
            Fresnels::DisneyFresnel(ref fr) => write!(f, "{}", fr),
        }
    }
}

pub struct FresnelConductor {
    etai: Spectrum,
    etat: Spectrum,
    k   : Spectrum
}

impl FresnelConductor {
    pub fn new(etai: &Spectrum, etat: &Spectrum, k: &Spectrum) -> Self {
        Self {
            etai: *etai,
            etat: *etat,
            k: *k
        }
    }
}

impl Fresnel for FresnelConductor {
    fn evaluate(&self, cosi: f32) -> Spectrum {
        fr_conductor(cosi.abs(), &self.etai, &self.etat, &self.k)
    }
}

impl Display for FresnelConductor {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FresnelConductor etaI: {} etaT: {} k: {} ]",
               self.etai, self.etat, self.k)
    }
}

pub struct FresnelDielectric {
    etai: Float,
    etat: Float
}

impl FresnelDielectric {
    pub fn new(etai: Float, etat: Float) -> Self {
        Self { etai, etat}
    }
}

impl Fresnel for FresnelDielectric {
    fn evaluate(&self, cosi: f32) -> Spectrum {
        Spectrum::new(fr_dielectric(cosi, self.etai, self.etat))
    }
}

impl Display for FresnelDielectric {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FresnelDielectric etaI: {} etaT: {} ]", self.etai, self.etat)
    }
}

pub struct FresnelNoOp();

impl Fresnel for FresnelNoOp {
    fn evaluate(&self, _cosi: f32) -> Spectrum {
        Spectrum::new(1.0)
    }
}

impl Display for FresnelNoOp {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FresnelNoOp ]")
    }
}

pub struct SpecularReflection<'a> {
    fresnel : &'a Fresnels,
    r       : Spectrum,
    bx_type : u8
}

impl<'a> SpecularReflection<'a> {
    pub fn new(r: &Spectrum, fresnel: &'a Fresnels) -> Self {
        Self {
            r: *r,
            fresnel,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Specular as u8
        }
    }
}

impl<'a> BxDF for SpecularReflection<'a> {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, _wi: &Vector3f) -> Spectrum {
        Spectrum::new(0.0)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, _sample: &Point2f,
                pdf: &mut Float, _sampled_type: &mut u8) -> Spectrum {
        *wi = Vector3f::new(-wo.x, -wo.y, wo.z);
        *pdf = 1.0;

        self.fresnel.evaluate(cos_theta(wi)) * self.r / abs_cos_theta(&*wi)
    }

    fn pdf(&self, _wo: &Vector3f, _wi: &Vector3f) -> Float {
        0.0
    }
}

impl<'a> Display for SpecularReflection<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ SpecularReflection R: {} fresnel: {} ]",
               self.r, self.fresnel)
    }
}

pub struct SpecularTransmission {
    t       : Spectrum,
    etaa    : Float,
    etab    : Float,
    fresnel : FresnelDielectric,
    mode    : TransportMode,
    bx_type : u8
}

impl SpecularTransmission {
    pub fn new(t: &Spectrum, etaa: Float, etab: Float, mode: TransportMode) -> Self {
        Self {
            t: *t,
            etaa, etab, mode,
            fresnel: FresnelDielectric::new(etaa, etab),
            bx_type: BxDFType::Transmission as u8 | BxDFType::Specular as u8
        }
    }
}

impl BxDF for SpecularTransmission {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, _wi: &Vector3f) -> Spectrum {
        Spectrum::new(0.0)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, _sample: &Point2f,
                pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        let (etai, etat) = if cos_theta(wo) > 0.0 {
            (self.etaa, self.etab)
        } else {
            (self.etab, self.etaa)
        };

        // Compute ray direction for specular transmission
        if !refract(wo, &Normal3f::new(0.0, 0.0, 1.0).face_foward_vec(wo), etai/ etat, wi) {
            return Spectrum::new(0.0);
        }

        *pdf = 1.0;
        let mut ft = self.t * (Spectrum::new(1.0) - self.fresnel.evaluate(cos_theta(wi)));

        // Account for non-symmetry with transmission to different medium
        if self.mode == TransportMode::Radiance {
            ft *= (etai * etai) / (etat * etat)
        }

        ft / abs_cos_theta(wi)
    }

    fn pdf(&self, _wo: &Vector3f, _wi: &Vector3f) -> Float {
        0.0
    }
}

impl Display for SpecularTransmission {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ SpecularTransmission: T: {} etaA: {} etaB: {} fresnel: {} mode: {} ]",
                self.t, self.etaa, self.etab, self.fresnel, self.mode)
    }
}

pub struct FresnelSpecular {
    r       : Spectrum,
    t       : Spectrum,
    etaa    : Float,
    etab    : Float,
    mode    : TransportMode,
    bx_type : u8
}

impl FresnelSpecular {
    pub fn new(r: &Spectrum, t: &Spectrum, etaa: Float, etab: Float, mode: TransportMode) -> Self {
        Self {
            r: *r, t: *t, etaa, etab, mode,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Transmission as u8 | BxDFType::Specular as u8
        }
    }
}

impl BxDF for FresnelSpecular {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, _wi: &Vector3f) -> Spectrum {
        Spectrum::new(1.0)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f,
                pdf: &mut f32, sampled_type: &mut u8) -> Spectrum {
        let f = fr_dielectric(cos_theta(wo), self.etaa, self.etab);

        if sample[0] < f {
            // Compute specular reflection for FresnelSpecular

            //Compute perfect specular reflection direction
            *wi = Vector3f::new(-wo.x, -wo.y, wo.z);
            *sampled_type = BxDFType::Specular as u8 | BxDFType::Reflection as u8;
            *pdf = f;

            self.r / abs_cos_theta(wi) * f
        } else {
            let (etai, etat) = if cos_theta(wo) > 0.0 {
                (self.etaa, self.etab)
            } else {
                (self.etab, self.etaa)
            };

            // Compute ray direction for specular transmission
            if !refract(wo, &Normal3f::new(0.0, 0.0, 1.0).face_foward_vec(wo), etai/ etat, wi) {
                return Spectrum::new(0.0);
            }

            let mut ft = self.t * (1.0 - f);

            // Account for non-symmetry with transmission to different medium
            if self.mode == TransportMode::Radiance {
                ft *= (etai * etai) / (etat * etat)
            }

            *sampled_type = BxDFType::Specular as u8 | BxDFType::Transmission as u8;
            *pdf = 1.0 - f;

            ft / abs_cos_theta(wi)
        }
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if same_hemisphere(wo, wi) {
            abs_cos_theta(wi) * INV_PI
        } else {
            0.0
        }
    }
}

impl Display for FresnelSpecular {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FresnelSpecular R: {} T: {} etaA: {} etaB: {} mode: {} ]",
               self.r, self.t, self.etaa, self.etab, self.mode)
    }
}

pub struct LambertianReflection {
    r: Spectrum,
    bx_type: u8
}

impl LambertianReflection {
    pub fn new(r: &Spectrum) -> Self {
        Self {
            r: *r,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Diffuse as u8
        }
    }
}

impl BxDF for LambertianReflection {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, _wi: &Vector3f) -> Spectrum {
        self.r * INV_PI
    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.r
    }

    fn rho2(&self, _nsamples: usize, _samples1: &[Point2f], _samples2: &[Point2f]) -> Spectrum {
        self.r
    }

}

impl Display for LambertianReflection {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ LambertianReflection R: {} ]", self.r)
    }
}

pub struct LambertianTransmission {
    t       : Spectrum,
    bx_type : u8
}

impl LambertianTransmission {
    pub fn new(t: &Spectrum) -> Self {
        Self {
            t: *t,
            bx_type: BxDFType::Transmission as u8 | BxDFType::Diffuse as u8
        }
    }
}

impl BxDF for LambertianTransmission {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, _wo: &Vector3f, _wi: &Vector3f) -> Spectrum {
        self.t * INV_PI
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f, pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        *wi = cosine_sample_hemisphere(sample);

        if wo.z > 0.0 {
            wi.z *= -1.0;
        }

        *pdf = self.pdf(wo, wi);

        self.f(wo, wi)
    }

    fn rho(&self, _wo: &Vector3f, _nsamples: usize, _samples: &[Point2f]) -> Spectrum {
        self.t
    }

    fn rho2(&self, _nsamples: usize, _samples1: &[Point2f], _samples2: &[Point2f]) -> Spectrum {
        self.t
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        if !same_hemisphere(wo, wi) {
            abs_cos_theta(wi)
        } else {
            0.0
        }
    }
}

impl Display for LambertianTransmission {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ LambertianTransmission T: {} ]", self.t)
    }
}

pub struct OrenNayar {
    r: Spectrum,
    A: Float,
    B: Float,
    bx_type: u8
}

impl OrenNayar {
    pub fn new(r: &Spectrum, mut sigma: Float) -> Self {
        sigma = radians(sigma);
        let sigma2 = sigma * sigma;
        let A = 1.0 - (sigma2 / (2.0 * (sigma2 + 0.33)));
        let B = 0.45 * sigma2 / (sigma2 + 0.09);

        Self {
            r: *r,
            A, B,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Diffuse as u8
        }
    }
}

impl BxDF for OrenNayar {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let sin_thetai = sin_theta(wi);
        let sin_thetao = sin_theta(wo);

        // Compute cosine term of Oren_Nayar model
        let mut max_cos = 0.0;

        if sin_thetai > 1e-4 && sin_thetao > 1e-4 {
            let sin_phii = sin_phi(wi);
            let cos_phii = cos_phi(wi);
            let sin_phio = sin_phi(wo);
            let cos_phio = cos_phi(wo);
            let dcos = cos_phii * cos_phio + sin_phii * sin_phio;
            max_cos = (dcos).max(0.0)
        }

        // Compute sine and tangent terms of Oren-Nayar model
        let (sin_alpha, tan_beta) = if abs_cos_theta(wi) > abs_cos_theta(wo) {
            (sin_thetao, sin_thetai / abs_cos_theta(wi))
        } else {
            (sin_thetai, sin_thetao / abs_cos_theta(wo))
        };

        self.r * INV_PI * (self.A + self.B * max_cos * sin_alpha * tan_beta)
    }
}

impl Display for OrenNayar {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ OrenNayar R: {} A: {} B: {} ]",
               self.r, self.A, self.B)
    }
}

pub struct MicrofacetReflection<'a> {
    r           : Spectrum,
    distribution: &'a MicrofacetDistributions,
    fresnel     : &'a Fresnels,
    bx_type     : u8
}

impl<'a> MicrofacetReflection<'a> {
    pub fn new(r: &Spectrum, distribution: &'a MicrofacetDistributions, fresnel: &'a Fresnels) -> Self {
        Self {
            r: *r,
            distribution,
            fresnel,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Glossy as u8
        }
    }
}

impl<'a> BxDF for MicrofacetReflection<'a> {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let cos_thetao = abs_cos_theta(wo);
        let cos_thetai = abs_cos_theta(wi);
        let mut wh = *wi + *wo;

        // Handle degenerate cases for microfacet reflection
        if cos_thetai == 0.0 || cos_thetao == 0.0 { return Spectrum::new(0.0); }
        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }
        wh = wh.normalize();

        // For the Fresnel call, make sure that wh is in the same hemisphere
        // as the surface normal, so that TIR is handled correctly
        let F = self.fresnel.evaluate(wi.dot(&wh.face_foward(&Vector3f::new(0.0, 0.0, 1.0))));

        self.r * self.distribution.d(&wh) * self.distribution.g(&wo, & wi) * F /
            (4.0 * cos_thetai * cos_thetao)
    }

    fn sample_f(&self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f, pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        // Sample microfacet orientation wh and reflected direction wi
        if wo.z == 0.0 { return Spectrum::new(0.0); }
        let wh = self.distribution.sample_wh(wo, sample);

        if wo.dot(&wh) < 0.0 { return Spectrum::new(0.0); }

        *wi = reflect(wo, &wh);

        if !same_hemisphere(wo, &*wi) { return Spectrum::new(0.0); }

        // Compute PDF of wi for microfacet reflection
        *pdf = self.distribution.pdf(wo, &wh) / (4.0 * wo.dot(&wh));

        self.f(wo, &*wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        if !same_hemisphere(wo, wi) { return 0.0; }

        let wh = (*wo + *wi).normalize();

        self.distribution.pdf(wo, &wh) / (4.0 * wo.dot(&wh))
    }
}

impl<'a> Display for MicrofacetReflection<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ MicrofacetReflection R: {} distribution: {} fresnel: {} ]",
               self.r, self.distribution.to_string(), self.fresnel.to_string())
    }
}

pub struct MicrofacetTransmission<'a> {
    t           : Spectrum,
    distribution: &'a MicrofacetDistributions,
    etaa        : Float,
    etab        : Float,
    fresnel     : FresnelDielectric,
    mode        : TransportMode,
    bx_type     : u8
}

impl<'a> MicrofacetTransmission<'a> {
    pub fn new(t: &Spectrum, distribution: &'a MicrofacetDistributions,
               etaa: Float, etab: Float, mode: TransportMode) -> Self {
        Self {
            t: *t,
            etaa, etab, distribution, mode,
            fresnel: FresnelDielectric::new(etaa, etab),
            bx_type: BxDFType::Transmission as u8 | BxDFType::Glossy as u8
        }
    }
}

impl<'a> BxDF for MicrofacetTransmission<'a> {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        if same_hemisphere(wo, wi) { return Spectrum::new(0.0); }

        let cos_thetao = cos_theta(wo);
        let cos_thetai = cos_theta(wi);
        if cos_thetai == 0.0 || cos_thetao == 0.0 { return Spectrum::new(0.0); }

        // Compute wh from wo and wi for microfacet transmission
        let eta = if cos_theta(wo) > 0.0 {
            self.etab / self.etaa
        } else {
            self.etaa / self.etab
        };
        let mut wh = (*wo + *wi * eta).normalize();
        if wh.z < 0.0 { wh = -wh; }

        // Same side?
        if wo.dot(&wh) * wi.dot(&wh) > 0.0 { return Spectrum::new(0.0); }

        let f = self.fresnel.evaluate(wo.dot(&wh));

        let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
        let factor = match self.mode {
            TransportMode::Radiance => 1.0 / eta,
            _                       => 1.0
        };

        (Spectrum::new(1.0) - f) * self.t *
            (self.distribution.d(&wh) * self.distribution.g(&wo, &wi) * eta * eta *
                wi.abs_dot(&wh) * wo.abs_dot(&wh) * factor * factor /
                (cos_thetai * cos_thetao * sqrt_denom * sqrt_denom)).abs()
    }

    fn sample_f(
        &self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f,
        pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        if wo.z == 0.0 { return Spectrum::new(0.0); }

        let wh = self.distribution.sample_wh(wo, sample);

        if wo.dot(&wh) < 0.0 { return Spectrum::new(0.0); }

        let eta = if cos_theta(wo) > 0.0 { self.etaa / self.etab } else { self.etab / self.etaa };

        if !refract(wo, &Normal3f::from(wh), eta, wi) { return Spectrum::new(0.0); }

        *pdf = self.pdf(wo, &*wi);

        self.f(wo, &*wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        if same_hemisphere(wo, wi) { return 0.0; }

        // Compute final wh from wo and wi for microfacet transmission
        let eta = if cos_theta(wo) > 0.0 { self.etaa / self.etab } else { self.etab / self.etaa };
        let wh = (*wo + *wi * eta).normalize();

        if wo.dot(&wh) * wi.dot(&wh) > 0.0 { return 0.0; }

        // Compute change of variables dwh/dwi for microfacet transmission
        let sqrt_denom = wo.dot(&wh) + eta * wi.dot(&wh);
        let dwh_dwi = (eta * eta * wi.dot(&wh)).abs() / (sqrt_denom * sqrt_denom);

        self.distribution.pdf(wo, &wh) * dwh_dwi
    }
}

impl<'a> Display for MicrofacetTransmission<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ MicrofacetTransmission T: {} distribution: {} \
            etaA: {} etaB: {} fresnel: {} mode: {} ]",
            self.t, self.distribution, self.etaa, self.etab, self.fresnel, self.mode
        )
    }
}

pub struct FresnelBlend<'a> {
    rd          : Spectrum,
    rs          : Spectrum,
    distribution: &'a MicrofacetDistributions,
    bx_type     : u8
}

impl<'a> FresnelBlend<'a> {
    pub fn new(rd: &Spectrum, rs: &Spectrum,
               distribution: &'a MicrofacetDistributions) -> Self {
        Self {
            rd: *rd,
            rs: *rs,
            distribution,
            bx_type: BxDFType::Reflection as u8 | BxDFType::Glossy as u8
        }
    }

    pub fn schlick_fresnel(&self, cos_theta: Float) -> Spectrum {
        let pow5 = |v: Float| { (v * v) * (v * v) * v };

        self.rs + (Spectrum::new(1.0) - self.rs) * pow5(1.0 - cos_theta)
    }
}

impl<'a> BxDF for FresnelBlend<'a> {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        let pow5 = |v: Float| { (v * v) * (v * v) * v };
        let diffuse =
             self.rd * (Spectrum::new(1.0) - self.rs) * (28.0 / (23.0 * PI)) *
            (1.0 - pow5(1.0 - 0.5 * abs_cos_theta(wi))) *
            (1.0 - pow5(1.0 - 0.5 * abs_cos_theta(wo)));
        let mut wh = *wi + *wo;

        if wh.x == 0.0 && wh.y == 0.0 && wh.z == 0.0 { return Spectrum::new(0.0); }

        wh = wh.normalize();
        let specular =
            self.schlick_fresnel(wi.dot(&wh)) * (self.distribution.d(&wh) /
            (4.0 * wi.abs_dot(&wh) * abs_cos_theta(wi).max(abs_cos_theta(wo))));

        diffuse + specular

    }

    fn sample_f(
        &self, wo: &Vector3f, wi: &mut Vector3f, sample: &Point2f,
        pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        let mut u = *sample;

        if u[0] < 0.5 {
            u[0] = (2.0 * u[0]).min(ONE_MINUS_EPSILON);
            // Cosine-sample hemisphere, flipping the direction if necessary
            *wi = cosine_sample_hemisphere(&u);

            if wo.z < 0.0 { wi.z *= -1.0; }
        } else {
            u[0] = (2.0 * (u[0] - 0.5)).min(ONE_MINUS_EPSILON);
            //Sample microfacet orientation wh and reflected direction wi
            let wh = self.distribution.sample_wh(wo, &u);
            *wi = reflect(wo, &wh);

            if !same_hemisphere(wo, &*wi) { return Spectrum::new(0.0); }
        }

        *pdf = self.pdf(wo, &*wi);

        self.f(wo, &*wi)
    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> f32 {
        if !same_hemisphere(wo, wi) { return 0.0; }
        let wh = (*wo + *wi).normalize();
        let pdf_wh = self.distribution.pdf(wo, &wh);
        //println!("pdf_wh: {}", pdf_wh);
        0.5 * (abs_cos_theta(wi) * INV_PI + pdf_wh / (4.0 * wo.dot(&wh)))
    }
}

impl<'a> Display for FresnelBlend<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FresnelBlend Rd: {} Rs: {} distribution: {} ]",
               self.rd, self.rs, self.distribution)
    }
}

pub struct FourierBSDF {
    table   : Arc<FourierBSDFTable>,
    mode    : TransportMode,
    bx_type : u8
}

impl FourierBSDF {
    pub fn new(table: Arc<FourierBSDFTable>, mode: TransportMode) -> Self {
        Self {
            table,
            mode,
            bx_type: BxDFType::Reflection as u8 |
                     BxDFType::Transmission as u8 |
                     BxDFType::Glossy as u8
        }
    }
}

impl BxDF for FourierBSDF {
    matches_flags_bxdf!();

    get_type!();

    fn f(&self, wo: &Vector3f, wi: &Vector3f) -> Spectrum {
        // Find the zenith angle cosines and azimuth difference angle
        let mui = cos_theta(&-(*wi));
        let muo = cos_theta(wo);
        let cos_phi = cos_d_phi(&-(*wi), wo) as f64;

        // Compute Fourier coefficients ak for mui, muo

        // Determine offsets and weights for mui and muo
        let mut offseti = 0;
        let mut offeto = 0;
        let mut weightsi = [0.0 as Float; 4];
        let mut weightso = [0.0 as Float; 4];

        if !self.table.get_weights_and_offset(mui, &mut offseti, &mut weightsi) ||
           !self.table.get_weights_and_offset(muo, &mut offeto, &mut weightso) {
            return Spectrum::new(0.0);
        }

        // Allocate storage to accumulate ak coefficients
        let mut ak: SmallVec<[Float; 256]> = smallvec![0.0; (self.table.nmax * self.table.nchannels) as usize];

        // Accumulate weighted sums of nearby ak coefficients
        let mut nmax = 0;

        for (b, weo) in weightso.iter().enumerate() {
            for (a, wei) in weightsi.iter().enumerate() {
                // Add contribution of (a, b) to ak values
                let weight = wei * weo;

                if weight != 0.0 {
                    let mut m = 0;
                    let offset = self.table.get_ak(offseti + a as i32, offeto + b as i32, &mut m);
                    nmax = std::cmp::max(nmax, m);

                    for c in 0..self.table.nchannels as usize {
                        for k in 0..m as usize {
                            ak[c * self.table.nmax as usize + k] +=
                                weight * self.table.a[offset as usize + c * m as usize + k];
                        }
                    }
                }
            }
        }

        // Evaluete Fourier expansion for angle phi
        let Y = (fourier(&ak, 0, nmax, cos_phi)).max(0.0);
        let mut scale = if mui != 0.0 {
            1.0 / mui.abs()
        } else {
            0.0
        };

        // Update scale to account for adjoint light transport
        if self.mode == TransportMode::Radiance && mui * muo > 0.0 {
            let eta = if mui > 0.0 {
                1.0 / self.table.eta
            } else {
                self.table.eta
            };
            scale *= eta * eta;
        }

        if self.table.nchannels == 1 {
            return Spectrum::new(Y * scale);
        }
        // Compute and return RGB colors for tabulated BSDF
        let R = fourier(&ak, self.table.nmax as usize, nmax, cos_phi);
        let B = fourier(&ak, 2 * self.table.nmax as usize, nmax, cos_phi);
        let G = 1.39829 * Y - 0.100913 * B - 0.297375 * R;
        let rgb = [R * scale, G * scale, B * scale];

        Spectrum::from_rgb(rgb, SpectrumType::Reflectance).clamps(0.0 ,INFINITY)
    }

    fn sample_f(
        &self, wo: &Vector3f, wi: &mut Vector3f, u: &Point2f,
        pdf: &mut f32, _sampled_type: &mut u8) -> Spectrum {
        // Sample zenith angle component for FourierBSDF
        let muo = cos_theta(wo);
        let mut pdf_mu = 0.0;
        let table = &self.table;
        let mui = sample_catmull_rom_2d(
            table.nmu, table.nmu, &table.mu,
            &table.mu, &table.a0, &table.cdf,
            muo, u[1], None, Some(&mut pdf_mu));
        // Compute Fourier coefficients ak for (mui, muo)

        // Determine offsets and weights for mui and muo
        let (mut offseti, mut offseto) = (0, 0);
        let (mut weightsi, mut weightso) = ([0.0; 4], [0.0; 4]);

        if !table.get_weights_and_offset(mui, &mut offseti, &mut weightsi) ||
           !table.get_weights_and_offset(muo, &mut offseto, &mut weightso) {
            return Spectrum::new(0.0);
        }


        // Allocate storage to acculate ak coefficients
        let mut ak: SmallVec<[Float; 256]> = smallvec![0.0; (table.nmax * table.nchannels) as usize];

        // Accumulate weight sums of nearby ak coefficients
        let mut mmax = 0;

        for b in 0..4 {
            for a in 0..4 {
                // Add contribution of (a, b) to ak values
                let weight = weightsi[a] * weightso[b];

                if weight != 0.0 {
                    let mut m = 0;
                    let ap = table.get_ak(offseti + a as i32, offseto + b as i32, &mut m);
                    mmax = std::cmp::max(mmax, m);

                    for c in 0.. table.nchannels {
                        for k in 0..m {
                            ak[(c * table.nmax + k) as usize] += weight * table.a[(ap + c * m + k) as usize];
                        }
                    }
                }
            }
        }

        // Importance sample the luminance Fourier expansion
        let mut phi = 0.0;
        let mut pdf_phi = 0.0;
        let Y = sample_fourier(&ak, &table.recip, mmax, u[0], &mut pdf_phi, &mut phi);
        *pdf = (pdf_phi * pdf_mu).max(0.0);

        // Compute the scattered direction for FourierBSDF
        let sin2_thetai = (1.0 - mui * mui).max(0.0);
        let mut norm = (sin2_thetai / sin2_theta(wo)).sqrt();

        if norm.is_infinite() { norm = 0.0; }

        let sin_phi = phi.sin();
        let cos_phi = phi.cos();
        *wi = -Vector3f::new(
            norm * (cos_phi * wo.x - sin_phi * wo.y),
            norm * (sin_phi * wo.x + cos_phi * wo.y),
            mui
        );

        // Mathematically, wi will be normalized (if wo was). However, in
        // practice, floating-point rounding error can cause some error to
        // accumulate in the computed value of wi here. This can be
        // catastrophic: if the ray intersects an object with the FourierBSDF
        // again and the wo (based on such a wi) is nearly perpendicular to the
        // surface, then the wi computed at the next intersection can end up
        // being substantially (like 4x) longer than normalized, which leads to
        // all sorts of errors, including negative spectral values. Therefore,
        // we normalize again here.
        *wi = wi.normalize();

        // Evaluate remaining Fourier expansions for angle phi
        let mut scale = if mui != 0.0 { 1.0 / mui.abs() } else { 0.0 };

        if self.mode == TransportMode::Radiance && mui * muo > 0.0 {
            let eta = if mui > 0.0 { 1.0 / table.eta } else { table.eta };
            scale *= eta * eta;
        }

        if table .nchannels == 1 { return Spectrum::new(Y * scale); }

        let R = fourier(&ak, table.nmax as usize, mmax, cos_phi as f64);
        let B = fourier(&ak, (2 * table.nmax) as usize, mmax, cos_phi as f64);
        let G = 1.39829 * Y - 0.100913 * B - 0.297375 * R;
        let rgb = [R * scale, G * scale, B * scale];

        Spectrum::from_rgb(rgb, SpectrumType::Reflectance).clamp(0.0, INFINITY)

    }

    fn pdf(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        // Find the zenith angle cosines and azimuth difference angle
        let muo = cos_theta(wo);
        let mui = cos_theta(&-(*wi));
        let cos_phi = cos_d_phi(&(-*wi), wo);
        let table = &self.table;

        // Compute luminance Fourier coefficients ak for (mui, muo)
        let mut offseti = 0;
        let mut offseto = 0;
        let mut weightsi = [0.0; 4];
        let mut weightso = [0.0; 4];


        if !table.get_weights_and_offset(mui, &mut offseti, &mut weightsi) ||
            !table.get_weights_and_offset(muo, &mut offseto, &mut weightso) {
            return 0.0;
        }

        let mut ak: SmallVec<[Float; 256]> = smallvec![0.0; table.nmax as usize];
        let mut mmax = 0;

        for o in 0..4 {
            for i in 0..4 {
                let weight = weightsi[i] * weightso[o];

                if weight == 0.0 { continue; }

                let mut order = 0;
                let coeffs = table.get_ak(offseti + i as i32, offseto + o as i32, &mut order);
                mmax = std::cmp::max(mmax, order);

                for k in 0..order as usize {
                    ak[k] += table.a[coeffs as usize + k] * weight;
                }

            }
        }

        // Evaluate probability of sampling wi
        let mut rho = 0.0;

        for o in 0..4 {
            if weightso[o] == 0.0 { continue; }

            rho += weightso[o] * table.cdf[((offseto + o as i32) * table.nmu + table.nmu - 1) as usize] * (2.0 * PI);
        }



        let Y = fourier(&ak, 0, mmax, cos_phi as f64);

        if rho > 0.0 && Y > 0.0 { Y / rho } else { 0.0 }
    }




}

impl<'a> Display for FourierBSDF {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ FourierBSDF eta: {} nMax: {} nChannels: {} nMu: {} mode: {} ]",
               self.table.eta, self.table.nmax, self.table.nchannels,
               self.table.nmu, self.mode)
    }
}

#[derive(Clone)]
pub struct BSDF<'a>{
    pub eta     : Float,
    ns          : Normal3f,
    ng          : Normal3f,
    ss          : Vector3f,
    ts          : Vector3f,
    n_bxdfs     : usize,
    pub bxdfs   : Vec<&'a BxDFs<'a>>,
}

impl<'a> BSDF<'a> {
    pub fn new(si: &SurfaceInteraction, eta: Float) -> Self {
        let ns = si.shading.n;
        let ss = si.shading.dpdu.normalize();

        Self {
            eta, ns, ss,
            ng: si.n,
            ts: ns.cross_vec(&ss),
            n_bxdfs: 0,
            bxdfs: Vec::with_capacity(MAX_BXDFS)
        }
    }

    pub fn add(&mut self, b: &'a BxDFs<'a>) {
        assert!(self.n_bxdfs < MAX_BXDFS);
        self.bxdfs.push(b);
        self.n_bxdfs += 1;
    }

    #[inline(always)]
    pub fn num_components(&self, flags: u8) -> usize {
        self.bxdfs.iter().filter(|b| b.matches_flags(flags)).count()
    }

    pub fn world_to_local(&self, v: &Vector3f) -> Vector3f {
        Vector3f::new(
            v.dot(&self.ss),
            v.dot(&self.ts),
            v.dot_norm(&self.ns)
        )
    }

    pub fn local_to_world(&self, v: &Vector3f) -> Vector3f {
        Vector3f::new(
            self.ss.x * v.x + self.ts.x * v.y + self.ns.x * v.z,
            self.ss.y * v.x + self.ts.y * v.y + self.ns.y * v.z,
            self.ss.z * v.x + self.ts.z * v.y + self.ns.z * v.z
        )
    }

    pub fn f(&self, wow: &Vector3f, wiw: &Vector3f, f: u8) -> Spectrum {
        // TODO: ProfilePhase
        let wi = self.world_to_local(wiw);
        let wo = self.world_to_local(wow);
        if wo.z == 0.0 { return Spectrum::new(0.0); }

        let reflect = wiw.dot_norm(&self.ng) * wow.dot_norm(&self.ng) > 0.0;
        let mut res = Spectrum::new(0.0);

        for i in 0..self.n_bxdfs {
            let b = self.bxdfs[i];

            if b.matches_flags(f) &&
                ((reflect && is_reflection(b.get_type())) ||
                (!reflect && is_transmission(b.get_type()))) {
                res += b.f(&wo, &wi);
            }
        }

        res
    }

    pub fn sample_f(
        &self, wow: &Vector3f, wiw: &mut Vector3f, u: &Point2f,
        pdf: &mut Float, ty: u8, sampled_type: &mut u8) -> Spectrum {
        // TODO: ProfilePhase
        // Choose which BXDF to sample
        let matchingcomps = self.num_components(ty);

        if matchingcomps == 0 {
            *pdf = 0.0;
            *sampled_type = 0u8;

            return Spectrum::new(0.0);
        }

        let comp = std::cmp::min((u[0] * matchingcomps as Float).floor() as usize, matchingcomps - 1);

        // Get BxDF pointer for chosen compoent
        let mut bxdf: Option<&BxDFs> = None;
        let mut count = comp as isize;
        let mut idx = 0;

        for i in 0..self.n_bxdfs {
            let matches = self.bxdfs[i].matches_flags(ty);
            if matches && count == 0 {
                bxdf = Some(self.bxdfs[i]);
                // store index for later
                idx = i;
                count -= 1;
                break
            } else if matches {
                count -= 1;
            }
        }

        assert!(bxdf.is_some());
        let b = bxdf.unwrap();
        debug!("BSDF::sample_f chose comp = {} / matching = {}, bxdf: {}",
              comp, matchingcomps, b);

        // Remap BxDF sample u to [0, 1)^2
        let uremapped = Point2f::new(
            (u[0] * matchingcomps as Float - comp as Float).min(ONE_MINUS_EPSILON),
            u[1]);

        // Sample chosen BxDF
        let wo = self.world_to_local(wow);
        let mut wi = Vector3f::default();
        if wo.z == 0.0 { return Spectrum::new(0.0); }
        *pdf = 0.0;

        *sampled_type = b.get_type();


        let mut f = b.sample_f(&wo, &mut wi, &uremapped, pdf, sampled_type);

        debug!(
            "For wo = {}, sampled f = {}, pdf = {}, ratio = {}, wi = {}" ,
            wo, f, *pdf, (if *pdf > 0.0 { f / *pdf } else { Spectrum::new(0.0) }), wi);

        if *pdf == 0.0 {
            *sampled_type = 0;
            return Spectrum::new(0.0);
        }

        *wiw = self.local_to_world(&wi);

        // Compute overall PDF with all matching BxDFs
        if (b.get_type() & BxDFType::Specular as u8 == 0) && matchingcomps > 1 {
            for i in 0..self.n_bxdfs {
                if idx != i && self.bxdfs[i].matches_flags(ty) {
                    *pdf += self.bxdfs[i].pdf(&wo, &wi);
                }
            }
        }

        if matchingcomps > 1 { *pdf /= matchingcomps as Float; }

        // Compute value of BSDF for sampled direction
        if b.get_type() & BxDFType::Specular as u8 == 0 {
            let reflect = wiw.dot_norm(&self.ng) * wow.dot_norm(&self.ng) > 0.0;
            f = Spectrum::new(0.0);

            for i in 0..self.n_bxdfs {
                if self.bxdfs[i].matches_flags(ty) &&
                    ((reflect && (self.bxdfs[i].get_type() & BxDFType::Reflection as u8 != 0)) ||
                     (!reflect && (self.bxdfs[i].get_type() & BxDFType::Transmission as u8 != 0))) {
                    f += self.bxdfs[i].f(&wo, &wi);
                }
            }
        }

        debug!(
            "Overall f = {}, pdf = {}, ratio = {}",
            f, *pdf, (if *pdf > 0.0 { f / *pdf } else { Spectrum::new(0.0) }));

        f
    }

    pub fn pdf(&self, wow: &Vector3f, wiw: &Vector3f, flags: u8) -> Float {
        // TODO: ProfilePhase
        if self.n_bxdfs == 0 { return 0.0; }

        let wo = self.world_to_local(wow);
        let wi = self.world_to_local(wiw);
        if wo.z == 0.0 { return 0.0; }
        let mut pdf = 0.0;
        let mut matchingcomps = 0;

        for i in 0..self.n_bxdfs {
            if self.bxdfs[i].matches_flags(flags) {
                matchingcomps += 1;
                pdf += self.bxdfs[i].pdf(&wo, &wi);
            }
        }

        if matchingcomps > 0 {
            pdf / matchingcomps as Float
        } else {
            0.0
        }
    }

    pub fn rho(&self, wo_world: &Vector3f, nsamples: usize,
               samples: &[Point2f], f: u8) -> Spectrum {
        let wo = self.world_to_local(wo_world);

        self.bxdfs.iter()
            .filter(|b| b.matches_flags(f))
            .map(|b| b.rho(&wo, nsamples, samples))
            .fold(Spectrum::new(0.0), |b1, b2| b1 + b2 )
    }

    pub fn rho2(&self, nsamples: usize, samples1: &[Point2f],
                samples2: &[Point2f], f: u8) -> Spectrum {
        self.bxdfs.iter()
            .filter(|b| b.matches_flags(f))
            .map(|b| b.rho2(nsamples, samples1, samples2))
            .fold(Spectrum::new(0.0), |b1, b2| b1 + b2 )
    }
}

#[inline(always)]
fn is_reflection(t: u8) -> bool {
    (t & BxDFType::Reflection as u8) != 0
}

#[inline(always)]
fn is_transmission(t: u8) -> bool {
    (t & BxDFType::Transmission as u8) != 0
}