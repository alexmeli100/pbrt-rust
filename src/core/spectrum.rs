use crate::core::pbrt::{Float, lerp, find_interval, inverse_gamma_correct};
use lazy_static::lazy_static;
use num::{Zero, One};
use crate::core::cie::*;
use pbrt_macros::{coefficient_spectrum};

pub type Spectrum = RGBSpectrum;

lazy_static! {
    static ref X: SampledSpectrum = SampledSpectrum::init_spectrum_xyz(&CIE_LAMBDA, &CIE_X);
    static ref Y: SampledSpectrum = SampledSpectrum::init_spectrum_xyz(&CIE_LAMBDA, &CIE_Y);
    static ref Z: SampledSpectrum = SampledSpectrum::init_spectrum_xyz(&CIE_LAMBDA, &CIE_Z);

    static ref RGB_REFL2_SPEC_WHITE: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_WHITE);
    static ref RGB_REFL2_SPEC_CYAN: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_CYAN);
    static ref RGB_REFL2_SPEC_MAGENTA: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_MAGENTA);
    static ref RGB_REFL2_SPEC_YELLOW: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_YELLOW);
    static ref RGB_REFL2_SPEC_RED: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_RED);
    static ref RGB_REFL2_SPEC_GREEN: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_GREEN);
    static ref RGB_REFL2_SPEC_BLUE: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_REFL2_SPECT_BLUE);

    static ref RGB_ILLUM2_SPEC_WHITE: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_WHITE);
    static ref RGB_ILLUM2_SPEC_CYAN: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_CYAN);
    static ref RGB_ILLUM2_SPEC_MAGENTA: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_MAGENTA);
    static ref RGB_ILLUM2_SPEC_YELLOW: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_YELLOW);
    static ref RGB_ILLUM2_SPEC_RED: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_RED);
    static ref RGB_ILLUM2_SPEC_GREEN: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_GREEN);
    static ref RGB_ILLUM2_SPEC_BLUE: SampledSpectrum = SampledSpectrum::init_spectrum(&RGB2_SPECT_LAMBDA, &RGB_ILLUM2_SPECT_BLUE);
}

trait CoefficientSpectrum {
    fn c(&self) -> &[Float];
}


pub fn black_body(lambda: &[Float], n: usize, t: Float, le: &mut [Float]) {
    if t <= 0.0 {
        for i in 0..n {
            le[i] = 0.0;
        }

        return;
    }

    let c = 299792458.0;
    let h = 6.62606957e-34;
    let kb = 1.3806488e-23;

    for i in 0..n {
        // compute emitted radiance for blackbody at wavelength lambda[i]
        let l = (lambda[i] as f64 * 1.0e-9 as f64) as Float;
        let lambda5 = (l * l) * (l * l) * l;
        let e = ((h * c) / (l * kb * t)).exp();
        le[i] = ( 2.0 * h * c * c) / (lambda5 * (e - 1.0));

        assert!(!le[i].is_nan())
    }
}

pub fn black_body_normalized(lambda: &[Float], n: usize, t: Float, le: &mut [Float]) {
    black_body(lambda, n, t, le);
    // Normalize le values based on maximum blackbody radiance
    let lambda_max = 2.8977721e-3 / t * 1.0e9;
    let mut maxl = [0.0; 1];
    black_body(&[lambda_max], 1, t, &mut maxl);
    
    for i in 0..n {
        le[i] /= maxl[0];
    }
}

#[derive(Copy, Clone, Eq, PartialEq)]
pub enum SpectrumType {
    Reflectance,
    Illuminant
}

#[coefficient_spectrum]
#[derive(Debug, Copy, Clone)]
pub struct RGBSpectrum {
    c: [Float; 3],
    n_spectral_samples: usize
}

impl RGBSpectrum {
    pub fn new(v: Float) -> Self {
        Self {
            c: [v; 3],
            n_spectral_samples: 3
        }
    }

    pub fn rgb(r: Float, g: Float, b: Float) -> Self {
        Self { c: [r, g, b], n_spectral_samples: 3 }
    }

    pub fn from_rgb(rgb: [Float; 3], _: SpectrumType) -> Self {
        Self {
            c: rgb,
            n_spectral_samples: 3
        }
    }

    pub fn from_rgb_spectrum(r: &RGBSpectrum, _stype: SpectrumType) -> Self {
        *r
    }

    pub fn from_xyz(xyz: [Float; 3], _: SpectrumType) -> Self {
        Self {
            c: xyz_to_rgb(xyz),
            n_spectral_samples: 3
        }
    }

    pub fn to_rgb(&self) -> [Float; 3] {
        self.c
    }

    pub fn to_xyz(&self) -> [Float; 3] {
        rgb_to_xyz(self.c)
    }

    pub fn y(&self) -> Float {
        let y_weight = [0.212671, 0.715160, 0.072169];

        y_weight[0] * self.c[0] + y_weight[1] * self.c[1] + y_weight[2] * self.c[2]
    }

    pub fn from_sampled(lambda: &[Float], v: &[Float], n: usize) -> Self {
        // Sort samples if unordered, use sorted for returned spectrum
        if !spectrum_sample_sorted(lambda, v, n) {
            let mut slambda: Vec<_> = lambda.iter().copied().collect();
            let mut sv: Vec<_> = v.iter().copied().collect();
            sort_spectrum_samples(&mut slambda, &mut sv, n);
            return Self::from_sampled(&slambda[..], &v[..], n);
        }

        let mut xyz = [0.0, 0.0, 0.0];

        for i in 0..N_CIE_SAMPLES {
            let val = interpolate_spectrum_samples(lambda, v, n, CIE_LAMBDA[i]);
            xyz[0] += val * CIE_X[i];
            xyz[1] += val * CIE_Y[i];
            xyz[2] += val * CIE_Z[i];
        }

        let scale =
            (CIE_LAMBDA[N_CIE_SAMPLES - 1] as Float - CIE_LAMBDA[0]) /
            (CIE_Y_INTEGRAL * N_CIE_SAMPLES as Float);
        xyz[0] *= scale;
        xyz[1] *= scale;
        xyz[2] *= scale;

        Self::from_xyz(xyz, SpectrumType::Reflectance)
    }

    pub fn inverse_gamma_correct(&self) -> Self {
        Self::rgb(
            inverse_gamma_correct(self.c[0]),
            inverse_gamma_correct(self.c[1]),
            inverse_gamma_correct(self.c[2]),
        )
    }

    pub const fn n() -> usize {
        3
    }
}

impl Default for RGBSpectrum {
    fn default() -> Self {
        Self {
            c: [0 as Float; 3],
            n_spectral_samples: 3
        }
    }
}

impl Zero for RGBSpectrum {
    fn zero() -> Self {
        RGBSpectrum::default()
    }

    fn is_zero(&self) -> bool {
        self.is_black()
    }
}

impl One for RGBSpectrum {
    fn one() -> Self {
        Self {
            c: [1.0; 3],
            n_spectral_samples: 3
        }
    }
}

impl From<SampledSpectrum> for RGBSpectrum {
    fn from(s: SampledSpectrum) -> Self {
        let rgb = s.to_rgb();

        RGBSpectrum::from_rgb(rgb, SpectrumType::Reflectance)
    }
}

impl From<RGBSpectrum> for Float {
    fn from(r: RGBSpectrum) -> Self {
        r.y()
    }
}

impl From<SampledSpectrum> for Float {
    fn from(s: SampledSpectrum) -> Self {
        s.y()
    }
}

impl From<Float> for RGBSpectrum {
    fn from(v: f32) -> Self {
        RGBSpectrum::new(v)
    }
}

impl From<RGBSpectrum> for SampledSpectrum {
    fn from(r: RGBSpectrum) -> Self {
        let rgb = r.to_rgb();

        SampledSpectrum::from_rgb(rgb, SpectrumType::Reflectance)
    }
}

#[coefficient_spectrum]
#[derive(Debug, Copy, Clone)]
pub struct SampledSpectrum {
    c: [Float; N_SPECTRAL_SAMPLES],
    n_spectral_samples: usize
}

impl Default for SampledSpectrum {
    fn default() -> Self {
        Self {
            c: [0 as Float; N_SPECTRAL_SAMPLES],
            n_spectral_samples: N_SPECTRAL_SAMPLES
        }
    }
}

impl From<Float> for SampledSpectrum {
    fn from(v: Float) -> Self {
        SampledSpectrum::new(v)
    }
}

impl SampledSpectrum {
    pub fn new(v: Float) -> Self {
        Self {
            c: [v; N_SPECTRAL_SAMPLES],
            n_spectral_samples: N_SPECTRAL_SAMPLES
        }
    }

    pub fn from_rgb_spectrum(r: &RGBSpectrum, stype: SpectrumType) -> Self {
        let rgb = r.to_rgb();

        SampledSpectrum::from_rgb(rgb, stype)
    }

    pub fn from_sampled(lambda: &[Float], v: &[Float], n: usize) -> Self {
        // Sort samples if unordered, use sorted for returned spectrum
        if !spectrum_sample_sorted(lambda, v, n) {
            let mut slambda: Vec<_> = lambda.iter().copied().collect();
            let mut sv: Vec<_> = v.iter().copied().collect();
            sort_spectrum_samples(&mut slambda, &mut sv, n);
            return SampledSpectrum::from_sampled(&slambda[..], &v[..], n);
        }

        let mut r = SampledSpectrum::default();

        for i in 0..N_SPECTRAL_SAMPLES {
            let lambda0 = lerp(
                i as Float / N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START as Float,
                SAMPLED_LAMBDA_END as Float
            );
            let lamdba1 = lerp(
                (i + 1) as Float/ N_SPECTRAL_SAMPLES as Float,
                SAMPLED_LAMBDA_START as Float,
                SAMPLED_LAMBDA_END as Float
            );
            r.c[i] = average_spectrum_samples(lambda, v, n, lambda0, lamdba1);
        }

        r
    }

    pub fn to_xyz(&self) -> [Float; 3] {
        let mut xyz = [0 as Float; 3];

        for i in 0..N_SPECTRAL_SAMPLES {
            xyz[0] += X.c[i] * self.c[i];
            xyz[1] += Y.c[i] * self.c[i];
            xyz[2] += Z.c[i] * self.c[i];
        }

        let scale = (SAMPLED_LAMBDA_END - SAMPLED_LAMBDA_START) as Float / (CIE_Y_INTEGRAL * N_SPECTRAL_SAMPLES as Float);
        xyz[0] *= scale;
        xyz[1] *= scale;
        xyz[2] *= scale;

        xyz
    }

    pub fn y(&self) -> Float {
        let mut yy = 0 as Float;

        for i in 0..N_SPECTRAL_SAMPLES {
            yy += Y.c[i] * self.c[i];
        }

        let scale  = (SAMPLED_LAMBDA_END - SAMPLED_LAMBDA_START) as Float / (CIE_Y_INTEGRAL * N_SPECTRAL_SAMPLES as Float);

        yy * scale
    }

    pub fn to_rgb(&self) -> [Float; 3] {
        let xyz = self.to_xyz();

        xyz_to_rgb(xyz)
    }

    pub fn from_rgb(rgb: [Float; 3], stype: SpectrumType) -> Self {
        let mut r = SampledSpectrum::default();

        match stype {
            SpectrumType::Reflectance => reflectance_to_rgb(rgb, &mut r),
            _                         => illuminant_to_rgb(rgb, &mut r)
        }

        r.clamps(0.0, Float::INFINITY)
    }

    pub fn from_xyz(xyz: [Float; 3], stype: SpectrumType) -> Self {
        let rgb = xyz_to_rgb(xyz);

        Self::from_rgb(rgb, stype)
    }

    pub fn init() {
        lazy_static::initialize(&X);
        lazy_static::initialize(&Y);
        lazy_static::initialize(&Z);
        lazy_static::initialize(&RGB_REFL2_SPEC_WHITE);
        lazy_static::initialize(&RGB_REFL2_SPEC_CYAN);
        lazy_static::initialize(&RGB_REFL2_SPEC_MAGENTA);
        lazy_static::initialize(&RGB_REFL2_SPEC_YELLOW);
        lazy_static::initialize(&RGB_REFL2_SPEC_RED);
        lazy_static::initialize(&RGB_REFL2_SPEC_GREEN);
        lazy_static::initialize(&RGB_REFL2_SPEC_BLUE);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_WHITE);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_CYAN);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_MAGENTA);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_YELLOW);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_RED);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_GREEN);
        lazy_static::initialize(&RGB_ILLUM2_SPEC_BLUE);

    }

    fn init_spectrum_xyz(lambda: &[Float], vals: &[Float]) -> Self {
        let mut s = SampledSpectrum::default();

        for i in 0..N_SPECTRAL_SAMPLES {

            let wl0 = lerp(i as Float / N_SPECTRAL_SAMPLES as Float, SAMPLED_LAMBDA_START as Float, SAMPLED_LAMBDA_END as Float);
            let wl1 = lerp((i + 1) as Float / N_SPECTRAL_SAMPLES as Float, SAMPLED_LAMBDA_START as Float, SAMPLED_LAMBDA_END as Float);

            s.c[i] = average_spectrum_samples(lambda, vals, N_CIE_SAMPLES, wl0, wl1);
        }

        s
    }

    fn init_spectrum(lambda: &[Float], vals: &[Float]) -> Self {
        let mut s = SampledSpectrum::default();

        for i in 0..N_SPECTRAL_SAMPLES {

            let wl0 = lerp(i as Float / N_SPECTRAL_SAMPLES as Float, SAMPLED_LAMBDA_START as Float, SAMPLED_LAMBDA_END as Float);
            let wl1 = lerp((i + 1) as Float / N_SPECTRAL_SAMPLES as Float, SAMPLED_LAMBDA_START as Float, SAMPLED_LAMBDA_END as Float);

            s.c[i] = average_spectrum_samples(lambda, vals, N_RGB2_SPECT_SAMPLES, wl0, wl1);
        }

        s
    }

    pub const fn n() -> usize {
        N_SPECTRAL_SAMPLES
    }
}

impl Zero for SampledSpectrum {
    fn zero() -> Self {
        SampledSpectrum::default()
    }

    fn is_zero(&self) -> bool {
        self.is_black()
    }
}

fn spectrum_sample_sorted(lambda: &[Float], _: &[Float], n: usize) -> bool {
    for i in 0..n-1 {
        if lambda[i] > lambda[i+1] {
            return false;
        }
    }

    true
}

fn sort_spectrum_samples(lambda: &mut [Float], vals: &mut [Float], n: usize) {
    let mut sort_vec: Vec<_> = lambda[0..n].iter().zip(vals[0..n].iter()).map(|(l, v)| (*l, *v)).collect();

    sort_vec.sort_by(|a, b| a.partial_cmp(b).unwrap());

    sort_vec
        .iter()
        .enumerate()
        .for_each(|(i, (f, s))| {
            lambda[i] = *f;
            vals[i] = *s;
        });
}

fn average_spectrum_samples(lambda: &[Float], vals: &[Float], n: usize, lambda_start: Float, lambda_end: Float) -> Float {
    for i in 0..(n - 1) { assert!(lambda[i + 1] > lambda[i]); }
    assert!(lambda_start < lambda_end);
    // Handle cases with out-of-bounds range or single sample only
    if lambda_end <= lambda[0] { return vals[0]; }
    if lambda_start >= lambda[n - 1] { return vals[n - 1]; }
    if n == 1 { return vals[0]; }

    let mut sum = 0 as Float;

    // Add contributions of constant segments before/after samples
    if lambda_start < lambda[0] { sum += vals[0] * (lambda[0] - lambda_start); }
    if lambda_end > lambda[n - 1] { sum += vals[n - 1] * (lambda_end - lambda[n - 1]); }

    // Advance to first relevant wavelength segment
    let mut i = 0;
    while lambda_start > lambda[i + 1] { i += 1; }
    assert!(i + 1 < n);

    let interp = |w: Float, i: usize| -> Float {
        lerp((w - lambda[i]) / (lambda[i + 1] - lambda[i]), vals[i], vals[i + 1])
    };

    while i + 1 < n && lambda_end >= lambda[i] {
        let seg_lambda_start = lambda_start.max(lambda[i]);
        let seg_lambda_end = lambda_end.min( lambda[i + 1]);
        sum += 0.5 * (interp(seg_lambda_start, i) + interp(seg_lambda_end, i)) * (seg_lambda_end - seg_lambda_start);
        i += 1;
    }

    sum / (lambda_end - lambda_start)
}

fn interpolate_spectrum_samples(lambda: &[Float], vals: &[Float], n: usize, l: Float) -> Float {
    for i in 0..n-1 {
        assert!(lambda[i + 1] > lambda[i]);
    }

    if l <= lambda[0] { return vals[0] }
    if l >= lambda[n - 1] { return vals[n - 1] }

    let offset = find_interval(n as i32, |i| lambda[i as usize] as Float <= l) as usize;
    assert!(l >= lambda[offset] && l <= lambda[offset + 1]);
    let t = (l - lambda[offset]) / (lambda[offset + 1] - lambda[offset]);

    lerp(t, vals[offset], vals[offset + 1])
}

pub fn xyz_to_rgb(xyz: [Float; 3]) -> [Float; 3] {
    let mut rgb = [0 as Float; 3];

    rgb[0] = 3.240479 * xyz[0] - 1.537150 * xyz[1] - 0.498535 * xyz[2];
    rgb[1] = -0.969256 * xyz[0] + 1.875991 * xyz[1] + 0.041556 * xyz[2];
    rgb[2] = 0.055648 * xyz[0] - 0.204043 * xyz[1] + 1.057311 * xyz[2];

    rgb
}

pub fn rgb_to_xyz(rgb: [Float; 3]) -> [Float; 3] {
    let mut xyz = [0 as Float; 3];

    xyz[0] = 0.412453 * rgb[0] + 0.357580 * rgb[1] + 0.180423 * rgb[2];
    xyz[1] = 0.212671 * rgb[0] + 0.715160 * rgb[1] + 0.072169 * rgb[2];
    xyz[2] = 0.019334 * rgb[0] + 0.119193 * rgb[1] + 0.950227 * rgb[2];

    xyz
}

fn reflectance_to_rgb(rgb: [Float; 3], r: &mut SampledSpectrum) {
    if rgb[0] <= rgb[1] && rgb[0] <= rgb[2] {
        // compute reflectance SampledSpectrum with rgb[0] as miniumum
        *r += *RGB_REFL2_SPEC_WHITE * rgb[0];

        if rgb[1] <= rgb[2] {
            *r += *RGB_REFL2_SPEC_CYAN * (rgb[1] - rgb[0]);
            *r += *RGB_REFL2_SPEC_BLUE * (rgb[2] - rgb[1]);
        } else {
            *r += *RGB_REFL2_SPEC_CYAN * (rgb[2] - rgb[0]);
            *r += *RGB_REFL2_SPEC_GREEN * (rgb[1] - rgb[2]);
        }
    } else if rgb[1] <= rgb[0] && rgb[1] <= rgb[2] {
        // compute reflectance SampledSpectrum with rgb[1] as miniumum
        *r += *RGB_REFL2_SPEC_WHITE * rgb[1];

        if rgb[0] <= rgb[2] {
            *r += *RGB_REFL2_SPEC_MAGENTA * (rgb[0] - rgb[1]);
            *r += *RGB_REFL2_SPEC_BLUE * (rgb[2] - rgb[0]);
        } else {
            *r += *RGB_REFL2_SPEC_MAGENTA * (rgb[2] - rgb[1]);
            *r += *RGB_REFL2_SPEC_RED * (rgb[0] - rgb[2]);
        }
    } else {
        // compute reflectance SampledSpectrum with rgb[0] as miniumum
        *r += *RGB_REFL2_SPEC_WHITE * rgb[2];

        if rgb[0] <= rgb[1] {
            *r += *RGB_REFL2_SPEC_YELLOW * (rgb[0] - rgb[2]);
            *r += *RGB_REFL2_SPEC_GREEN * (rgb[1] - rgb[0]);
        } else {
            *r += *RGB_REFL2_SPEC_YELLOW * (rgb[1] - rgb[2]);
            *r += *RGB_REFL2_SPEC_RED * (rgb[0] - rgb[1]);
        }
    }

    *r *= 0.94;
}

fn illuminant_to_rgb(rgb: [Float; 3], r: &mut SampledSpectrum) {
    if rgb[0] <= rgb[1] && rgb[0] <= rgb[2] {
        // compute illuminant SampledSpectrum with rgb[0] as miniumum
        *r += *RGB_ILLUM2_SPEC_WHITE * rgb[0];

        if rgb[1] <= rgb[2] {
            *r += *RGB_ILLUM2_SPEC_CYAN * (rgb[1] - rgb[0]);
            *r += *RGB_ILLUM2_SPEC_BLUE * (rgb[2] - rgb[1]);
        } else {
            *r += *RGB_ILLUM2_SPEC_CYAN * (rgb[2] - rgb[0]);
            *r += *RGB_ILLUM2_SPEC_GREEN * (rgb[1] - rgb[2]);
        }
    } else if rgb[1] <= rgb[0] && rgb[1] <= rgb[2] {
        // compute illuminant SampledSpectrum with rgb[1] as miniumum
        *r += *RGB_ILLUM2_SPEC_WHITE * rgb[1];

        if rgb[0] <= rgb[2] {
            *r += *RGB_ILLUM2_SPEC_MAGENTA * (rgb[0] - rgb[1]);
            *r += *RGB_ILLUM2_SPEC_BLUE * (rgb[2] - rgb[0]);
        } else {
            *r += *RGB_ILLUM2_SPEC_MAGENTA * (rgb[2] - rgb[1]);
            *r += *RGB_ILLUM2_SPEC_RED * (rgb[0] - rgb[2]);
        }
    } else {
        // compute illuminant SampledSpectrum with rgb[0] as miniumum
        *r += *RGB_ILLUM2_SPEC_WHITE * rgb[2];

        if rgb[0] <= rgb[1] {
            *r += *RGB_ILLUM2_SPEC_YELLOW * (rgb[0] - rgb[2]);
            *r += *RGB_ILLUM2_SPEC_GREEN * (rgb[1] - rgb[0]);
        } else {
            *r += *RGB_ILLUM2_SPEC_YELLOW * (rgb[1] - rgb[2]);
            *r += *RGB_ILLUM2_SPEC_RED * (rgb[0] - rgb[1]);
        }
    }

    *r *= 0.86445;
}