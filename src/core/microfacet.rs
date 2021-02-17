use crate::core::geometry::vector::Vector3f;
use crate::core::pbrt::{Float, PI, erf, erf_inv};
use crate::core::geometry::point::Point2f;
use crate::core::reflection::{abs_cos_theta, tan2_theta, cos2_theta, cos2_phi, sin2_phi, tan_theta, sin_phi, cos_theta, same_hemisphere, cos_phi};
use std::fmt::{Display, Result, Formatter};
use enum_dispatch::enum_dispatch;
use crate::core::geometry::geometry::spherical_direction;
use crate::materials::disney::DisneyMicrofacetDistribution;

pub fn beckmann_sample11(
    cos_thetai: Float, u1: Float, u2: Float,
    slopex: &mut Float, slopey: &mut Float) {
    // Special case (normal incidence
    if cos_thetai > 0.9999 {
        let r = (-(1.0 - u1).ln()).sqrt();
        let sin_phi = (2.0 * PI * u2).sin();
        let cos_phi = (2.0 * PI * u2).cos();
        *slopex = r * cos_phi;
        *slopey = r * sin_phi;
        return;
    }

    let sin_thetai = ((1.0 - cos_thetai * cos_thetai).max(0.0)).sqrt();
    let tan_thetai = sin_thetai / cos_thetai;
    let cot_thetai = 1.0 / tan_thetai;

    /* Search interval -- everything is parameterized
       in the Erf() domain */
    let mut a = -1.0;
    let mut c = erf(cot_thetai);
    let samplex = u1.max(1.0e-6);

    /* Start with a good initial guess */
    // Float b = (1-sample_x) * a + sample_x * c;

    /* We can do better (inverse of an approximation computed in
     * Mathematica) */
    let thetai = cos_thetai.acos();
    let fit = 1.0 + thetai * (-0.876 + thetai * (0.4265 - 0.0594 * thetai));
    let mut b = c - (1.0 + c) * (1.0 - samplex).powf(fit);

    // Normalize factor for CDF
    let sqrt_inv_pi: Float = 1.0 / PI.sqrt();
    let normalization = 1.0 / (1.0 + c + sqrt_inv_pi * tan_thetai * (-cot_thetai * cot_thetai).exp());

    for _it in 0..10 {
        /* Bisection criterion -- the oddly-looking
           Boolean expression are intentional to check
           for NaNs at little additional cost */
        if !(b >= a && b <= c) { b = 0.5 * (a + c); }

        /* Evaluate the CDF and its derivative
           (i.e. the density function) */
        let inv_erf = erf_inv(b);
        let val = normalization * (1.0 - b + sqrt_inv_pi * tan_thetai * (-inv_erf * inv_erf).exp()) - samplex;
        let derivative = normalization * (1.0 - inv_erf * tan_thetai);

        if val.abs() < 1.0e-5 { break; }

        /* Update bisection intervals */
        if val > 0.0 {
            c = b;
        } else {
            a = b
        }

        b -= val / derivative;
    }

    // Now convert back into a slope value
    *slopex = erf_inv(b);

    // Simulate Y component
    *slopey = erf_inv(2.0 * u2.max(1.0e-6 - 1.0));

    assert!(!(*slopex).is_infinite());
    assert!(!(*slopex).is_nan());
    assert!(!(*slopey).is_infinite());
    assert!(!(*slopey).is_nan());
}

pub fn beckmann_sample(
    wi: &Vector3f, alphax: Float, alphay: Float,
    u1: Float, u2: Float) -> Vector3f {
    // 1. stretch wi
    let wi_stretched = Vector3f::new(alphax * wi.x, alphay * wi.y, wi.z).normalize();

    // 2. simulate P22_{wi}(x_slope, y_slope, 1, 1)
    let mut slopex = 0.0; let mut slopey = 0.0;
    beckmann_sample11(cos2_theta(&wi_stretched), u1, u2, &mut slopex, &mut slopey);

    // 3. rotate
    let tmp = cos2_phi(&wi_stretched) * slopex - sin_phi(&wi_stretched) * slopey;
    slopey = sin_phi(&wi_stretched) * slopex + cos_theta(&wi_stretched) * slopey;
    slopex = tmp;

    // 4. unstretch
    slopex = alphax * slopex;
    slopey = alphay * slopey;

    // 5. compute normal
    Vector3f::new(-slopex, -slopey, 1.0).normalize()
}

#[enum_dispatch]
pub trait MicrofacetDistribution: Display {
    fn d(&self, wh: &Vector3f) -> Float;
    fn lambda(&self, w: &Vector3f) -> Float;
    fn g1(&self, w: &Vector3f) -> Float {
        1.0 / (1.0 + self.lambda(w))
    }

    fn g(&self, wo: &Vector3f, wi: &Vector3f) -> Float {
        1.0 / (1.0 + self.lambda(wo) + self.lambda(wi))
    }
    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f;
    fn pdf(&self, wo: &Vector3f, wh: &Vector3f) -> Float;
}

#[macro_export]
macro_rules! pdf {
    () => {
        fn pdf(&self, wo: &Vector3f, wh: &Vector3f) -> Float {
            if self.samplevis {
                self.d(wh) * self.g1(wo) * wo.abs_dot(wh) / abs_cos_theta(wo)
            } else {
                self.d(wh) * abs_cos_theta(wh)
            }
        }
    }
}

#[enum_dispatch(MicrofacetDistribution, Display)]
pub enum MicrofacetDistributions {
    BeckmannDistribution(BeckmannDistribution),
    DisneyMicrofacetDistribution(DisneyMicrofacetDistribution),
    TrowbridgeReitzDistribution(TrowbridgeReitzDistribution)
}

impl Display for MicrofacetDistributions {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self {
            MicrofacetDistributions::BeckmannDistribution(ref b)         => writeln!(f, "{}", b),
            MicrofacetDistributions::DisneyMicrofacetDistribution(ref d) => writeln!(f, "{}", d),
            MicrofacetDistributions::TrowbridgeReitzDistribution(ref t)  => writeln!(f, "{}", t)
        }
    }
}

pub struct BeckmannDistribution {
    alphax      : Float,
    alphay      : Float,
    samplevis   : bool
}

impl BeckmannDistribution {
    pub fn new(alphax: Float, alphay: Float, samplevis: bool) -> Self {
        Self {
            alphax: (alphax).max(0.001),
            alphay: (alphay).max(0.001),
            samplevis
        }
    }
}

impl MicrofacetDistribution for BeckmannDistribution {
    fn d(&self, wh: &Vector3f) -> f32 {
        let tan2_theta = tan2_theta(wh);

        if tan2_theta.is_infinite() { return 0.0; }
        let cos4_theta = cos2_theta(wh) * cos2_theta(wh);

        (-tan2_theta * (cos2_phi(wh) / (self.alphax * self.alphay) +
            sin2_phi(wh) / (self.alphay * self.alphay))).exp() /
        (PI * self.alphax * self.alphay * cos4_theta)
    }

    fn lambda(&self, w: &Vector3f) -> f32 {
        let abs_tan_theta = tan_theta(w).abs();
        if abs_tan_theta.is_infinite() { return 0.0; }

        // Compute alpha for direction w
        let alpha = (cos2_phi(w) * self.alphax * self.alphax +
            sin2_phi(w) * self.alphay * self.alphay).sqrt();
        let a = 1.0 / (alpha * abs_tan_theta);
        if a >= 1.6 { return 0.0; }

        (1.0 - 1.259 * a + 0.396 * a * a) / (3.535 * a + 2.181 * a * a)
    }

    fn sample_wh(&self, wo: &Vector3f, u: &Point2f) -> Vector3f {
        if !self.samplevis {
            // Sample full distribution of normals for Beckmann distribution

            // Compute tan^2 theta and phi for Beckmann distribution sample
            let tan2_theta: Float;
            let mut phi: Float;

            if self.alphax == self.alphay {
                let log_sample = (1.0 - u[0]).ln();
                assert!(!log_sample.is_infinite());
                tan2_theta = -self.alphax * self.alphax * log_sample;
                phi = u[1] * 2.0 * PI;
            } else {
                // Compute tan2theta and phi for for anisotropic Beckmann distribution
                let log_sample = (1.0 - u[0]).ln();
                assert!(!log_sample.is_infinite());
                phi = (self.alphay / self.alphax * (2.0 * PI * u[1] + 0.5 * PI).tan()).atan();
                if u[1] > 0.5 { phi += PI; }

                let sin_phi = phi.sin();
                let cos_phi = phi.cos();
                let alphax2 = self.alphax * self.alphax;
                let alphay2 = self.alphay * self.alphay;
                tan2_theta = -log_sample / (cos_phi * cos_phi / alphax2 + sin_phi * sin_phi / alphay2);
            }

            // Map sampled Beckmann angles to normal direction wh
            let cos_theta = 1.0 / (1.0 + tan2_theta).sqrt();
            let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
            let mut wh = spherical_direction(sin_theta, cos_theta, phi);

            if !same_hemisphere(wo, &wh) { wh = -wh; }

            wh
        } else {
            // Sample visible area of normals for Beckmann distribution
            let flip = wo.z < 0.0;
            let nwo = -(*wo);
            let mut wh = beckmann_sample(if flip { &nwo } else { wo }, self.alphax, self.alphay, u[0], u[1]);

            if flip { wh = -wh }

            wh
        }

    }

    pdf!();
}

impl Display for BeckmannDistribution {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ BeckmannDistribution alphax: {} alphay: {} ]",
               self.alphax, self.alphay)
    }
}

fn trowbridge_reitz_sample11(
    cos_theta: Float, u1: Float, mut u2: Float,
    slopex: &mut Float, slopey: &mut Float) {
    // Special case (normal incidence
    if cos_theta > 0.9999 {
        let r = (u1 / (1.0 - u1)).sqrt();
        let phi = 6.28318530718 * u2;
        *slopex = r * phi.cos();
        *slopey = r * phi.sin();
        return;
    }

    let sin_theta = ((1.0 - cos_theta * cos_theta).max(0.0)).sqrt();
    let tan_theta = sin_theta / cos_theta;
    let a = 1.0 / tan_theta;
    let G1 = 2.0 / (1.0 + (1.0 + 1.0 / (a * a)).sqrt());

    // sample slopex
    let A = 2.0 * u1 / G1 - 1.0;
    let mut tmp = 1.0 / (A * A - 1.0);
    if tmp > 1.0e10 { tmp = 1.0e10; }
    let B = tan_theta;
    let D = (B * B * tmp * tmp - (A * A - B * B) * tmp).max(0.0);
    let slopex1 = B * tmp - D;
    let slopex2 = B * tmp + D;
    *slopex = if A < 0.0 || slopex2 > 1.0 / tan_theta { slopex1 } else { slopex2 };

    // Sample slopey
    let s = if u2 > 0.5 {
        u2 = 2.0 * (u2 - 0.5);
        1.0
    } else {
        u2 = 2.0 * (0.5 - u2);
        -1.0
    };
    let z =
        (u2 * (u2 * (u2 * u2 * 0.27385 - 0.73369) + 0.46341)) /
        (u2 * (u2 * (u2 * 0.093073 + 0.309420) - 1.000000) + 0.597999);
    *slopey = s * z * (1.0 + *slopex * *slopex);

    assert!(!(*slopey).is_infinite());
    assert!(!(*slopey).is_nan());

}

pub fn trowbridge_reitz_sample(
    wi: &Vector3f, alphax: Float, alphay: Float,
    u1: Float, u2: Float) -> Vector3f {
    // 1. stretch wi
    let wi_stretched = Vector3f::new(alphax * wi.x, alphay * wi.y, wi.z).normalize();

    // 2. simulate pzz{wi}(x_slope, y_slope, 1, 1)
    let mut slopex = 0.0;
    let mut slopey = 0.0;
    trowbridge_reitz_sample11(cos_theta(&wi_stretched), u1, u2, &mut slopex, &mut slopey);

    // 3. rotate
    let tmp = cos_phi(&wi_stretched) * slopex - sin_phi(&wi_stretched) * slopey;
    slopey = sin_phi(&wi_stretched) * slopex + cos_phi(&wi_stretched) * slopey;
    slopex = tmp;

    // 4. unstretch
    slopex = alphax * slopex;
    slopey = alphay * slopey;

    // 5. compute normal
    Vector3f::new(-slopex, -slopey, 1.0).normalize()
}

pub struct TrowbridgeReitzDistribution {
    alphax      : Float,
    alphay      : Float,
    samplevis   : bool
}

impl TrowbridgeReitzDistribution {
    pub fn new(alphax: Float, alphay: Float, samplevis: bool) -> Self {
        Self {
            alphax: (alphax).max(0.001),
            alphay: (alphay).max(0.001),
            samplevis
        }
    }

    #[inline(always)]
    pub fn roughness_to_alpha(mut roughness: Float) -> Float {
        roughness = roughness.max(1.0e-3);
        let x = roughness.ln();

        1.62142 + 0.819955 * x + 0.1734 * x * x +
        0.0171201 * x * x * x + 0.000640711 * x * x * x * x
    }
}

impl MicrofacetDistribution for TrowbridgeReitzDistribution {
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

impl Display for TrowbridgeReitzDistribution {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        write!(f, "[ TrowbridgeReitzDistribution alphax: {} alphay: {} ]",
               self.alphax, self.alphay)
    }
}