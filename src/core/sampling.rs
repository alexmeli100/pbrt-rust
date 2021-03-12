    use crate::core::geometry::point::Point2f;
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::pbrt::{PI_OVER4, PI_OVER2, Float, INV_PI, PI, find_interval, INV2_PI, INV4_PI, lerp, clamp};
use crate::core::rng::{RNG, ONE_MINUS_EPSILON};

pub struct Distribution1D {
    pub func        : Vec<Float>,
    pub cdf         : Vec<Float>,
    pub func_int    : Float
}

impl Distribution1D {
    pub fn new(func: Vec<Float>) -> Self {
        let n = func.len();
        let mut cdf = vec![0.0; n + 1];

        // Compute integral of step function xi
        for i in 1..n + 1 {
            cdf[i] = cdf[i - 1] + func[i - 1] / n as Float;
        }

        // Transform step function integral into CDF
        let func_int = cdf[n];

        if func_int == 0.0 {
            for i in 1..n + 1 { cdf[i] = i as Float / n as Float; }
        } else {
            for i in 1..n + 1 { cdf[i] /= func_int; }
        }

        Self { func, cdf, func_int }
    }

    pub fn count(&self) -> usize { self.func.len() }

    pub fn sample_continous(
        &self, u: Float, pdf: Option<&mut Float>,
        off: Option<&mut usize>) -> Float {
        // Find surrounding CDF segments and offset
        let offset = find_interval(self.cdf.len() as i32, |i| self.cdf[i as usize] <= u) as usize;

        if let Some(off) = off {
            *off = offset;
        }

        // Compute offset along CDF segment
        let mut du = u - self.cdf[offset];
        let diff = self.cdf[offset + 1] - self.cdf[offset];

        if diff > 0.0 {
            assert!(self.cdf[offset + 1] > self.cdf[offset]);
            du /= diff
        }

        assert!(!du.is_nan());

        // Compute PDF for sampled offset
        if let Some(p) = pdf {
            *p = if self.func_int > 0.0 { self.func[offset] / self.func_int } else { 0.0 };
        }

        (offset as Float + du) / self.count() as Float
    }

    pub fn sample_discrete(
        &self, u: Float, pdf: Option<&mut Float>,
        uremmaped: Option<&mut Float>) -> usize {
        // Find surrounding CDF segments and offset
        let offset = find_interval(self.cdf.len() as i32, |i| self.cdf[i as usize] <= u) as usize;

        if let Some(p) = pdf {
            *p = if self.func_int > 0.0 {
                self.func[offset] / (self.func_int * self.count() as Float)
            } else {
                0.0
            };
        }

        if let Some(r) = uremmaped {
            *r = (u - self.cdf[offset]) / (self.cdf[offset + 1] - self.cdf[offset]);
            assert!(*r >= 0.0 && *r <= 1.0)
        }

        offset
    }

    pub fn discrete_pdf(&self, index: usize) -> Float {
        assert!(index < self.count());

        self.func[index] / (self.func_int * self.count() as Float)
    }
}

pub struct Distribution2D {
    pconditional_v  : Vec<Distribution1D>,
    pmarginal       : Distribution1D
}

impl Distribution2D {
    pub fn new(func: &[Float], nu: usize, nv: usize) -> Self {
        let mut pcond = Vec::with_capacity(nv);

        // Compute conditional sampling distribution for v
        for v in 0..nv {
            let f = func[(v * nu)..(v + 1) * nu].to_vec();
            pcond.push(Distribution1D::new(f))
        }

        // Compute marginal sampling distribution v
        let mut marginal_func = Vec::with_capacity(nv);

        for v in 0..nv {
            marginal_func.push(pcond[v].func_int);
        }

        let pmarginal = Distribution1D::new(marginal_func);

        Self { pmarginal, pconditional_v: pcond }
    }

    pub fn sample_continuous(&self, u: &Point2f, pdf: &mut Float) -> Point2f {
        let mut pdfs = [0.0; 2];
        let mut v = 0;

        let d1 = self.pmarginal.sample_continous(u[1], Some(&mut pdfs[1]), Some(&mut v));
        let d0 = self.pconditional_v[v].sample_continous(u[0], Some(&mut pdfs[0]), None);
        *pdf = pdfs[0] * pdfs[1];

        Point2f::new(d0, d1)

    }

    pub fn pdf(&self, p: &Point2f) -> Float {
        let iu = clamp(
            (p[0] * self.pconditional_v[0].count() as Float) as usize,
            0,
            self.pconditional_v[0].count() - 1);
        let iv = clamp(
            (p[1] * self.pmarginal.count() as Float) as usize,
            0,
            self.pmarginal.count() - 1);

        self.pconditional_v[iv].func[iu] / self.pmarginal.func_int
    }
}

pub fn uniform_sample_disk(u: &Point2f) -> Point2f {
    let r = u[0].sqrt();
    let theta = 2.0 * PI * u[1];

    Point2f::new(r * theta.cos(), r * theta.sin())
}

pub fn concentric_sample_disk(u: &Point2f) -> Point2f {
    // Map uniform random number sto [-1, 1]^2
    let uoffset = *u * 2.0 - Vector2f::new(1.0, 1.0);

    // Handle degeneracy at the origin
    if uoffset.x == 0.0 && uoffset.y == 0.0 { 
        return Point2f::new(0.0, 0.0)
    }
    
    // Apply concentric mapping to point
    let theta: Float;
    let r: Float;
    
    if uoffset.x.abs() > uoffset.y.abs() {
        r = uoffset.x;
        theta = PI_OVER4 * (uoffset.y / uoffset.x);
    } else {
        r = uoffset.y;
        theta = PI_OVER2 - PI_OVER4 * (uoffset.x / uoffset.y)
    }
    
    Point2f::new(theta.cos(), theta.sin()) * r
}

pub fn shuffle<T>(samp: &mut [T], count: usize, ndimensions: usize, rng: &mut RNG) {
    for i in 0..count {
        let other = i + rng.uniform_int32_2((count - i) as u32) as usize;

        for j in 0..ndimensions {
            samp.swap(ndimensions * i + j, ndimensions * other + j);
        }
    }
}

pub fn cosine_sample_hemisphere(u: &Point2f) -> Vector3f {
    let d = concentric_sample_disk(u);
    let z = ((0.0 as Float).max(1.0 - d.x * d.x - d.y * d.y)).sqrt();
    
    Vector3f::new(d.x, d.y, z)
}

pub fn cosine_hemisphere_pdf(cos_theta: Float) -> Float {
    cos_theta * INV_PI
}

pub fn uniform_sample_hemisphere(u: &Point2f) -> Vector3f {
    let z = u[0];
    let r = ((0.0 as Float).max(1.0 - z * z)).sqrt();
    let phi = 2.0 * PI * u[1];
    
    Vector3f::new(r * phi.cos(), r * phi.sin(), z)
    
}

pub fn uniform_hemisphere_pdf() -> Float {
    INV2_PI
}

pub fn uniform_sample_sphere(u: &Point2f) -> Vector3f {
    let z = 1.0 - 2.0 * u[0];
    let r = ((1.0 - z * z).max(0.0)).sqrt();
    let phi = 2.0 * PI * u[1];

    Vector3f::new(r * phi.cos(), r * phi.sin(), z)
}

pub fn uniform_sphere_pdf() -> Float { INV4_PI }

pub fn uniform_cone_pdf(cos_thetamax: Float) -> Float {
    1.0 / (2.0 * PI * (1.0 - cos_thetamax))
}

pub fn uniform_sample_cone(u: &Point2f, cos_thetamax: Float) -> Vector3f {
    let cos_theta = (1.0 - u[0]) + u[0] * cos_thetamax;
    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
    let phi = u[1] * 2.0 * PI;

    Vector3f::new(phi.cos() * sin_theta, phi.sin() * sin_theta, cos_theta)
}

pub fn uniform_sample_cone2(
    u: &Point2f, cos_thetamax: Float, x: &Vector3f,
    y: &Vector3f, z: &Vector3f) -> Vector3f {
    let cos_theta = lerp(u[0], cos_thetamax, 1.0);
    let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();
    let phi = u[1] * 2.0 * PI;

    *x * sin_theta * phi.cos() + *y * sin_theta * phi.sin() + *z * cos_theta
}

pub fn uniform_sample_triangle(u: &Point2f) -> Point2f {
    let su0 = u[0].sqrt();

    Point2f::new(1.0 - su0, u[1] * su0)
}

pub fn stratified_sample_1d(samp: &mut [Float], nsamples: usize, rng: &mut RNG, jitter: bool) {
    let inv_samples = 1.0 / nsamples as Float;

    for i in 0..nsamples {
        let delta = if jitter { rng.uniform_float()} else { 0.5 };
        samp[i] = ((i as Float + delta) * inv_samples).min(ONE_MINUS_EPSILON)
    }
}

pub fn rejection_sample_hemisphere(rng: &mut RNG) -> Point2f {
    let mut p = Point2f::default();

    loop {
        p.x = 1.0 - 2.0 * rng.uniform_float();
        p.y = 1.0 - 2.0 * rng.uniform_float();

        if p.x * p.x + p.y * p.y > 1.0 { break; }
    }

    p
}

pub fn stratified_sample_2d(samp: &mut [Point2f], nx: usize, ny: usize, rng: &mut RNG, jitter: bool) {
    let dx = 1.0 / nx as Float;
    let dy = 1.0 / ny as Float;
    let mut i = 0;

    for y in 0..ny {
        for x in 0..nx {
            let jx = if jitter { rng.uniform_float() } else { 0.5 };
            let jy = if jitter { rng.uniform_float() } else { 0.5 };
            samp[i].x = ((x as Float + jx) * dx).min(ONE_MINUS_EPSILON);
            samp[i].y = ((y as Float + jy) * dy).min(ONE_MINUS_EPSILON);
            i += 1;
        }
    }
}

macro_rules! swap_sample {
    ($v:ident, $i:ident, $j:ident, $dim:ident) => {{
        let tmp = $v[$i].$dim;
        $v[$i].$dim = $v[$j].$dim;
        $v[$j].$dim = tmp;
    }}
}

pub fn latin_hypercube(samples: &mut [Point2f], nsamples: usize, ndim: usize, rng: &mut RNG) {
    // Generate LHS samples along diagonal
    let inv_samples = 1.0 / nsamples as Float;

    for i in 0..nsamples {
        for j in 0..ndim {
            let sj = (i as Float + (rng.uniform_float())) * inv_samples;
            match j {
                0 => samples[i].x = sj.min(ONE_MINUS_EPSILON),
                _ => samples[i].y = sj.min(ONE_MINUS_EPSILON)
            }
        }
    }

    // Permute LHS samples in each dimension
    for i in 0..ndim {
        for j in 0..nsamples {
            let other = j + rng.uniform_int32_2((nsamples - j) as u32) as usize;
            match i {
                0 => { swap_sample!(samples, j, other, x) },
                _ => { swap_sample!(samples, j, other, y) }
            }
        }
    }
}

#[inline(always)]
pub fn balance_heuristic(nf: usize, fpdf: Float, ng: usize, gpdf: Float) -> Float {
    (nf as Float * fpdf) / (nf as Float * fpdf * ng as Float * gpdf)
}

#[inline(always)]
pub fn power_heuristic(nf: usize, fpdf: Float, ng: usize, gpdf: Float) -> Float {
    let f = nf as Float * fpdf;
    let g = ng as Float * gpdf;
    
    (f * f) / (f * f + g * g)
}