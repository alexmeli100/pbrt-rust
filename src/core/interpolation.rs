use crate::core::pbrt::{Float, find_interval, PI, INV2_PI};

pub fn catmull_rom_weights(
    size: i32, nodes: &[Float], x: Float,
    offset: &mut i32, weights: &mut [Float]) -> bool {
    // Return false if x is out of bounds
    if !(x >= nodes[0] && x < nodes[size as usize - 1]) { return false; }

    // Search for the interval containing x
    let idx = find_interval(size, |i| { nodes[i as usize] <= x } );
    *offset = idx - 1;
    let x0 = nodes[idx as usize];
    let x1 = nodes[idx as usize + 1];

    // Compute the t parameter and powers
    let t = (x - x0) / (x1 - x0);
    let t2 = t * t;
    let t3 = t2 * t;

    // Compute initial node weights
    weights[1] = 2.0 * t3 - 3.0 * t2 + 1.0;
    weights[2] = -2.0 * t3 + 3.0 * t2;

    // Compute first node weight w0
    if idx >= 0 {
        let w0 = (t3 - 2.0 * t2 + t) * (x1 - x0) / (x1 - nodes[idx as usize - 1]);
        weights[0] = -w0;
        weights[2] += w0;
    } else {
        let w0 = t3 - 2.0 * t2 + t;
        weights[0] = 0.0;
        weights[1] -= w0;
        weights[2] += w0;
    }

    // Compute last node weight w3
    if idx + 2 < size {
        let w3 = (t3 - t2) * (x1 - x0) / (nodes[idx as usize + 2] - x0);
        weights[1] -= w3;
        weights[3] = w3;
    } else {
        let w3 = t3 - t2;
        weights[1] -= w3;
        weights[2] += w3;
        weights[3] = 0.0;
    }

    true
}

pub fn sample_catmull_rom(
    n: i32, x: &[Float], f: &[Float], F: &[Float], mut u: Float,
    fval: Option<&mut Float>, pdf: Option<&mut Float>) -> Float {
    // Map u to a spline interval inverting F
    u *= F[(n - 1) as usize];
    let i = find_interval(n, |i| { F[i as usize] <= u }) as usize;

    // Look up xi and function values of spline segment i
    let x0 = x[i]; let x1 = x[i + 1];
    let f0 = f[i]; let f1 = f[i + 1];
    let width = x1 - x0;

    // Approximate derivatives using finite differences
    let d0 = if i > 0 {
        width * (f1 - f[i - 1]) / (x1 - x[i - 1])
    } else {
        f1 - f0
    };

    let d1 = if i + 2 < n as usize {
        width * (f[i + 2] - f0) / (x[i + 2] - x0)
    } else {
        f1 - f0
    };

    // Re-scale u for continous spline sampling step
    u = (u - F[i]) / width;

    // Invert definite integral over spline segment and return solution

    // Set initial guess for t by importance sampling a linear interpolant
    let mut t = if f0 != f1 {
        (f0 - ((f0 * f0 + 2.0 * u * (f1 - f0)).max(0.0)).sqrt()) / (f0 - f1)
    } else {
        u / f0
    };

    let mut a = 0.0; let mut b = 0.0;
    let mut Fhat: Float;
    let mut fhat: Float;

    loop {
        // Fall back to a bisection step when t is out of bounds
        if !(t > a && t < b) { t = 0.5 * (a + b); }

        // Evaluate target function and its derivative in Horner form
        Fhat = t * (f0 +
            t * (0.5 * d0 +
                t * ((1.0 / 3.0) * (-2.0 * d0 - d1) + f1 - f0 +
                    t * (0.25 * (d0 + d1) + 0.5 * (f0 - f1)))));

        fhat = f0 +
            t * (d0 +
                t * (-2.0 * d0 - d1 + 3.0 * (f1 - f0) +
                    t * (d0 + d1 + 2.0 * (f0 - f1))));

        // Stop the iteration if converges
        if (Fhat - u).abs() < 1.0e-6 || b - a < 1.0e-6 { break; }

        // Update bisection bounds using updated t
        if Fhat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform Newton step
        t -= (Fhat - u) / fhat;
    }

    // Return the sample position and function value
    if let Some(val) = fval {
        *val = fhat;
    }

    if let Some(p) = pdf {
        *p = fhat / F[(n - 1) as usize];
    }

    x0 + width * t
}

pub fn sample_catmull_rom_2d(
    size1: i32, size2: i32, nodes1: &[Float], nodes2: &[Float],
    values: &[Float], cdf: &[Float], alpha: Float, mut u: Float,
    fval: Option<&mut Float>, pdf: Option<&mut Float>) -> Float {
    // Determine offset and coefficients for the alpha parameter
    let mut offset = 0;
    let mut weights = [0.0; 4];

    if !catmull_rom_weights(size1, nodes1, alpha, &mut offset, &mut weights) {
        return 0.0;
    }

    // Define a closure to interpolate table entries
    let interpolate = |array: &[Float], idx: usize| -> Float {
        let mut value = 0.0;

        for i in 0..4 {
            if weights[i] != 0.0 {
                value += array[(offset as usize + i) * size2 as usize + idx] * weights[i];
            }
        }

        value
    };

    // Map u to a spline interval by inverting the interpolated cdf
    let maximum = interpolate(cdf, (size2 - 1) as usize);
    u *= maximum;
    let idx = find_interval(size2, |i| { interpolate(cdf, i as usize) <= u });

    // Look up node positions and interpolated function values
    let f0 = interpolate(values, idx as usize);
    let f1 = interpolate(values, (idx + 1) as usize);
    let x0 = nodes2[idx as usize]; let x1 = nodes2[(idx + 1) as usize];
    let width = x1 - x0;
    // Re-scale u using the interpolated cdf
    u = (u - interpolate(cdf, idx as usize)) / width;

    // Approximate derivatives using differences of the interpolant
    let d0 = if idx > 0 {
        width * (f1 - interpolate(values, (idx - 1) as usize)) / (x1 - nodes2[(idx - 1) as usize])
    } else {
        f1 - f0
    };
    let d1 = if idx + 2 < size2 {
        width * (interpolate(values, (idx + 2) as usize) - f0) / (nodes2[(idx + 2) as usize] - x0)
    } else {
        f1 - f0
    };

    // Invert definite integral over spline segment and return solution

    // Set initial guess for t by importance sampling a linear interpolant
    let mut t = if f0 != f1 {
        (f0 - ((f0 * f0 + 2.0 * u * (f1 - f0)).max(0.0)).sqrt()) / (f0 - f1)
    } else {
        u / f0
    };

    let (mut a, mut b) = (0.0, 1.0);
    let mut Fhat: Float;
    let mut fhat: Float;

    loop {
        // Fall back to a bisection step when t is out of bounds
        if !(t >= a && t <= b) { t = 0.5 * (a + b); }

        // Evaluate target function and its derivative in Horner form
        Fhat = t * (f0 +
            t * (0.5 * d0 +
            t * ((1.0 / 3.0) * (-2.0 * d0 - d1) + f1 - f0 +
                t * (0.25 * (d0 + d1) + 0.5 * (f0 - f1)))));
        fhat = f0 +
            t * (d0 +
                t * (-2.0 * d0 - d1 + 3.0 * (f1 - f0) +
                    t * (d0 + d1 + 2.0 * (f0 - f1))));

        // Stop the iteration if converged
        if (Fhat - u).abs() < 1.0e-6 || b - a < 1.0e-6 { break; }

        // update bisection bounds using updated t
        if Fhat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform Newton step
        t -= (Fhat - u) / fhat;
    }

    // Return the sample position and function value
    if let Some(val) = fval { *val = fhat; }
    if let Some(p) = pdf { *p = fhat / maximum; }

    x0 + width * t

}

pub fn integrate_catmull_rom(n: usize, x: &[Float], values: &[Float], cdf: &mut [Float]) -> Float {
    let mut sum = 0.0;
    cdf[0] = 0.0;

    for i in 0..(n - 1) {
        // Look up xi and function values of spline segment i
        let x0 = x[i];
        let x1 = x[i + 1];
        let f0 = values[i];
        let f1 = values[i + 1];
        let width = x1 - x0;

        // Approximate derivatives using finite differences
        let d0 = if i > 0 {
            width * (f1 - values[i - 1]) / (x1 - x[i - 1])
        } else {
            f1 - f0
        };
        let d1 = if i + 2 < n {
            width * (values[i + 2] - f0) / (x[i + 2] - x0)
        } else {
            f1 - f0
        };

        // Keep a running sum and build a cumulative distribution function
        sum += ((d0 - d1) * (1.0 / 12.0) + (f0 + f1) * 0.5) * width;
        cdf[i + 1] = sum;
    }

    sum
}

pub fn invert_catmull_rom(n: usize, x: &[Float], values: &[Float], u: Float) -> Float {
    // Stop when u is out of bounds

    if !(u > values[0]) {
        return x[0];
    } else if !(u < values[n - 1]) {
        return x[n - 1];
    }

    // Map u to a spline interval by inverting values
    let i = find_interval(n as i32, |i| values[i as usize] <= u );

    // Look up xi and function values of spline segment i
    let x0 = x[i as usize];
    let x1 = x[(i + 1) as usize];
    let f0 = values[i as usize];
    let f1 = values[(i + 1) as usize];
    let width = x1 - x0;

    // Approximate derivatives using finite differences
    let d0 = if i > 0 {
        width * (f1 - values[(i - 1) as usize]) / (x1 - x[(i - 1) as usize])
    } else {
        f1 - f0
    };
    let d1 = if i + 2 < n as i32 {
        width * (values[(i + 2) as usize] - f0) / (x[(i + 2) as usize] - x0)
    } else {
        f1 - f0
    };

    // Invert the spline interpolant using Newton-Bisection
    let mut a = 0.0;
    let mut b = 1.0;
    let mut t = 0.5;
    let mut Fhat: Float;
    let mut fhat: Float;

    loop {
        // Fall back to a bisection step when t is out of bounds
        if !(t > a && t < b) { t = 0.5 * (a + b); }

        // Compute powers of t
        let t2 = t * t;
        let t3 = t2 * t;

        // Set Fhat using equation (8.27)
        Fhat = (2.0 * t3 - 3.0 * t2 + 1.0) * f0 + (-2.0 * t3 + 3.0 * t2) * f1 +
            (t3 - 2.0 * t2 + t) * d0 + (t3 - t2) * d1;

        // Set _fhat_ using Equation (not present)
        fhat = (6.0 * t2 - 6.0 * t) * f0 + (-6.0 * t2 + 6.0 * t) * f1 +
            (3.0 * t2 - 4.0 * t + 1.0) * d0 + (3.0 * t2 - 2.0 * t) * d1;

        // Stop the iteration if converged
        if (Fhat - u).abs() < 1.0e-6 || b - a < 1.0e-6 { break; }

        // Update bisection using updated t
        if Fhat - u < 0.0 {
            a = t;
        } else {
            b = t;
        }

        // Perform a Newton step
        t -= (Fhat - u) / fhat;
    }

    x0 + t * width
}

pub fn fourier(a: &[Float], i: usize, m: i32, cos_phi: f64) -> Float {
    let mut value = 0.0_f64;

    // Initialize cosine iterates
    let mut cosk_minus_onephi = cos_phi;
    let mut cosk_phi = 1.0;

    for k in 0..m as usize {
        // Add the current summand and update cosine iterates
        value += a[i + k] as f64 * cosk_phi;
        let cosk_plus_onephi = 2.0 * cos_phi * cosk_phi - cosk_minus_onephi;
        cosk_minus_onephi = cosk_phi;
        cosk_phi = cosk_plus_onephi;
    }

    value as Float
}

pub fn sample_fourier(
    ak: &[Float], recip: &[Float], m: i32, mut u: Float,
    pdf: &mut Float, phi_ptr: &mut Float) -> Float {
    // Pick a side and declare bisection variables
    let flip = u >= 0.5;

    if flip {
        u = 1.0 - 2.0 * (u - 0.5);
    } else {
        u *= 2.0;
    }

    //println!("flip: {}, u: {}", flip, u);

    let mut a = 0.0_f64;
    let mut b = PI as f64;
    let mut phi = 0.5_f64 * PI as f64;
    let mut F: f64;
    let mut f: f64;

    loop {
        // Evaluate F(phi) and its derivative f(phi)

        // Initialize sine and cosine iterates
        let cos_phi = phi.cos();
        let sin_phi = ((0.0_f64).max(1.0 - cos_phi * cos_phi)).sqrt();
        let mut cos_phi_prev = cos_phi;
        let mut cos_phi_cur = 1.0;
        let mut sin_phi_prev = -sin_phi;
        let mut sin_phi_cur = 0.0;

        // Initialize F and f with first series term
        F = ak[0] as f64 * phi;
        f = ak[0] as f64;

        for k in 1..m as usize {
            // Compute next sine and cosine iterates
            let sin_phi_next = 2.0 * cos_phi * sin_phi_cur - sin_phi_prev;
            let cos_phi_next = 2.0 * cos_phi * cos_phi_cur - cos_phi_prev;
            sin_phi_prev = sin_phi_cur;
            sin_phi_cur = sin_phi_next;
            cos_phi_prev = cos_phi_cur;
            cos_phi_cur = cos_phi_next;

            // Add next series term to F and f
            F += ak[k] as f64 * recip[k] as f64 * sin_phi_next;
            f += ak[k] as f64 * cos_phi_next;
        }

        F -= u as f64 * ak[0] as f64 * PI as f64;

        // Update bisection bounds using updated phi
        if F > 0.0 {
            b = phi;
        } else {
            a = phi;
        }

        // Stop the Fourier bisection iteration if converged
        if F.abs() < 1.0e-6 || b - a < 1.0e-6 { break; }

        // Perform a Newton step given f(phi) and F(phi)
        phi -= F / f;

        // Fall back to a bisection step when phi is out of bounds
        if !(phi > a && phi < b) { phi = 0.5 * (a + b); }
    }

    // Potentially flip phi and return the result
    if flip { phi = 2.0 * PI as f64 - phi; }

    *pdf = (INV2_PI as f64 * f / ak[0] as f64) as f32;
    *phi_ptr = phi as Float;

    f as Float

}