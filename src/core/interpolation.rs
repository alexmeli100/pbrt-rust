use crate::core::pbrt::{Float, find_interval};

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