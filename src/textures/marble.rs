use crate::core::texture::{TextureMapping3Ds, Texture, TextureMapping3D, fbm, TextureSpec, IdentityMapping3D, TextureFloat};
use crate::core::pbrt::Float;
use crate::core::spectrum::Spectrum;
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use std::sync::Arc;

const C: [[Float; 3]; 9] = [
    [0.58, 0.58, 0.6], [0.58, 0.58, 0.6], [0.58, 0.58, 0.6],
    [0.5, 0.5, 0.5],   [0.6, 0.59, 0.58], [0.58, 0.58, 0.6],
    [0.58, 0.58, 0.6], [0.2, 0.2, 0.33],  [0.58, 0.58, 0.6],
];

pub struct MarbleTexture {
    omega      : Float,
    scale      : Float,
    variation  : Float,
    octaves    : usize,
    mapping    : TextureMapping3Ds,
}

impl MarbleTexture {
    pub fn new(
        omega: Float, scale: Float, variation: Float,
        octaves: usize, mapping: TextureMapping3Ds) -> Self {
        Self {
            omega, scale, variation,
            octaves, mapping
        }
    }
}

impl<T: From<Spectrum>> Texture<T> for MarbleTexture {
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        use crate::core::spectrum::SpectrumType::*;

        let mut dpdx = Vector3f::default();
        let mut dpdy = Vector3f::default();
        let mut p = self.mapping.map(s, &mut dpdx, &mut dpdy);

        p *= self.scale;
        let fbm = fbm(
            &p, &(dpdx * self.scale), &(dpdy * self.scale),
            self.omega, self.octaves);
        let marble = p.y + self.variation * fbm;
        let t = 0.5 + 0.5 * marble.sin();

        // Evaluate marble spline at t
        let nseg = 6;
        let first = std::cmp::min(5, (t * nseg as Float).floor() as usize);
        let c0 = Spectrum::from_rgb(C[first], Reflectance);
        let c1 = Spectrum::from_rgb(C[first + 1], Reflectance);
        let c2 = Spectrum::from_rgb(C[first + 2], Reflectance);
        let c3 = Spectrum::from_rgb(C[first + 3], Reflectance);
        // Bezier spline evaluated with Castilejau's algorithm
        let mut s0 = c0 * (1.0 - t) + c1 * t;
        let mut s1 = c1 * (1.0 - t) + c2 * t;
        let s2 = c2 * (1.0 - t) + c3 * t;
        s0 = s0 * (1.0 - t) + s1 * t;
        s1 = s1 * (1.0 - t) + s2 * t;

        T::from((s0 * (1.0 - t) + s1 * t) * 1.5)
    }
}

pub fn create_marble_float(_t2w: &Transform, _tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    None
}

pub fn create_marble_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map: TextureMapping3Ds = IdentityMapping3D::new(t2w).into();
    let octaves = tp.find_int("octaves", 8) as usize;
    let omega = tp.find_float("roughness", 0.5);
    let scale = tp.find_float("scale", 1.0);
    let variation = tp.find_float("variation", 0.2);

    Some(Arc::new(MarbleTexture::new(omega, scale, variation, octaves, map).into()))

}

