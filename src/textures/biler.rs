use crate::core::texture::{TextureMapping2Ds, Texture, TextureMapping2D, TextureFloat, get_mapping2d, TextureSpec};
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::{Vector2f, Vector3f};
use static_assertions::_core::ops::{Mul, Add};
use crate::core::pbrt::Float;
use crate::core::paramset::TextureParams;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;
use std::sync::Arc;

pub struct BilerTexture<T> {
    v00     : T,
    v01     : T,
    v10     : T,
    v11     : T,
    mapping : TextureMapping2Ds,
}

impl<T> BilerTexture<T> {
    pub fn new(mapping: TextureMapping2Ds, v00: T, v01: T, v10: T, v11: T) -> Self {
        Self { v00, v01, v10, v11, mapping }
    }
}

impl<T> Texture<T> for BilerTexture<T>
where T: Add<T, Output = T> + Mul<Float, Output = T> + Copy
{
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dstdx = Vector2f::default();
        let mut dstdy = Vector2f::default();
        let st = self.mapping.map(s, &mut dstdx, &mut dstdy);

        self.v00 * (1.0 - st[1]) * (1.0 - st[0]) + self.v01 * (1.0 - st[0]) * (st[1]) +
        self.v10 * (1.0 - st[1]) * (st[0]) + self.v11 * (st[1]) * (st[0])
    }
}

pub fn create_biler_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let map = get_mapping2d(t2w, tp);
    let v00 = tp.find_float("v00", 0.0);
    let v01 = tp.find_float("v01", 1.0);
    let v10 = tp.find_float("v10", 0.0);
    let v11 = tp.find_float("v11", 1.0);

    Some(Arc::new(BilerTexture::new(map, v00, v01, v10, v11).into()))
}

pub fn create_biler_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map = get_mapping2d(t2w, tp);
    let v00 = tp.find_spectrum("v00", Spectrum::new(0.0));
    let v01 = tp.find_spectrum("v01", Spectrum::new(1.0));
    let v10 = tp.find_spectrum("v10", Spectrum::new(0.0));
    let v11 = tp.find_spectrum("v11", Spectrum::new(1.0));

    Some(Arc::new(BilerTexture::new(map, v00, v01, v10, v11).into()))
}