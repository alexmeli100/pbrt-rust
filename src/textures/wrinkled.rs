use crate::core::texture::{TextureMapping3Ds, Texture, TextureMapping3D, turbulence, TextureFloat, IdentityMapping3D, TextureSpec};
use crate::core::pbrt::Float;
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use std::sync::Arc;

pub struct WrinkledTexture {
    omega   : Float,
    octaves : usize,
    mapping : TextureMapping3Ds,
}

impl WrinkledTexture {
    pub fn new(mapping: TextureMapping3Ds, octaves: usize, omega: Float) -> Self {
        Self { mapping, octaves, omega }
    }
}

impl<T: From<Float>> Texture<T> for WrinkledTexture {
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dpdx = Vector3f::default();
        let mut dpdy = Vector3f::default();
        let p = self.mapping.map(s, &mut dpdx, &mut dpdy);

        T::from(turbulence(&p, &dpdx, &dpdy, self.omega, self.octaves))
    }
}

pub fn create_wrinkled_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let map: TextureMapping3Ds = IdentityMapping3D::new(t2w).into();
    let octaves = tp.find_int("octaves", 8) as usize;
    let roughness = tp.find_float("roughness", 0.5);

    Some(Arc::new(WrinkledTexture::new(map, octaves, roughness).into()))
}

pub fn create_wrinkled_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map: TextureMapping3Ds = IdentityMapping3D::new(t2w).into();
    let octaves = tp.find_int("octaves", 8) as usize;
    let roughness = tp.find_float("roughness", 0.5);

    Some(Arc::new(WrinkledTexture::new(map, octaves, roughness).into()))
}

