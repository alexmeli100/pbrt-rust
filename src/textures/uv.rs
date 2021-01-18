use crate::core::texture::{TextureMapping2Ds, Texture, TextureMapping2D, TextureSpec, UVMapping2D, SphericalMapping2D, CylindricalMapping2D, PlannarMapping2D, get_mapping2d, TextureFloat};
use crate::core::spectrum::{Spectrum, SpectrumType};
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::{Vector2f};
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use std::sync::Arc;

pub struct UVTexture {
    mapping: TextureMapping2Ds
}

impl UVTexture {
    pub fn new(mapping: TextureMapping2Ds) -> Self {
        Self { mapping }
    }
}

impl<T: From<Spectrum>> Texture<T> for UVTexture {
    fn evaluate(&self, s: &SurfaceInteraction) -> T{
        let mut dstdx = Vector2f::default();
        let mut dstdy = Vector2f::default();
        let st = self.mapping.map(s, &mut dstdx, &mut dstdy);
        let rgb = [
            st.x - st.x.floor(),
            st.y - st.y.floor(),
            0.0
        ];
        let s = Spectrum::from_rgb(rgb, SpectrumType::Reflectance);

        T::from(s)
    }
}

pub fn create_uv_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    None
}

pub fn create_uv_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map = get_mapping2d(t2w, tp);

    Some(Arc::new(UVTexture::new(map).into()))
}

