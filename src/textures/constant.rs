use crate::core::texture::{Texture, TextureFloat, TextureSpec};
use crate::core::interaction::SurfaceInteraction;
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;
use bumpalo::core_alloc::sync::Arc;

pub struct ConstantTexture<T> {
    value: T
}

impl<T> ConstantTexture<T> {
   pub fn new(value: T) -> Self {
        Self { value }
    }
}

impl<T: Copy> Texture<T> for ConstantTexture<T> {
    fn evaluate(&self, _s: &SurfaceInteraction) -> T {
        self.value
    }
}

pub fn create_constant_float(_t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let value = tp.find_float("value", 1.0);

    Some(Arc::new(ConstantTexture::new(value).into()))
}

pub fn create_constant_spectrum(_t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let value = tp.find_spectrum("value", Spectrum::new(1.0));

    Some(Arc::new(ConstantTexture::new(value).into()))
}