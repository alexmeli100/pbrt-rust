use std::sync::Arc;
use crate::core::texture::{Textures, TextureFloat, Texture, SpectrumT, TextureSpec};
use crate::core::interaction::SurfaceInteraction;
use crate::core::paramset::TextureParams;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;


pub struct MixTexture<T: Copy>
where T: SpectrumT<T>
{
    tex1    : Arc<Textures<T, T>>,
    tex2    : Arc<Textures<T, T>>,
    amount  : Arc<TextureFloat>
}

impl<T: Copy> MixTexture<T>
    where T: SpectrumT<T>
{
    pub fn new(
        tex1: Arc<Textures<T, T>>, tex2: Arc<Textures<T, T>>,
        amount: Arc<TextureFloat>) -> Self {
        Self { tex1, tex2, amount }
    }
}

impl<T> Texture<T> for MixTexture<T>
    where T: SpectrumT<T>
{
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let t1 = self.tex1.evaluate(s);
        let t2 = self.tex2.evaluate(s);
        let amt = self.amount.evaluate(s);

        t1 * (1.0 - amt) + t2 * amt
    }
}

pub fn create_mix_float(_t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let tex1 = tp.get_floattexture("tex1", 0.0);
    let tex2 = tp.get_floattexture("tex2", 1.0);
    let amount = tp.get_floattexture("amount", 0.5);

    Some(Arc::new(MixTexture::new(tex1, tex2, amount).into()))
}

pub fn create_mix_spectrum(_t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let tex1 = tp.get_spectrumtexture("tex1", Spectrum::new(0.0));
    let tex2 = tp.get_spectrumtexture("tex2", Spectrum::new(1.0));
    let amount = tp.get_floattexture("amount", 0.5);

    Some(Arc::new(MixTexture::new(tex1, tex2, amount).into()))
}