use std::sync::Arc;
use crate::core::texture::{Textures, Texture, TextureFloat, TextureSpec, SpectrumT};
use crate::core::interaction::SurfaceInteraction;
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;
use std::ops::{Mul, Add, AddAssign};

pub struct ScaleTexture<T1: Copy, T2: Copy>
where T1: SpectrumT<T1>,
      T2: SpectrumT<T2>
{
    tex1: Arc<Textures<T1, T1>>,
    tex2: Arc<Textures<T2, T2>>,
}

impl<T1: Copy, T2: Copy> ScaleTexture<T1, T2>
    where T1: SpectrumT<T1>,
          T2: SpectrumT<T2>
{
    pub fn new(tex1: Arc<Textures<T1, T1>>, tex2: Arc<Textures<T2, T2>>) -> Self {
        Self { tex1, tex2 }
    }
}

impl<T1: Copy, T2: Copy> Texture<T2> for ScaleTexture<T1, T2>
    where T1: Mul<T2, Output=T2> + SpectrumT<T1>,
          T2: SpectrumT<T2>

{
    fn evaluate(&self, s: &SurfaceInteraction) -> T2 {
        self.tex1.evaluate(s) * self.tex2.evaluate(s)
    }
}

pub fn create_scale_float(
    t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let s = ScaleTexture::new(
        tp.get_floattexture("tex1", 1.0),
        tp.get_floattexture("tex2", 1.0)
    );

    Some(Arc::new(s.into()))
}

pub fn create_scale_spectrum(
    t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let s = ScaleTexture::new(
        tp.get_spectrumtexture("tex1", Spectrum::new(1.0)),
        tp.get_spectrumtexture("tex2", Spectrum::new(1.0))
    );

    Some(Arc::new(s.into()))
}