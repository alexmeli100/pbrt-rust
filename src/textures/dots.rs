use std::sync::Arc;
use crate::core::texture::{Textures, Texture, SpectrumT, TextureMapping2Ds, TextureMapping2D, noise, TextureFloat, get_mapping2d, TextureSpec};
use crate::core::interaction::SurfaceInteraction;
use crate::core::pbrt::Float;
use crate::core::geometry::vector::Vector2f;
use crate::core::geometry::point::Point2f;
use crate::core::paramset::TextureParams;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;

pub struct DotsTexture<T: SpectrumT<T>> {
    mapping : TextureMapping2Ds,
    outside : Arc<Textures<T, T>>,
    inside  : Arc<Textures<T, T>>,
}

impl<T: SpectrumT<T>> DotsTexture<T> {
    pub fn new(
        mapping: TextureMapping2Ds,
        outside: Arc<Textures<T, T>>,
        inside: Arc<Textures<T, T>>) -> Self {
        Self { mapping, outside, inside }
    }
}

impl<T: SpectrumT<T>> Texture<T> for DotsTexture<T>
{
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dstdx = Vector2f::default();
        let mut dstdy = Vector2f::default();
        let st = self.mapping.map(s, &mut dstdx, &mut dstdy);
        let (scell, tcell) = (
            (st.x + 0.5).floor() as usize,
            (st.y + 0.5).floor() as usize);

        // Return insideDot result if point is inside dot
        if noise(scell as Float + 0.5, tcell as Float + 0.5, 0.5) > 0.0 {
            let radius = 0.35;
            let max_shift = 0.5 - radius;
            let scenter =
                scell as Float + max_shift *
                noise(scell as Float + 1.5, tcell as Float + 2.8, 0.5);
            let tcenter =
                tcell as Float + max_shift *
                noise(scell as Float + 4.5, tcell as Float + 9.8, 0.5);
            let dst = st - Point2f::new(scenter, tcenter);

            if dst.length_squared() < radius * radius {
                return self.inside.evaluate(s);
            }
        }

        self.outside.evaluate(s)
    }
}

pub fn create_dots_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let map = get_mapping2d(t2w, tp);
    let inside = tp.get_floattexture("inside", 1.0);
    let outside = tp.get_floattexture("outside", 0.0);

    Some(Arc::new(DotsTexture::new(map, inside, outside).into()))
}

pub fn create_dots_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map = get_mapping2d(t2w, tp);
    let inside = tp.get_spectrumtexture("inside", Spectrum::new(1.0));
    let outside = tp.get_spectrumtexture("outside", Spectrum::new(0.0));

    Some(Arc::new(DotsTexture::new(map, inside, outside).into()))
}