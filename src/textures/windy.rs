use crate::core::texture::{TextureMapping3D, TextureMapping3Ds, Texture, fbm, TextureFloat, IdentityMapping3D, TextureSpec};
use crate::core::pbrt::Float;
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;
use std::sync::Arc;

pub struct WindyTexture {
    mapping: TextureMapping3Ds
}

impl WindyTexture {
    pub fn new(mapping: TextureMapping3Ds) -> Self {
        Self { mapping }
    }
}

impl<T: From<Float>> Texture<T> for WindyTexture  {
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dpdx = Vector3f::default();
        let mut dpdy = Vector3f::default();
        let p = self.mapping.map(s, &mut dpdx, &mut dpdy);

        let wstrength = fbm(&(p * 0.1), &(dpdx * 0.1), &(dpdy * 0.1), 0.5, 3 );
        let wheight = fbm(&p, &dpdx, &dpdy, 0.5, 6);

        T::from((wstrength * wheight).abs())
    }
}

pub fn create_windy_float(t2w: &Transform, _tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let map: TextureMapping3Ds = IdentityMapping3D::new(t2w).into();

    Some(Arc::new(WindyTexture::new(map).into()))
}

pub fn create_windy_spectrum(t2w: &Transform, _tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map: TextureMapping3Ds = IdentityMapping3D::new(t2w).into();

   Some(Arc::new(WindyTexture::new(map).into()))
}