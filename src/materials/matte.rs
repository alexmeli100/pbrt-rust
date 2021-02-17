use std::sync::Arc;
use crate::core::texture::{Texture, TextureSpec, TextureFloat};
use crate::core::spectrum::Spectrum;
use crate::core::pbrt::{clamp, INFINITY};
use crate::core::material::{Material, TransportMode, bump, Materials};
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, BxDFs, OrenNayar};
use crate::core::reflection::{LambertianReflection};
use crate::core::paramset::TextureParams;
use bumpalo_herd::Member;

pub struct MatteMaterial {
    kd      : Arc<TextureSpec>,
    sigma   : Arc<TextureFloat>,
    bump_map: Option<Arc<TextureFloat>>
}

impl MatteMaterial {
    pub fn new(
        kd: Arc<TextureSpec>,
        sigma: Arc<TextureFloat>,
        bump_map: Option<Arc<TextureFloat>>) -> Self {
        Self { kd, sigma, bump_map }
    }
}

impl Material for MatteMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, _mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpMap if present
        if self.bump_map.is_some() { bump(self.bump_map.as_ref().unwrap(), si); }

        // Evaluate textures for MatteMaterial and allocate BRDF
        let bsdf = arena.alloc(BSDF::new(si, 1.0));
        let r = self.kd.evaluate(si).clamps(0.0, INFINITY);
        let sig = clamp(self.sigma.evaluate(si), 0.0 , 90.0);

        if !r.is_black() {
            let bxdf: &mut BxDFs = if sig == 0.0 {
                arena.alloc(LambertianReflection::new(&r).into())
            } else {
                arena.alloc(OrenNayar::new(&r, sig).into())
            };

            bsdf.add(bxdf);
        }

        si.bsdf = Some(bsdf);
    }
}

pub fn create_matte_material(mp: &mut TextureParams) -> Materials {
    let kd = mp.get_spectrumtexture("Kd", Spectrum::new(0.5));
    let sigma = mp.get_floattexture("sigma", 0.0);
    let bump_map = mp.get_floattexture_ornull("bumpmap");

    MatteMaterial::new(kd, sigma, bump_map).into()

}