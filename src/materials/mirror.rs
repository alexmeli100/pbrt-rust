use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, TransportMode, bump, Materials};
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, BxDFs, SpecularReflection, Fresnels, FresnelNoOp};
use crate::core::pbrt::INFINITY;
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;
use bumpalo_herd::Member;

pub struct MirrorMaterial {
    Kr       : Arc<TextureSpec>,
    bump_map : Option<Arc<TextureFloat>>
}

impl MirrorMaterial {
    pub fn new(Kr: Arc<TextureSpec>, bump_map: Option<Arc<TextureFloat>>) -> Self {
        Self { Kr, bump_map }
    }
}

impl Material for MirrorMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, _mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bump_map if present
        if let Some(ref map) = self.bump_map {
            bump(map, si);
        }

        let bsdf: &mut BSDF = arena.alloc(BSDF::new(si, 1.0));
        let R = self.Kr.evaluate(si).clamps(0.0, INFINITY);

        if !R.is_black() {
            let fresnel: &mut Fresnels = arena.alloc(FresnelNoOp().into());
            let bxdf: &mut BxDFs = arena.alloc(SpecularReflection::new(&R, fresnel).into());
            bsdf.add(bxdf);
        }

        si.bsdf = Some(bsdf)
    }
}

pub fn create_mirror_material(mp: &mut TextureParams) -> Materials {
    let Kr = mp.get_spectrumtexture("Kr", Spectrum::new(0.9));
    let bump_map = mp.get_floattexture_ornull("bumpmap");
    let mirror = MirrorMaterial::new(Kr, bump_map);

    mirror.into()
}

