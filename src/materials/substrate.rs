use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, TransportMode, bump, Materials};
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, FresnelBlend, BxDFs};
use crate::core::pbrt::INFINITY;
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;
use bumpalo_herd::Member;

pub struct SubstrateMaterial {
    kd              : Arc<TextureSpec>,
    ks              : Arc<TextureSpec>,
    nu              : Arc<TextureFloat>,
    nv              : Arc<TextureFloat>,
    bumpmap         : Option<Arc<TextureFloat>>,
    remap_roughness : bool
}

impl SubstrateMaterial {
    pub fn new(
        kd: Arc<TextureSpec>,
        ks: Arc<TextureSpec>,
        nu: Arc<TextureFloat>,
        nv: Arc<TextureFloat>,
        bumpmap: Option<Arc<TextureFloat>>,
        remap_roughness: bool) -> Self {
        Self { kd, ks, nu, nv, bumpmap, remap_roughness }
    }
}

impl Material for SubstrateMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, _mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        let bsdf = arena.alloc(BSDF::new(si, 1.0));
        let d = self.kd.evaluate(si).clamps(0.0, INFINITY);
        let s = self.ks.evaluate(si).clamps(0.0, INFINITY);
        let mut roughu = self.nu.evaluate(si);
        let mut roughv = self.nv.evaluate(si);

        if !d.is_black() || !s.is_black() {
            if self.remap_roughness {
                roughu = TrowbridgeReitzDistribution::roughness_to_alpha(roughu);
                roughv = TrowbridgeReitzDistribution::roughness_to_alpha(roughv)
            }

            let distrib: &mut MicrofacetDistributions =
                arena.alloc(TrowbridgeReitzDistribution::new(roughu, roughv, true).into());
            let bxdf: &mut BxDFs = arena.alloc(FresnelBlend::new(&d, &s, distrib).into());
            bsdf.add(bxdf);
            si.bsdf = Some(bsdf);
        }
    }
}

pub fn create_substrate_material(mp: &mut TextureParams) -> Materials {
    let kd = mp.get_spectrumtexture("Kd", Spectrum::new(0.5));
    let ks = mp.get_spectrumtexture("Ks", Spectrum::new(0.5));
    let urough = mp.get_floattexture("uroughness", 0.1);
    let vrough = mp.get_floattexture("vroughness", 0.1);
    let bumpmap = mp.get_floattexture_ornull("bumpmap");
    let remap_roughness = mp.find_bool("remaproughness", true);

    SubstrateMaterial::new(kd, ks, urough, vrough, bumpmap, remap_roughness).into()
}