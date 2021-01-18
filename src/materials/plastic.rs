use std::sync::Arc;
use crate::core::texture::{Texture, TextureSpec, TextureFloat};
use crate::core::spectrum::Spectrum;
use crate::core::pbrt::{INFINITY};
use crate::core::material::{Material, TransportMode, bump, Materials};
use bumpalo::Bump;
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, Fresnels, MicrofacetReflection, BxDFs};
use crate::core::reflection::{LambertianReflection};
use crate::core::reflection::FresnelDielectric;
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;


pub struct PlasticMaterial {
    kd: Arc<TextureSpec>,
    ks: Arc<TextureSpec>,
    roughness: Arc<TextureFloat>,
    bump_map: Option<Arc<TextureFloat>>,
    remap_roughness: bool

}

impl PlasticMaterial {
    pub fn new(
        kd: Arc<TextureSpec>, ks: Arc<TextureSpec>,
        roughness: Arc<TextureFloat>, bump_map: Option<Arc<TextureFloat>>,
        remap_roughness: bool) -> Self {
        Self { kd, ks, roughness, bump_map, remap_roughness }
    }
}

impl Material for PlasticMaterial {
    fn compute_scattering_functions<'a: 'a>(
        &self, si: &mut SurfaceInteraction<'a>, arena: &'a Bump,
        _mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if self.bump_map.is_some() { bump(self.bump_map.as_ref().unwrap(), si); }
        let bsdf = arena.alloc(BSDF::new(si, 1.0));

        // Initialize diffuse component of plastic material
        let kd = self.kd.evaluate(si).clamps(0.0, INFINITY);

        if !kd.is_black() {
            bsdf.add(arena.alloc(LambertianReflection::new(&kd).into()));
        }

        // Initialize specular component of plastic material
        let ks = self.ks.evaluate(si).clamps(0.0, INFINITY);

        if !ks.is_black() {
            let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.5, 1.5).into());

            // Create microfacet distribution distrib for plastic material
            let mut rough = self.roughness.evaluate(si);

            if self.remap_roughness {
                rough = TrowbridgeReitzDistribution::roughness_to_alpha(rough);
            }

            let distrib: &mut MicrofacetDistributions =
                arena.alloc(TrowbridgeReitzDistribution::new(rough, rough, true).into());
            let spec: &mut BxDFs =
                arena.alloc(MicrofacetReflection::new(&ks, distrib, fresnel).into());
            bsdf.add(spec);
        }

        si.bsdf = Some(bsdf);
    }
}

pub fn create_plastic_material(mp: &mut TextureParams) -> Materials {
    let kd = mp.get_spectrumtexture("Kd", Spectrum::new(0.25));
    let ks = mp.get_spectrumtexture("Ks", Spectrum::new(0.25));
    let roughness = mp.get_floattexture("roughness", 0.1);
    let bump_map = mp.get_floattexture_ornull("bumpmap");
    let remap_roughness = mp.find_bool("remaproughness", true);

    PlasticMaterial::new(kd, ks, roughness, bump_map, remap_roughness).into()

}