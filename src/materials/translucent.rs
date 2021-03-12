use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, Materials, TransportMode, bump};
use crate::core::interaction::SurfaceInteraction;
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;
use crate::core::reflection::{BSDF, BxDFs, LambertianReflection, LambertianTransmission, Fresnels, FresnelDielectric, MicrofacetReflection, MicrofacetTransmission};
use crate::core::pbrt::INFINITY;
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use bumpalo_herd::Member;

pub struct TranslucentMaterial {
    kd              : Arc<TextureSpec>,
    ks              : Arc<TextureSpec>,
    roughness       : Arc<TextureFloat>,
    reflect         : Arc<TextureSpec>,
    transmit        : Arc<TextureSpec>,
    bumpmap         : Option<Arc<TextureFloat>>,
    remap_roughness : bool
}

impl TranslucentMaterial {
    pub fn new(
        kd: Arc<TextureSpec>, ks: Arc<TextureSpec>, roughness: Arc<TextureFloat>,
        reflect: Arc<TextureSpec>, transmit: Arc<TextureSpec>,
        bumpmap: Option<Arc<TextureFloat>>, remap_roughness: bool) -> Self {
        Self { kd, ks, roughness, reflect, transmit, bumpmap, remap_roughness }
    }
}

impl Material for TranslucentMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        let eta = 1.5;
        let mut bsdf = BSDF::new(si, eta);

        let r = self.reflect.evaluate(si).clamps(0.0, INFINITY);
        let t = self.transmit.evaluate(si).clamps(0.0, INFINITY);

        if r.is_black() && t.is_black() { return; }

        let kd = self.kd.evaluate(si).clamps(0.0, INFINITY);
        if !kd.is_black() {
            if !r.is_black() {
                bsdf.add(arena.alloc(LambertianReflection::new(&(r * kd)).into()))
            }
            if !t.is_black() {
                bsdf.add(arena.alloc(LambertianTransmission::new(&(t * kd)).into()))
            }
        }

        let ks = self.ks.evaluate(si).clamps(0.0, INFINITY);
        if !ks.is_black() && (!r.is_black() || !t.is_black()) {
            let mut rough = self.roughness.evaluate(si);

            if self.remap_roughness {
                rough = TrowbridgeReitzDistribution::roughness_to_alpha(rough)
            }

            let distrib: &mut MicrofacetDistributions = arena.alloc(TrowbridgeReitzDistribution::new(rough, rough, true).into());

            if !r.is_black() {
                let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.0, eta).into());
                let bxdf: &mut BxDFs = arena.alloc(MicrofacetReflection::new(&(r * ks), distrib, fresnel).into());
                bsdf.add(bxdf);
            }

            if !t.is_black() {
                let bxdf: &mut BxDFs = arena.alloc(MicrofacetTransmission::new(&(t * ks), distrib, 1.0, eta, mode).into());
                bsdf.add(bxdf);
            }
        }

        si.bsdf = Some(bsdf);
    }
}

pub fn create_translucent_material(mp: &mut TextureParams) -> Materials {
    let kd = mp.get_spectrumtexture("Kd", Spectrum::new(0.25));
    let ks = mp.get_spectrumtexture("Ks", Spectrum::new(0.25));
    let reflect = mp.get_spectrumtexture("reflect", Spectrum::new(0.5));
    let transmit = mp.get_spectrumtexture("transmit", Spectrum::new(0.5));
    let roughness = mp.get_floattexture("roughness", 0.1);
    let bumpmap = mp.get_floattexture_ornull("bumpmap");
    let remap = mp.find_bool("remaproughness", true);

    TranslucentMaterial::new(kd, ks, roughness, reflect, transmit, bumpmap, remap).into()
}