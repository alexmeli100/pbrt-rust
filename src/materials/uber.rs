use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, Materials, TransportMode, bump};
use crate::core::interaction::SurfaceInteraction;
use crate::core::pbrt::INFINITY;
use crate::core::spectrum::Spectrum;
use crate::core::reflection::{BSDF, BxDFs, SpecularTransmission, LambertianReflection, MicrofacetReflection, Fresnels, FresnelDielectric, SpecularReflection};
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;
use bumpalo_herd::Member;

pub struct UberMaterial {
    kd              : Arc<TextureSpec>,
    ks              : Arc<TextureSpec>,
    kr              : Arc<TextureSpec>,
    kt              : Arc<TextureSpec>,
    opacity         : Arc<TextureSpec>,
    roughness       : Arc<TextureFloat>,
    roughnessu      : Option<Arc<TextureFloat>>,
    roughnessv      : Option<Arc<TextureFloat>>,
    eta             : Arc<TextureFloat>,
    bumpmap         : Option<Arc<TextureFloat>>,
    remap_roughness : bool,
}

impl UberMaterial {
    pub fn new(
        kd: Arc<TextureSpec>, ks: Arc<TextureSpec>, kr: Arc<TextureSpec>,
        kt: Arc<TextureSpec>, opacity: Arc<TextureSpec>, roughness: Arc<TextureFloat>,
        roughnessu: Option<Arc<TextureFloat>>, roughnessv: Option<Arc<TextureFloat>>,
        eta: Arc<TextureFloat>, bumpmap : Option<Arc<TextureFloat>>, remap_roughness: bool) -> Self {
        Self {
            kd, ks, kr, kt, opacity, roughness, roughnessu,
            roughnessv, eta, bumpmap, remap_roughness
        }
    }
}

impl Material for UberMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        let e = self.eta.evaluate(si);
        let op = self.opacity.evaluate(si).clamps(0.0, INFINITY);
        let t = (-op + Spectrum::new(1.0)).clamps(0.0, INFINITY);

        let mut bsdf = if !t.is_black() {
            let mut bsdf = BSDF::new(si, 1.0);
            let tr: &mut BxDFs = arena.alloc(SpecularTransmission::new(&t, 1.0,1.0, mode).into());
            bsdf.add(tr);

            bsdf
        } else {
            BSDF::new(si, e)
        };

        let kd = op * self.kd.evaluate(si).clamps(0.0, INFINITY);
        if !kd.is_black() {
            let diff: &mut BxDFs = arena.alloc(LambertianReflection::new(&kd).into());
            bsdf.add(diff);
        }

        let ks = op * self.ks.evaluate(si).clamps(0.0, INFINITY);
        if !ks.is_black() {
            let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.0, e).into());
            let mut roughu = if let Some(ref ru) = self.roughnessu {
                ru.evaluate(si)
            } else {
                self.roughness.evaluate(si)
            };
            let mut roughv = if let Some(ref rv) = self.roughnessv {
                rv.evaluate(si)
            } else {
                self.roughness.evaluate(si)
            };

            if self.remap_roughness {
                roughu = TrowbridgeReitzDistribution::roughness_to_alpha(roughu);
                roughv = TrowbridgeReitzDistribution::roughness_to_alpha(roughv);
            }

            let distrib: &mut MicrofacetDistributions = arena.alloc(TrowbridgeReitzDistribution::new(roughu, roughv, true).into());
            let spec: &mut BxDFs = arena.alloc(MicrofacetReflection::new(&ks, distrib, fresnel).into());
            bsdf.add(spec)
        }

        let kr = op * self.kr.evaluate(si).clamps(0.0, INFINITY);
        if !kr.is_black() {
            let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.0, e).into());
            let bxdf: &mut BxDFs = arena.alloc(SpecularReflection::new(&kr, fresnel).into());
            bsdf.add(bxdf);
        }

        let kt = op * self.kt.evaluate(si).clamps(0.0, INFINITY);
        if !kt.is_black() {
            let bxdf: &mut BxDFs = arena.alloc(SpecularTransmission::new(&kt, 1.0, e, mode).into());
            bsdf.add(bxdf);
        }

        si.bsdf = Some(bsdf);
    }
}

pub fn create_uber_material(mp: &mut TextureParams) -> Materials {
    let kd = mp.get_spectrumtexture("Kd", Spectrum::new(0.25));
    let ks = mp.get_spectrumtexture("Ks", Spectrum::new(0.25));
    let kr = mp.get_spectrumtexture("Kr", Spectrum::new(0.0));
    let kt = mp.get_spectrumtexture("Kt", Spectrum::new(0.0));
    let roughness = mp.get_floattexture("roughness", 0.1);
    let urough = mp.get_floattexture_ornull("uroughness");
    let vrough = mp.get_floattexture_ornull("vroughness");
    let eta = if let Some(e) = mp.get_floattexture_ornull("eta") {
        e
    } else {
        mp.get_floattexture("index", 1.5)
    };
    let opacity = mp.get_spectrumtexture("opacity", Spectrum::new(1.0));
    let bumpmap = mp.get_floattexture_ornull("bumpmap");
    let remap_roughness = mp.find_bool("remaproughness", true);

    UberMaterial::new(
        kd, ks, kr, kt, opacity, roughness, urough,
        vrough, eta, bumpmap, remap_roughness).into()
}