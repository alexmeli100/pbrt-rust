use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use std::sync::Arc;
use crate::core::material::{Material, TransportMode, bump, Materials};
use bumpalo_herd::Member;
use crate::core::interaction::{SurfaceInteraction};
use crate::core::pbrt::INFINITY;
use crate::core::reflection::{BSDF, BxDFs, FresnelSpecular, Fresnels, FresnelDielectric, SpecularReflection, MicrofacetReflection, SpecularTransmission, MicrofacetTransmission};
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;
use crate::core::spectrum::Spectrum;

pub struct GlassMaterial {
    Kr              : Arc<TextureSpec>,
    Kt              : Arc<TextureSpec>,
    uroughness      : Arc<TextureFloat>,
    vroughness      : Arc<TextureFloat>,
    index           : Arc<TextureFloat>,
    bump_map        : Option<Arc<TextureFloat>>,
    remap_roughness : bool
}

impl GlassMaterial {
    pub fn new(
        Kr: Arc<TextureSpec>, Kt: Arc<TextureSpec>, uroughness: Arc<TextureFloat>,
        vroughness: Arc<TextureFloat>, index: Arc<TextureFloat>,
        bump_map : Option<Arc<TextureFloat>>, remap_roughness : bool) -> Self {
        Self {
            Kr, Kt, uroughness, vroughness,
            index, bump_map, remap_roughness
        }
    }
}

impl Material for GlassMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        _mat: Option<Arc<Materials>>, mode: TransportMode, allow_multiple_lobes: bool) {
        // Perform bump mapping with bump_map if present
        if let Some(ref map) = self.bump_map {
            bump(map, si);
        }

        let eta = self.index.evaluate(si);
        let mut urough = self.uroughness.evaluate(si);
        let mut vrough = self.vroughness.evaluate(si);
        let R = self.Kr.evaluate(si).clamps(0.0, INFINITY);
        let T = self.Kt.evaluate(si).clamps(0.0, INFINITY);


        let mut bsdf = BSDF::new(si, eta);

        if R.is_black() && T.is_black() { return; }

        let is_specular = urough == 0.0 && vrough == 0.0;

        if is_specular && allow_multiple_lobes {
            let bxdf: &mut BxDFs = arena.alloc(FresnelSpecular::new(&R, &T, 1.0, eta, mode).into());
            bsdf.add(bxdf)
        } else {
            if self.remap_roughness {
                urough = TrowbridgeReitzDistribution::roughness_to_alpha(urough);
                vrough = TrowbridgeReitzDistribution::roughness_to_alpha(vrough);
            }

            let distrib: &mut MicrofacetDistributions = arena.alloc(TrowbridgeReitzDistribution::new(urough, vrough, true).into());


            if !R.is_black() {
                let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.0, eta).into());
                let bxdf: &mut BxDFs = if is_specular {
                    arena.alloc(SpecularReflection::new(&R, fresnel).into())
                } else {
                    arena.alloc(MicrofacetReflection::new(&R, distrib, fresnel).into())
                };

                bsdf.add(bxdf)
            }

            if !T.is_black() {
                let bxdf: &mut BxDFs = if is_specular {
                    arena.alloc(SpecularTransmission::new(&T, 1.0, eta, mode).into())
                } else {
                    arena.alloc(MicrofacetTransmission::new(&T, distrib, 1.0, eta, mode).into())
                };

                bsdf.add(bxdf)
            }
        }

        si.bsdf = Some(bsdf);

    }
}

pub fn create_glass_material(mp: &mut TextureParams) -> Materials {
    let Kr = mp.get_spectrumtexture("Kr", Spectrum::new(1.0));
    let Kt = mp.get_spectrumtexture("Kt", Spectrum::new(1.0));
    let eta = match mp.get_floattexture_ornull("eta") {
        Some(t) => t,
        _       => mp.get_floattexture("index", 1.5)
    };
    let roughu = mp.get_floattexture("uroughness", 0.0);
    let roughv = mp.get_floattexture("vroughness", 0.0);
    let bump_map = mp.get_floattexture_ornull("bumpmap");
    let remap_roughness = mp.find_bool("remaproughness", true);

    let glass = GlassMaterial::new(Kr, Kt, roughu, roughv, eta, bump_map, remap_roughness);

    glass.into()
}