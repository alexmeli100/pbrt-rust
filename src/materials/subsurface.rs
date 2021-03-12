use crate::core::pbrt::{Float, INFINITY};
use std::sync::Arc;
use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use crate::core::bssrdf::{BSSRDFTable, compute_beam_diffusion_bssrdf, TabulatedBSSRDF};
use crate::core::material::{Material, Materials, TransportMode, bump};
use log::warn;
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, FresnelSpecular, BxDFs, Fresnels, FresnelDielectric, SpecularReflection, MicrofacetReflection, SpecularTransmission, MicrofacetTransmission};
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;
use crate::core::spectrum::{Spectrum, SpectrumType};
use crate::core::medium::get_medium_scattering_properties;
use bumpalo_herd::Member;

pub struct SubsurfaceMaterial {
    scale           : Float,
    kr              : Arc<TextureSpec>,
    kt              : Arc<TextureSpec>,
    sigma_a         : Arc<TextureSpec>,
    sigma_s         : Arc<TextureSpec>,
    uroughness      : Arc<TextureFloat>,
    vroughness      : Arc<TextureFloat>,
    bumpmap         : Option<Arc<TextureFloat>>,
    eta             : Float,
    remap_roughness : bool,
    table           : Arc<BSSRDFTable>
}

impl SubsurfaceMaterial {
    pub fn new(
        scale: Float, kr: Arc<TextureSpec>, kt: Arc<TextureSpec>, 
        sigma_a: Arc<TextureSpec>, sigma_s: Arc<TextureSpec>, g: Float, 
        eta: Float, uroughness: Arc<TextureFloat>, vroughness: Arc<TextureFloat>,
        bumpmap: Option<Arc<TextureFloat>>, remap_roughness: bool) -> Self {
        let mut table = BSSRDFTable::new(100, 64);
        compute_beam_diffusion_bssrdf(g, eta, &mut table);

        Self {
            scale, kr, kt, sigma_a, sigma_s, eta, uroughness,
            vroughness, bumpmap, remap_roughness,
            table: Arc::new(table)
        }
    }
}

impl Material for SubsurfaceMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        mat: Option<Arc<Materials>>, mode: TransportMode, allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        // Initialize BSDF for SubsurfaceMaterial
        let R = self.kr.evaluate(si).clamps(0.0, INFINITY);
        let T = self.kt.evaluate(si).clamps(0.0, INFINITY);
        let mut urough = self.uroughness.evaluate(si);
        let mut vrough = self.vroughness.evaluate(si);

        // Initalize BSDF for msooth or rough dielectric
        let mut bsdf = BSDF::new(si, self.eta);

        if R.is_black() && T.is_black() { return; }

        let is_specular = urough == 0.0 && vrough == 0.0;

        if is_specular && allow_multiple_lobes {
            let bxdf: &mut BxDFs = arena.alloc(FresnelSpecular::new(&R, &T, 1.0, self.eta, mode).into());
            bsdf.add(bxdf);
        } else {
            if self.remap_roughness {
                urough = TrowbridgeReitzDistribution::roughness_to_alpha(urough);
                vrough = TrowbridgeReitzDistribution::roughness_to_alpha(vrough);
            }

            let distrib: &mut MicrofacetDistributions = arena.alloc(TrowbridgeReitzDistribution::new(urough, vrough, true).into());

            if !R.is_black() {
                let fresnel: &mut Fresnels = arena.alloc(FresnelDielectric::new(1.0, self.eta).into());

                let bxdf: &mut BxDFs = if is_specular {
                    arena.alloc(SpecularReflection::new(&R, fresnel).into())
                } else {
                    arena.alloc(MicrofacetReflection::new(&R, distrib, fresnel).into())
                };
                bsdf.add(bxdf)
            }

            if !T.is_black() {
                let bxdf: &mut BxDFs = if is_specular {
                    arena.alloc(SpecularTransmission::new(&T, 1.0, self.eta, mode).into())
                } else {
                    arena.alloc(MicrofacetTransmission::new(&T, distrib, 1.0, self.eta, mode).into())
                };
                bsdf.add(bxdf)
            }
        }

        si.bsdf = Some(bsdf);
        let siga = self.sigma_a.evaluate(si).clamps(0.0, INFINITY) * self.scale;
        let sigs = self.sigma_s.evaluate(si).clamps(0.0, INFINITY) * self.scale;
        let bssrdf = TabulatedBSSRDF::new(si, mat, mode, self.eta, &siga, &sigs, self.table.clone()).into();

        si.bssrdf = Some(bssrdf)
    }
}

pub fn create_subsurface_material(mp: &mut TextureParams) -> Materials {
    let mut siga = Spectrum::from_rgb([0.0011, 0.0024, 0.014], SpectrumType::Reflectance);
    let mut sigs = Spectrum::from_rgb([2.55, 3.21, 3.77], SpectrumType::Reflectance);
    let name = mp.find_string("name", "");
    let found = get_medium_scattering_properties(&name, &mut siga, &mut sigs);
    let mut g = mp.find_float("g", 0.0);

    if !name.is_empty() {
        if !found {
            warn!("Named material \"{}\" not found. using defaults.", name);
        } else {
            g = 0.0;
        }
    }
    
    let scale = mp.find_float("scale", 1.0);
    let eta = mp.find_float("eta", 1.33);
    
    let sigma_a = mp.get_spectrumtexture("sigma_a", siga);
    let sigma_s = mp.get_spectrumtexture("sigma_s", sigs);
    let kr = mp.get_spectrumtexture("Kr", Spectrum::new(1.0));
    let kt = mp.get_spectrumtexture("Kt", Spectrum::new(1.0));
    let roughu = mp.get_floattexture("uroughness", 0.0);
    let roughv = mp.get_floattexture("vroughness", 0.0);
    let bumpmap = mp.get_floattexture_ornull("bumpmap");
    let remap_roughness = mp.find_bool("remaproughness", true);
    
    SubsurfaceMaterial::new(
        scale, kr, kt, sigma_a, sigma_s, g, eta, 
        roughu, roughv, bumpmap, remap_roughness).into()
    

}