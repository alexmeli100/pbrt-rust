use crate::core::pbrt::{Float, INFINITY};
use std::sync::Arc;
use crate::core::texture::{TextureSpec, TextureFloat, Texture};
use crate::core::bssrdf::{BSSRDFTable, subsurface_from_diffuse, TabulatedBSSRDF, compute_beam_diffusion_bssrdf};
use crate::core::material::{Material, TransportMode, bump, Materials};
use bumpalo_herd::Member;
use crate::core::interaction::SurfaceInteraction;
use crate::core::reflection::{BSDF, BxDFs, FresnelSpecular, FresnelDielectric, Fresnels, SpecularReflection, MicrofacetReflection, SpecularTransmission, MicrofacetTransmission};
use crate::core::microfacet::{TrowbridgeReitzDistribution, MicrofacetDistributions};
use crate::core::paramset::TextureParams;
use crate::core::spectrum::{Spectrum, SpectrumType};

pub struct KdSubsurfaceMaterial {
    scale           : Float,
    kd              : Arc<TextureSpec>,
    kr              : Arc<TextureSpec>,
    kt              : Arc<TextureSpec>,
    mfp             : Arc<TextureSpec>,
    uroughness      : Arc<TextureFloat>,
    vroughness      : Arc<TextureFloat>,
    bumpmap         : Option<Arc<TextureFloat>>,
    eta             : Float,
    remap_roughness : bool,
    table           : Arc<BSSRDFTable>
}

impl KdSubsurfaceMaterial {
    pub fn new(
        scale: Float, kd: Arc<TextureSpec>, kr: Arc<TextureSpec>,
        kt: Arc<TextureSpec>, mfp: Arc<TextureSpec>, uroughness: Arc<TextureFloat>,
        vroughness: Arc<TextureFloat>, bumpmap: Option<Arc<TextureFloat>>, eta: Float,
        g: Float, remap_roughness: bool) -> Self {
        let mut table = BSSRDFTable::new(100, 64);
        compute_beam_diffusion_bssrdf(g, eta, &mut table);

        Self {
            scale, kd, kr, kt, mfp, uroughness,
            vroughness, bumpmap, eta, remap_roughness,
            table: Arc::new(table)
        }
    }
}

impl Material for KdSubsurfaceMaterial {
    fn compute_scattering_functions<'b:'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        mat: Option<Arc<Materials>>, mode: TransportMode, allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpmap if present
        if let Some(ref map) = self.bumpmap {
            bump(map, si);
        }

        let R = self.kr.evaluate(si).clamps(0.0, INFINITY);
        let T = self.kt.evaluate(si).clamps(0.0, INFINITY);
        let mut urough = self.uroughness.evaluate(si);
        let mut vrough = self.vroughness.evaluate(si);

        // Initialize bsdf for smooth or rough dielectric
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

                bsdf.add(
                    if is_specular {
                        arena.alloc(SpecularReflection::new(&R, fresnel).into())
                    } else {
                        arena.alloc(MicrofacetReflection::new(&R, distrib, fresnel).into())
                    }
                )
            }

            if !T.is_black() {
                bsdf.add(
                    if is_specular {
                        arena.alloc(SpecularTransmission::new(&T, 1.0, self.eta, mode).into())
                    } else {
                        arena.alloc(MicrofacetTransmission::new(&T, distrib, 1.0, self.eta, mode).into())
                    }
                )
            }
        }

        si.bsdf = Some(bsdf);

        let mfree = self.mfp.evaluate(si).clamps(0.0, INFINITY) * self.scale;
        let kd = self.kd.evaluate(si).clamps(0.0, INFINITY).clamps(0.0, INFINITY);
        let (sigma_a, sigma_s) = subsurface_from_diffuse(&self.table, &kd, &mfree);
        let bssrdf = TabulatedBSSRDF::new(si, mat, mode, self.eta, &sigma_a, &sigma_s, self.table.clone()).into();

        si.bssrdf = Some(bssrdf)
    }
}

pub fn create_kdsubsurface_material(mp: &mut TextureParams) -> Materials {
    let Kd = [0.5, 0.5, 0.5];
    let kd = mp.get_spectrumtexture("Kd", Spectrum::from_rgb(Kd, SpectrumType::Reflectance));
    let mfp = mp.get_spectrumtexture("mfp", Spectrum::new(1.0));
    let kr = mp.get_spectrumtexture("Kr", Spectrum::new(1.0));
    let kt = mp.get_spectrumtexture("Kt", Spectrum::new(1.0));
    let roughu = mp.get_floattexture("uroughness", 0.0);
    let roughv = mp.get_floattexture("vroughness", 0.0);
    let bumpmap = mp.get_floattexture_ornull("bumpmap");
    let eta = mp.find_float("eta", 1.33);
    let scale = mp.find_float("scale", 1.0);
    let g = mp.find_float("g", 0.0);
    let remap_roughness = mp.find_bool("remaproughness", true);

    KdSubsurfaceMaterial::new(
        scale, kd, kr, kt, mfp, roughu,
        roughv, bumpmap, eta, g, remap_roughness).into()
}