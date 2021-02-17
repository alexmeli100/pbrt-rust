use crate::core::material::{Materials, Material, TransportMode};
use std::sync::Arc;
use crate::core::texture::{Texture, TextureSpec};
use crate::core::spectrum::Spectrum;
use crate::core::interaction::SurfaceInteraction;
use crate::core::pbrt::INFINITY;
use crate::core::reflection::{BxDFType, ScaledBxDF};
use crate::core::paramset::TextureParams;
use bumpalo_herd::Member;

pub struct MixMaterial {
    m1      : Arc<Materials>,
    m2      : Arc<Materials>,
    scale   : Arc<TextureSpec>
}

impl MixMaterial {
    pub fn new(
        m1: Arc<Materials>, m2: Arc<Materials>,
        scale: Arc<TextureSpec>) -> Self {
        Self { m1, m2, scale }
    }
}

impl Material for MixMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: & Member<'b>,
        _mat: Option<Arc<Materials>>, mode: TransportMode, allow_multiple_lobes: bool) {
        let s1 = self.scale.evaluate(si).clamps(0.0, INFINITY);
        let s2 = (Spectrum::new(1.0) - s1).clamps(0.0, INFINITY);
        let mut si2 = SurfaceInteraction::new(
            &si.p, &si.p_error, &si.uv, &si.wo, &si.dpdu, &si.dpdv,
            &si.dndu, &si.dndv, si.time, si.shape.clone());

        self.m1.compute_scattering_functions(si, arena, Some(self.m1.clone()), mode, allow_multiple_lobes);
        self.m2.compute_scattering_functions(&mut si2, arena, Some(self.m2.clone()), mode, allow_multiple_lobes);

        let b1 = si.bsdf.as_mut().unwrap();
        let b2 = si2.bsdf.as_mut().unwrap();
        let n1 = b1.num_components(BxDFType::All as u8);
        let n2 = b2.num_components(BxDFType::All as u8);

        for i in 0..n1 {
            b1.bxdfs[i] = arena.alloc(ScaledBxDF::new(b1.bxdfs[i], &s1).into());
        }

        for i in 0..n2 {
            b1.add(arena.alloc(ScaledBxDF::new(b2.bxdfs[i], &s2).into()));
        }
    }
}

pub fn create_mix_material(mp: &mut TextureParams, m1: Arc<Materials>, m2: Arc<Materials>) -> Materials {
    let scale = mp.get_spectrumtexture("amount", Spectrum::new(0.5));

    MixMaterial::new(m1, m2, scale).into()
}