use enum_dispatch::enum_dispatch;
use crate::core::interaction::SurfaceInteraction;
use bumpalo_herd::Member;
use crate::core::texture::{Texture, TextureFloat};
use std::sync::Arc;
use std::fmt::{Display, Result, Formatter};
use crate::materials::matte::MatteMaterial;
use crate::materials::plastic::PlasticMaterial;
use crate::materials::fourier::FourierMaterial;
use crate::materials::mix::MixMaterial;
use crate::materials::glass::GlassMaterial;
use crate::materials::mirror::MirrorMaterial;
use crate::materials::hair::HairMaterial;
use crate::materials::metal::MetalMaterial;
use crate::materials::translucent::TranslucentMaterial;
use crate::materials::substrate::SubstrateMaterial;
use crate::materials::uber::UberMaterial;
use crate::materials::kdsubsurface::KdSubsurfaceMaterial;
use crate::materials::subsurface::SubsurfaceMaterial;
use crate::materials::disney::DisneyMaterial;
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::geometry::normal::Normal3f;

#[derive(Debug, Eq, PartialEq, Copy, Clone)]
pub enum TransportMode {
    Radiance,
    Importance
}

impl Display for TransportMode {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        match self {
            TransportMode::Importance => write!(f, "Importance"),
            TransportMode::Radiance   => write!(f, "Radiance")
        }
    }
}

#[enum_dispatch(Materials)]
pub trait Material {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
        mat: Option<Arc<Materials>>, mode: TransportMode, allow_multiple_lobes: bool);
}

pub fn bump(d: &Arc<TextureFloat>, si: &mut SurfaceInteraction) {
    // Compute offset positions and evaluete displacement texture
    let mut si_eval = si.clone();

    // Shift si_eval du in the u direction
    let mut du = 0.5 * (si.dudx.get().abs() + si.dudy.get().abs());

    // The most common reason for du to be zero is for ray that start from
    // light sources, where no differentials are available. In this case,
    // we try to choose a small enough du so that we still get a decently
    // accurate bump value.
    if du == 0.0 { du = 0.0005; }
    si_eval.p = si.p + si.shading.dpdu * du;
    si_eval.uv = si.uv + Vector2f::new(du, 0.0);
    si_eval.n = Normal3f::from(si.shading.dpdu.cross(&si.shading.dpdv)).normalize() + si.dndu * du;
    let udisplace = d.evaluate(&si_eval);

    // Shift si_eval dv in the v direction
    let mut dv = 0.5 * (si.dvdx.get().abs() + si.dvdy.get().abs());
    if dv == 0.0 { dv = 0.0005; }

    si_eval.p = si.p + si.shading.dpdv * dv;
    si_eval.uv = si.uv + Vector2f::new(0.0, dv);
    si_eval.n = Normal3f::from(si.shading.dpdu.cross(&si.shading.dpdv)).normalize() + si.dndv * dv;
    let vdisplace = d.evaluate(&si_eval);
    let displace = d.evaluate(si);

    // Compute bump-mapped differential geometry
    let dpdu =
        si.shading.dpdu +
        Vector3f::from(si.shading.n) * ((udisplace - displace) / du) +
        Vector3f::from(si.shading.dndu) * displace;
    let dpdv =
        si.shading.dpdv +
        Vector3f::from(si.shading.n) * ((vdisplace - displace) / dv) +
        Vector3f::from(si.shading.dndv) * displace;
    let dndu = si.shading.dndu;
    let dndv = si.shading.dndv;

    si.set_shading_geometry(&dpdu, &dpdv, &dndu, &dndv, false);

}

#[enum_dispatch]
pub enum Materials {
    MixMaterial,
    HairMaterial,
    MetalMaterial,
    UberMaterial,
    GlassMaterial,
    MatteMaterial,
    MirrorMaterial,
    PlasticMaterial,
    FourierMaterial,
    DisneyMaterial,
    SubstrateMaterial,
    SubsurfaceMaterial,
    TranslucentMaterial,
    KdSubsurfaceMaterial
}