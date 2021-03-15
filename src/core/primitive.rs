use enum_dispatch::enum_dispatch;
use crate::core::geometry::bounds::Bounds3f;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{SurfaceInteraction};
use crate::core::material::Material;
use crate::core::material::{Materials, TransportMode};
use std::sync::Arc;
use crate::core::light::{Lights};
use bumpalo_herd::Member;
use crate::core::transform::{AnimatedTransform, Transform};
use crate::core::shape::{Shape, Shapes};
use crate::core::medium::MediumInterface;
use crate::accelerators::bvh::BVHAccel;
use crate::accelerators::kdtreeaccel::KdTreeAccel;

#[enum_dispatch]
pub trait Primitive {
    fn world_bound(&self) -> Bounds3f;
    fn intersect(
        &self, r: &mut Ray,
        s: &mut SurfaceInteraction,
        p: Arc<Primitives>) -> bool;

    fn intersect_p(&self, r: &mut Ray) -> bool;
    fn get_material(&self) -> Option<Arc<Materials>>;
    fn get_area_light(&self) -> Option<Arc<Lights>>;
    fn compute_scattering_functions<'b: 'b>(
        &self, isect: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
         mode: TransportMode, allow_multiple_lobes: bool);
}


#[enum_dispatch(Primitive)]
pub enum Primitives {
    BVHAccel,
    KdTreeAccel,
    TransformedPrimitive,
    GeometricPrimitive,
}

pub struct TransformedPrimitive {
    primitive       : Arc<Primitives>,
    prim_to_world   : AnimatedTransform

}

impl TransformedPrimitive {
    pub fn new(primitive: Arc<Primitives>, prim_to_world: AnimatedTransform) -> Self {
        Self { primitive, prim_to_world }
    }
}

impl Primitive for TransformedPrimitive {
    fn world_bound(&self) -> Bounds3f {
        self.prim_to_world.motion_bounds(&self.primitive.world_bound())
    }

    fn intersect(
        &self, r: &mut Ray,
        isect: &mut SurfaceInteraction,
        _p: Arc<Primitives>) -> bool {
        // Compute ray after transformation by PrimitiveToWorld
        let mut inter_prim_toworld = Transform::default();
        self.prim_to_world.interpolate(r.time, &mut inter_prim_toworld);
        let mut ray = Transform::inverse(&inter_prim_toworld).transform_ray(r);

        if !self.primitive.intersect(&mut ray, isect, self.primitive.clone()) {
            return false;
        }

        r.t_max = ray.t_max;

        // Transform instance's intersection data to world space
        if !inter_prim_toworld.is_identity() {
            *isect = inter_prim_toworld.transform_surface_interaction(isect);
        }

        assert!(isect.n.dot(&isect.shading.n) >= 0.0);
        true
    }

    fn intersect_p(&self, r: &mut Ray) -> bool {
        let mut inter_prim_toworld = Transform::default();
        self.prim_to_world.interpolate(r.time, &mut inter_prim_toworld);
        let inter_worldto_prim = Transform::inverse(&inter_prim_toworld);

        self.primitive.intersect_p(&mut inter_worldto_prim.transform_ray(r))
    }

    fn get_material(&self) -> Option<Arc<Materials>> {
        None
    }

    fn get_area_light(&self) -> Option<Arc<Lights>> {
        None
    }

    fn compute_scattering_functions<'b: 'b>(
        &self, _isect: &mut SurfaceInteraction<'b>, _arena: &Member<'b>,
        _mode: TransportMode, _allow_multiple_lobes: bool) {
        panic!("TransformedPrimitive::compute_scattering_functions() shouldn't be called")
    }
}

#[derive(Clone)]
pub struct GeometricPrimitive {
    shape               : Arc<Shapes>,
    material            : Option<Arc<Materials>>,
    area_light          : Option<Arc<Lights>>,
    medium_interface    : MediumInterface
}

impl GeometricPrimitive {
    pub fn new(
        shape: Arc<Shapes>, material: Option<Arc<Materials>>,
        area_light: Option<Arc<Lights>>, medium_interface: MediumInterface) -> Self {
        Self { shape, material, area_light, medium_interface }
    }
}

impl Primitive for GeometricPrimitive {
    fn world_bound(&self) -> Bounds3f {
        self.shape.world_bound()
    }

    fn intersect(
        &self, r: &mut Ray,
        isect: &mut SurfaceInteraction,
        p: Arc<Primitives>) -> bool {
        let mut thit = 0.0;

        let s = Some(self.shape.clone());
        if !self.shape.intersect(r, &mut thit, isect, true, s) {
            return false;
        }

        r.t_max = thit;
        isect.primitive = Some(p);
        assert!(isect.n.dot(&isect.shading.n) >= 0.0, "n: {}, shanding_n: {}", isect.n, isect.shading.n);

        let m = match self.medium_interface.is_medium_transition() {
            true => self.medium_interface.clone(),
            _    => MediumInterface::new(r.medium.clone())
        };

        isect.medium_interface = Some(m);

        true
    }

    fn intersect_p(&self, r: &mut Ray) -> bool {
        self.shape.intersect_p(r, true)
    }

    fn get_material(&self) -> Option<Arc<Materials>> {
        self.material.clone()
    }

    fn get_area_light(&self) -> Option<Arc<Lights>> {
        self.area_light.clone()
    }

    fn compute_scattering_functions<'b: 'b>(
        &self, isect: &mut SurfaceInteraction<'b>, arena: &Member<'b>,
         mode: TransportMode, allow_multiple_lobes: bool) {
        // TODO: ProfilePhase

        if let Some(m) = &self.material {
            m.compute_scattering_functions(isect, arena, Some(m.clone()), mode, allow_multiple_lobes);
        }

        assert!(isect.n.dot(&isect.shading.n) >= 0.0)
    }
}


