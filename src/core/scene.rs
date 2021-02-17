use bumpalo::core_alloc::sync::Arc;
use crate::core::light::{Lights, LightFlags};
use crate::core::primitive::Primitives;
use crate::core::geometry::bounds::Bounds3f;
use crate::core::medium::Medium;
use crate::core::light::Light;
use crate::core::primitive::Primitive;
use crate::core::sampler::Samplers;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{SurfaceInteraction, Interaction};
use crate::core::spectrum::Spectrum;
use crate::core::geometry::vector::Vector3f;

stat_counter!("Intersections/Regular ray intersection tests", nintersection_tests);
stat_counter!("Intersections/Shadow ray intersection tests", nshadow_tests);

pub fn init_stats() {
    nintersection_tests::init();
    nshadow_tests::init();
}

#[derive(Clone)]
pub struct Scene {
    pub lights          : Vec<Arc<Lights>>,
    pub infinite_lights : Vec<Arc<Lights>>,
    pub aggregate       : Arc<Primitives>,
    // world bound
    pub wb              : Bounds3f
}

impl Scene {
    pub fn new(aggregate: Arc<Primitives>, lights: Vec<Arc<Lights>>) -> Self {
        let wb = aggregate.world_bound();
        let mut scene = Scene {
            aggregate, wb, lights,
            infinite_lights: Vec::new()
        };


        let mut infinite_lights = Vec::new();

        for light in  scene.lights.iter() {
            light.preprocess(&scene);

            if (light.flags() & LightFlags::Infinite as u8) != 0 {
                infinite_lights.push(light.clone())
            }
        }

        scene.infinite_lights = infinite_lights;
        scene
    }

    pub fn intersect(&self, r: &mut Ray, isect: &mut SurfaceInteraction) -> bool {
        nintersection_tests::inc();
        assert_ne!(r.d, Vector3f::new(0.0, 0.0, 0.0));

        self.aggregate.intersect(r, isect, self.aggregate.clone())
    }

    pub fn intersect_p(&self, ray: &mut Ray) -> bool {
        nshadow_tests::inc();
        assert_ne!(ray.d, Vector3f::new(0.0, 0.0, 0.0));

        self.aggregate.intersect_p(ray)
    }

    pub fn intersect_tr(
        &self, mut ray: Ray, sampler: &mut Samplers,
        isect: &mut SurfaceInteraction, tr: &mut Spectrum) -> bool {
        *tr = Spectrum::new(1.0);

        loop {
            let hits = self.intersect(&mut ray, isect);

            // Accumulate beam transmittance for ray segment
            if let Some(ref m) = ray.medium {
                *tr *= m.tr(&ray, sampler);
            }

            // Initialize next ray segment or terminate transmittance computation
            if !hits { return false; }
            if isect.primitive.as_ref().unwrap().get_material().is_some() { return true; }

            ray = isect.spawn_ray(&ray.d);
        }
    }
}