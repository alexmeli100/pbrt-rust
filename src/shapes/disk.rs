use crate::core::pbrt::{Float, radians, clamp, PI};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::interaction::{SurfaceInteraction, InteractionData};
use crate::core::geometry::vector::{Vector3f, Vector3};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::sampling::concentric_sample_disk;
use crate::core::paramset::ParamSet;

#[derive(Debug, Clone, Default)]
pub struct Disk {
    height                   : Float,
    radius                   : Float,
    inner_radius             : Float,
    phi_max                  : Float,
    object_to_world          : Arc<Transform>,
    world_to_object          : Arc<Transform>,
    reverse_orientation      : bool,
    transform_swapshandedness: bool
}

impl Disk {
    pub fn new(
        object_to_world: Arc<Transform>, world_to_object: Arc<Transform>,
        reverse_orientation: bool, height: Float, radius: Float,
        inner_radius: Float, phi_max: Float) -> Self {
        let sh = object_to_world.swaps_handedness();

        Self {
            object_to_world, world_to_object,
            reverse_orientation, height,
            radius, inner_radius,
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            transform_swapshandedness: sh
        }
    }
}

impl Shape for Disk {
    fn object_bound(&self) -> Bounds3f {
        Bounds3f::from_points(
            Point3f::new(-self.radius, -self.radius, self.height),
            Point3f::new(self.radius, self.radius, self.height)
        )
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        // Transform ray to object space
        let mut o_err = Vector3::default();
        let mut d_err = Vector3::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut o_err, &mut d_err);

        // compute plane intersection for disk
        // reject disk intersection for rays parallel to the disk's plane
        if ray.d.z == 0.0 { return false; }

        let t_shape_hit = (self.height - ray.o.z) / r.d.z;

        if t_shape_hit <= 0.0 || t_shape_hit >= ray.t_max { return false; }

        // see if hit point is inside disk radii and phi_max
        let mut p_hit = ray.find_point(t_shape_hit);
        let dist2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;

        if dist2 > self.radius * self.radius || dist2 < self.inner_radius * self.inner_radius {
            return false
        }

        // test disk phi value against phi_max
        let mut phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 { phi += 2.0 * PI; }

        if phi > self.phi_max { return false; }

        // find parametric representation of disk hit
        let u = phi / self .phi_max;
        let r_hit = dist2.sqrt();
        let v = (self.radius - r_hit) / (self.radius - self.inner_radius);
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(p_hit.x, p_hit.y, 0.0) * (self.inner_radius - self.radius) / r_hit;
        let dndu = Normal3f::new(0.0, 0.0, 0.0);
        let dndv = Normal3f::new(0.0, 0.0, 0.0);

        // refine disk intersection point
        p_hit.z = self.height;

        // compute error bounds for disl intersection
        let p_error = Vector3f::new(0.0, 0.0, 0.0);

        // initialize surface interaction from parametric information
        let mut s = SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v), &-ray.d, &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
        *isect = self.object_to_world.transform_surface_interaction(&mut s);

        *t_hit = t_shape_hit;

        true

    }

    fn intersect_p(&self, r: &Ray, _test_alpha_texture: bool) -> bool {
        // Transform ray to object space
        let mut o_err = Vector3::default();
        let mut d_err = Vector3::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut o_err, &mut d_err);

        // compute plane intersection for disk
        // reject disk intersection for rays parallel to the disk's plane
        if ray.d.z == 0.0 { return false; }
        let t_shape_hit = (self.height - ray.o.z) / ray.d.z;

        if t_shape_hit <= 0.0 || t_shape_hit >= ray.t_max { return false; }

        // see if hit point is inside disk radii and phi_max
        let p_hit = ray.find_point(t_shape_hit);
        let dist2 = p_hit.x * p_hit.x + p_hit.y * p_hit.y;

        if dist2 > self.radius * self.radius || dist2 < self.inner_radius * self.inner_radius {
            return false
        }

        // test disk phi value against phi_max
        let mut phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 { phi += 2.0 * PI; }

        if phi > self.phi_max { return false; }

        true
    }

    fn area(&self) -> f32 {
        self.phi_max * 0.5 * (self.radius * self.radius - self.inner_radius * self.inner_radius)
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let pd = concentric_sample_disk(u);
        let pobj = Point3f::new(pd.x * self.radius, pd.y * self.radius, self.height);
        let mut it = InteractionData::default();
        it.n = (self.object_to_world.transform_normal(&Normal3f::new(0.0, 0.0, 0.1))).normalize();
        if self.reverse_orientation { it.n *= -1.0; }
        it.p = self.object_to_world.transform_point_abs_error(
            &pobj,
            &Vector3f::new(0.0, 0.0, 0.0),
            &mut it.p_error);



        *pdf = 1.0 / self.area();
        it
    }

    fn reverse_orientation(&self) -> bool {
        self.reverse_orientation
    }

    fn transform_swapshandedness(&self) -> bool {
        self.transform_swapshandedness
    }

    fn object_to_world(&self) -> Arc<Transform> {
        self.object_to_world.clone()
    }

    fn world_to_object(&self) -> Arc<Transform> {
        self.world_to_object.clone()
    }
}

pub fn create_disk_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool, params: &ParamSet) -> Arc<Shapes> {
    let height = params.find_one_float("height", 0.0);
    let radius = params.find_one_float("radius", 1.0);
    let iradius = params.find_one_float("innerradius", 0.0);
    let phimax = params.find_one_float("phimax", 360.0);

    let d = Disk::new(
        o2w, w2o, reverse_orientation,
        height, radius, iradius, phimax);

    Arc::new(d.into())
}