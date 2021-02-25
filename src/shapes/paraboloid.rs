use crate::core::pbrt::{Float, clamp, radians, PI};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::interaction::{InteractionData, SurfaceInteraction};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use log::error;
use crate::core::geometry::vector::Vector3f;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::paramset::ParamSet;

#[derive(Clone)]
pub struct Paraboloid {
    radius                          : Float,
    zmin                            : Float,
    zmax                            : Float,
    phi_max                         : Float,
    pub object_to_world             : Arc<Transform>,
    pub world_to_object             : Arc<Transform>,
    pub reverse_orientation         : bool,
    pub transform_swapshandedness   : bool
}

impl Paraboloid {
    pub fn new(
        o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool,
        radius: Float, z0: Float, z1: Float, phi_max: Float) -> Self {
        let sh = o2w.swaps_handedness();

        Self {
            radius,
            reverse_orientation,
            zmin: z0.min(z1),
            zmax: z0.max(z1),
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            object_to_world: o2w,
            world_to_object: w2o,
            transform_swapshandedness: sh
        }
    }
}

impl Shape for Paraboloid {
    fn object_bound(&self) -> Bounds3f {
        let p1 = Point3f::new(-self.radius, -self.radius, self.zmin);
        let p2 = Point3f::new(self.radius, self.radius, self.zmax);

        Bounds3f::from_points(p1, p2)
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        // TODO: ProfilePhase
        let mut phi: Float;
        let mut phit: Point3f;
        //Transform Ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic paraboloid coefficients

        // Initialize EFloat and ray coordinate values
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let k = EFloat::from(self.zmax) / EFloat::from(self.radius) * EFloat::from(self.radius);
        let a = k * (dx * dx + dy * dy);
        let b = (dx * ox + dy * oy) * k * 2.0 - dz;
        let c = k * (ox * ox + oy * oy) - oz;

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; }

        // Check quadtic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute paraboloid inverse mapping
        phit = ray.find_point(tshape_hit.into());
        phi = phit.y.atan2(phit.x);
        if phi < 0.0 { phi += 2.0 * PI; }

        // Test paraboloid intersection against clipping parameters
        if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute paraboloid inverse mapping
            phit = ray.find_point(tshape_hit.into());
            phi = phit.y.atan2(phit.x);
            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max { return false; }
        }

        // Find parametric representation of paraboloid hit
        let u = phi / self.phi_max;
        let v = (phit.z - self.zmin) / (self.zmax - self.zmin);

        // Compute paraboloid dpdu and dpdv
        let dpdu = Vector3f::new(-self.phi_max * phit.y, self.phi_max * phit.x, 0.0);
        let dpdv = Vector3f::new(phit.x / (2.0 * phit.z), phit.y / (2.0 * phit.z), 1.0) *
            (self.zmax - self.zmin);

        // Compute paraboloid dndu dndv
        let d2pduu = Vector3f::new(phit.x, phit.y, 0.0) * -self.phi_max * self.phi_max;
        let d2pduv = Vector3f::new(-phit.y / (2.0 * phit.z), phit.x / (2.0 * phit.z), 0.0) *
            (self.zmax - self.zmin) * self.phi_max;
        let d2pdvv = Vector3f::new(phit.x / (4.0 * phit.z * phit.z), phit.y / (4.0 * phit.z * phit.z), 0.0) *
            -(self.zmax - self.zmin) * (self.zmax - self.zmin);

        // Compute coefficients for funcdamenta forms
        let E = dpdu.dot(&dpdu);
        let F = dpdu.dot(&dpdv);
        let G = dpdv.dot(&dpdv);
        let N = dpdu.cross(&dpdv).normalize();
        let e = N.dot(&d2pduu);
        let f = N.dot(&d2pduv);
        let g = N.dot(&d2pdvv);

        // Compute dndu and dndv from fundamental form coefficients
        let inv_EGF2 = 1.0 / (E * G - F * F);
        let dndu = Normal3f::from(
            dpdu * inv_EGF2 * (f * F - e * G) +
               dpdv * inv_EGF2 * (e * F - f * E));
        let dndv = Normal3f::from(
            dpdu * inv_EGF2 * (g * F - f * G) +
               dpdv * inv_EGF2 * (f * F - g * E));

        // Compute error bounds for paraboloid intersection

        // Compute error bounds for intersection computed with ray equation
        let px = ox + tshape_hit * dx;
        let py = oy + tshape_hit * dy;
        let pz = oz + tshape_hit * dz;
        let perror = Vector3f::new(
            px.get_absolute_error(),
            py.get_absolute_error(),
            pz.get_absolute_error());

        let uv = Point2f::new(u, v);
        let mut si = SurfaceInteraction::new(
            &phit, &perror, &uv, &(-ray.d),
            &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
        *isect = self.object_to_world.transform_surface_interaction(&mut si);
        *t_hit = tshape_hit.into();

        true
    }

    fn intersect_p(&self, r: &Ray, _test_alpha_texture: bool) -> bool {
        // TODO: ProfilePhase
        let mut phi: Float;
        let mut phit: Point3f;
        //Transform Ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic paraboloid coefficients

        // Initialize EFloat and ray coordinate values
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let k = EFloat::from(self.zmax) / EFloat::from(self.radius) * EFloat::from(self.radius);
        let a = k * (dx * dx + dy * dy);
        let b = (dx * ox + dy * oy) * k * 2.0 - dz;
        let c = k * (ox * ox + oy * oy) - oz;

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; }

        // Check quadtic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute paraboloid inverse mapping
        phit = ray.find_point(tshape_hit.into());
        phi = phit.y.atan2(phit.x);
        if phi < 0.0 { phi += 2.0 * PI; }

        // Test paraboloid intersection against clipping parameters
        if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute paraboloid inverse mapping
            phit = ray.find_point(tshape_hit.into());
            phi = phit.y.atan2(phit.x);
            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max { return false; }
        }

        true
    }

    fn area(&self) -> f32 {
        let radius2 = self.radius * self.radius;
        let k = 4.0 * self.zmax / radius2;

        (radius2 * radius2 * self.phi_max / (12.0 * self.zmax * self.zmax)) *
        ((k * self.zmax + 1.0).powf(1.5) - (k * self.zmin + 1.0).powf(1.5))
    }

    fn sample(&self, _u: &Point2f, _pdf: &mut f32) -> InteractionData {
        error!("Paraboloid::sample not implemented.");

        Default::default()
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

pub fn create_paraboloid_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool, params: &ParamSet) -> Arc<Shapes> {
    let radius = params.find_one_float("radius", 1.0);
    let zmin = params.find_one_float("zmin", 0.0);
    let zmax = params.find_one_float("zmax", 1.0);
    let phimax = params.find_one_float("phimax", 360.0);

    let p = Paraboloid::new(
        o2w, w2o, reverse_orientation,
        radius, zmin, zmax, phimax);
    Arc::new(p.into())
}