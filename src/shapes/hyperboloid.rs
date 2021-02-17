use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::pbrt::{Float, clamp, radians, PI};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::interaction::{InteractionData, SurfaceInteraction};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use log::error;
use crate::core::geometry::vector::Vector3f;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::paramset::ParamSet;

#[derive(Default, Clone)]
pub struct Hyperboloid {
    p1                              : Point3f,
    p2                              : Point3f,
    zmin                            : Float,
    zmax                            : Float,
    phi_max                         : Float,
    rmax                            : Float,
    ah                              : Float,
    ch                              : Float,
    pub object_to_world             : Arc<Transform>,
    pub world_to_object             : Arc<Transform>,
    pub reverse_orientation         : bool,
    pub transform_swapshandedness   : bool
}

impl Hyperboloid {
    pub fn new(
        o2w: Arc<Transform>, w2o: Arc<Transform>, ro: bool,
        point1: Point3f, point2: Point3f, tm: Float) -> Self {
        let mut p1 = point1;
        let mut p2 = point2;
        let phi_max = radians(clamp(tm, 0.0, 360.0));
        let radius1 = (p1.x * p1.x + p1.y * p1.y).sqrt();
        let radius2 = (p2.x * p2.x + p2.y * p2.y).sqrt();
        let rmax = radius1.max(radius2);
        let zmin = p1.z.min(p2.z);
        let zmax = p1.z.max(p2.z);

        // Compute implicit function coefficients for hyperboloid
        if p2.z == 0.0 { std::mem::swap(&mut p1, &mut p2); }
        let mut pp = p1;
        let mut xy1: Float;
        let mut xy2: Float;
        let mut ah: Float;
        let mut ch: Float;

        loop {
            pp += (p2 - p1) * 2.0;
            xy1 = pp.x * pp.x + pp.y * pp.y;
            xy2 = p2.x * p2.x + p2.y * p2.y;

            ah = (1.0 / xy1 - (pp.z * pp.z) / (xy1 * p2.z * p2.z)) /
                 (1.0 - (xy2 * pp.z * pp.z) / (xy1 * p2.z * p2.z));
            ch = (ah * xy2 - 1.0) / (p2.z * p2.z);

            if !ah.is_infinite() && !ah.is_nan() { break; }
        }

        let sh = o2w.swaps_handedness();
        Self {
            p1, p2, zmin, zmax, phi_max, rmax,
            ah, ch,
            object_to_world: o2w,
            world_to_object: w2o,
            reverse_orientation: ro,
            transform_swapshandedness: sh
        }
    }
}

macro_rules! sqr {
    ($a:expr) => {{
        $a * $a
    }}
}

macro_rules! quad {
    ($a:expr) => {{
        sqr!($a) * sqr!($a)
    }}
}

impl Shape for Hyperboloid {
    fn object_bound(&self) -> Bounds3f {
        let p1 = Point3f::new(-self.rmax, -self.rmax, self.zmin);
        let p2 = Point3f::new(self.rmax, self.rmax, self.zmax);

        Bounds3f::from_points(p1, p2)
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        // TODO: ProfilePhase
        let mut phi: Float;
        let mut v: Float;
        let mut phit: Point3f;
        //Transform ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic hyperboloid coefficients
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let a = dx * dx * self.ah + dy * dy * self.ah - dz * dz * self.ch;
        let b = (dx * ox * self.ah + dy * oy * self.ah - dz * oz * self.ch) * 2.0;
        let c = ox * ox * self.ah + oy * oy * self.ah - oz * oz * self.ch - 1.0;

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; }

        // Check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute hyperboloid inverse mapping
        phit = ray.find_point(tshape_hit.into());
        v = (phit.z - self.p1.z) / (self.p2.z - self.p1.z);
        let pr = self.p1 * (1.0 - v) + self.p2 * v;
        phi = (pr.x * phit.y - phit.x * pr.y).atan2(phit.x * pr.x + phit.y * pr.y);

        if phi < 0.0 { phi += 2.0 * PI; }

        // Test hyperboloid intersection against clipping parameters
        if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute hyperboloid inverse mapping
            phit = ray.find_point(tshape_hit.into());
            v = (phit.z - self.p1.z) / (self.p2.z - self.p1.z);
            let pr = self.p1 * (1.0 - v) + self.p2 * v;
            phi = (pr.x * phit.y - phit.x * pr.y).atan2(phit.x * pr.x + phit.y * pr.y);

            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max { return false; }
        }

        // Compute parametric representation fo hyperboloid hit
        let u = phi / self.phi_max;

        // Compute hyperboloid dpdu and dpdv
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();
        let dpdu = Vector3f::new(
            -self.phi_max * phit.y,
            self.phi_max * phit.x,
            0.0);
        let dpdv = Vector3f::new(
            (self.p2.x - self.p1.x) * cos_phi - (self.p2.y - self.p1.y) * sin_phi,
            (self.p2.x - self.p1.x) * sin_phi - (self.p2.y - self.p1.y) * cos_phi,
            self.p2.z - self.p1.z);

        // Compute hyperboloid dndu and dndv
        let d2pduu = Vector3f::new(phit.x, phit.y, 0.0) * -self.phi_max * self.phi_max;
        let d2pduv = Vector3f::new(-dpdv.y, dpdv.x, 0.0) * self.phi_max;
        let d2pdvv = Vector3f::default();

        // Compute coefficients for fundamental forms
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
            dpdu * inv_EGF2 * (g * F -f * G) +
               dpdv * inv_EGF2 * (f * F - g * E));

        // Compute error bounds for hyperboloid intersection

        // Compute error bounds for intersection computed with ray equation
        let px = ox + tshape_hit * dx;
        let py = oy + tshape_hit * dy;
        let pz = oz * tshape_hit * dz;
        let perror = Vector3f::new(
            px.get_absolute_error(),
            py.get_absolute_error(),
            pz.get_absolute_error());

        let uv = Point2f::new(u, v);
        let si = SurfaceInteraction::new(
            &phit, &perror, &uv, &(-ray.d),
            &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
        *isect = self.object_to_world.transform_surface_interaction(&si);
        *t_hit = tshape_hit.into();

        true
    }

    fn intersect_p(&self, r: &Ray, _test_alpha_texture: bool) -> bool {
        // TODO: ProfilePhase
        let mut phi: Float;
        let mut v: Float;
        let mut phit: Point3f;
        //Transform ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic hyperboloid coefficients
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let a = dx * dx * self.ah + dy * dy * self.ah - dz * dz * self.ch;
        let b = (dx * ox * self.ah + dy * oy * self.ah - dz * oz * self.ch) * 2.0;
        let c = ox * ox * self.ah + oy * oy * self.ah - oz * oz * self.ch - 1.0;

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; }

        // Check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute hyperboloid inverse mapping
        phit = ray.find_point(tshape_hit.into());
        v = (phit.z - self.p1.z) / (self.p2.z - self.p1.z);
        let pr = self.p1 * (1.0 - v) + self.p2 * v;
        phi = (pr.x * phit.y - phit.x * pr.y).atan2(phit.x * pr.x + phit.y * pr.y);

        if phi < 0.0 { phi += 2.0 * PI; }

        // Test hyperboloid intersection against clipping parameters
        if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute hyperboloid inverse mapping
            phit = ray.find_point(tshape_hit.into());
            v = (phit.z - self.p1.z) / (self.p2.z - self.p1.z);
            let pr = self.p1 * (1.0 - v) + self.p2 * v;
            phi = (pr.x * phit.y - phit.x * pr.y).atan2(phit.x * pr.x + phit.y * pr.y);

            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < self.zmin || phit.z > self.zmax || phi > self.phi_max { return false; }
        }

        true
    }

    fn area(&self) -> f32 {
        self.phi_max / 6.0 *
        (2.0 * quad!(self.p1.x) - 2.0 * self.p1.x * self.p1.x * self.p1.x * self.p2.x + 2.0 * quad!(self.p2.x) +
            2.0 * (self.p1.y * self.p1.y + self.p1.y * self.p2.y + self.p2.y * self.p2.y) *
                (sqr!(self.p1.y - self.p2.y) + sqr!(self.p1.z - self.p2.z)) +
            self.p2.x * self.p2.x * (5.0 * self.p1.y * self.p1.y + 2.0 * self.p1.y * self.p2.y - 4.0 * self.p2.y * self.p2.y +
                                     2.0 * sqr!(self.p1.z - self.p2.z)) +
            self.p1.x * self.p1.x * (-4.0 * self.p1.y * self.p1.y + 2.0 * self.p1.y * self.p2.y +
                                     5.0 * self.p2.y * self.p2.y + 2.0 * sqr!(self.p1.z - self.p2.z)) -
            2.0 * self.p1.x * self.p2.x *
                  (self.p2.x * self.p2.x - self.p1.y * self.p1.y + 5.0 * self.p1.y * self.p2.y - self.p2.y * self.p2.y -
                   self.p1.z * self.p1.z + 2.0 * self.p1.z * self.p2.z - self.p2.z * self.p2.z))
    }

    fn sample(&self, _u: &Point2f, _pdf: &mut f32) -> InteractionData {
        error!("Hyperboloid::Sample not implemented");

        InteractionData::default()
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

pub fn create_hyperboloid_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool, params: &ParamSet) -> Arc<Shapes> {
    let p1 = params.find_one_point3f("p1", Default::default());
    let p2 = params.find_one_point3f("p2", Point3f::new(1.0, 1.0, 1.0));
    let phimax = params.find_one_float("phimax", 360.0);

    let h = Hyperboloid::new(o2w, w2o, reverse_orientation, p1, p2, phimax);

    Arc::new(h.into())
}