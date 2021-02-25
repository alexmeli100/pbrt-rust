use crate::core::pbrt::{Float, radians, clamp, PI};
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
pub struct Cone {
    radius                          : Float,
    height                          : Float,
    phi_max                         : Float,
    pub object_to_world             : Arc<Transform>,
    pub world_to_object             : Arc<Transform>,
    pub reverse_orientation         : bool,
    pub transform_swapshandedness   : bool
}

impl Cone {
    pub fn new(
        o2w: Arc<Transform>, w2o: Arc<Transform>,
        ro: bool, height: Float, radius: Float,
        phi_max: Float) -> Self {
        let sh = o2w.swaps_handedness();

        Self {
            radius, height,
            object_to_world: o2w,
            world_to_object: w2o,
            reverse_orientation: ro,
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            transform_swapshandedness: sh
        }
    }
}

impl Shape for Cone {
    fn object_bound(&self) -> Bounds3f {
        let p1 = Point3f::new(-self.radius, -self.radius, 0.0);
        let p2 = Point3f::new(self.radius, self.radius, self.height);

        Bounds3f::from_points(p1, p2)
    }

    fn intersect(
        &self, r: &Ray, thit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        // TODO: ProfilePhase
        let mut phi;
        let mut phit;
        // Transform Ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic cone coefficients

        // Initialize EFloat ray coordinate values
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let mut k = EFloat::from(self.radius) / EFloat::from(self.radius);
        k = k * k;
        let a  = dx * dx + dy * dy - k * dz * dz;
        let b = (dx * ox + dy * oy - k * dz * (oz - self.height)) * 2.0;
        let c = ox * ox + oy * oy - k * (oz - self.height) * (oz - self.height);

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; };

        // Check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute cone inverse mapping
        phit = ray.find_point(tshape_hit.into());
        phi = phit.y.atan2(phit.x);
        if phi < 0.0 { phi += 2.0 * PI; }

        // Test cone intersection against clipping parameters
        if phit.z < 0.0 || phit.z > self.height || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute cone inverse mapping
            phit = ray.find_point(tshape_hit.into());
            phi = phit.y.atan2(phit.x);
            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < 0.0 || phit.z > self.height || phi > self.phi_max { return false; }
        }

        // Find parametric representation of cone hit
        let u = phi / self.phi_max;
        let v = phit.z / self.height;

        // Compute cone dpdu and dpdv
        let dpdu = Vector3f::new(-self.phi_max * phit.y, self.phi_max * phit.x, 0.0);
        let dpdv = Vector3f::new(-phit.x / (1.0 - v), -phit.y / (1.0 - v), self.height);

        // Compute cone dndu and dndv
        let d2pduu = Vector3f::new(phit.x, phit.y, 0.0) * -self.phi_max * self.phi_max;
        let d2pduv = Vector3f::new(phit.y, -phit.x, 0.0) * self.phi_max / (1.0 - v);
        let d2pdvv = Vector3f::default();

        // Compute coefficients for fundamenta forms
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
               dpdv * inv_EGF2 * (e * F -f * E));
        let dndv = Normal3f::from(
            dpdu * inv_EGF2 * (g * F - f * G) +
               dpdv * inv_EGF2 * (f * F - g * E));

        // Compute error bounds for cone intersection

        // Compute error bounds for intersection computed with ray equation
        let px = ox + tshape_hit * dx;
        let py = oy + tshape_hit * dy;
        let pz = oz + tshape_hit * dz;
        let perror = Vector3f::new(
            px.get_absolute_error(),
            py.get_absolute_error(),
            pz.get_absolute_error());

        // Initialize SurfaceInteraction from parametric information
        let uv = Point2f::new(u, v);
        let mut si = SurfaceInteraction::new(
            &phit, &perror, &uv, &(-ray.d),
            &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
        *isect = self.object_to_world.transform_surface_interaction(&mut si);
        *thit = tshape_hit.into();

        true
    }

    fn intersect_p(&self, r: &Ray, _test_alpha_texture: bool) -> bool {
        // TODO: ProfilePhase
        let mut phi;
        let mut phit;
        // Transform Ray to object space
        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute quadratic cone coefficients

        // Initialize EFloat ray coordinate values
        let ox = EFloat::new(ray.o.x, oerr.x);
        let oy = EFloat::new(ray.o.y, oerr.y);
        let oz = EFloat::new(ray.o.z, oerr.z);
        let dx = EFloat::new(ray.d.x, derr.x);
        let dy = EFloat::new(ray.d.y, derr.y);
        let dz = EFloat::new(ray.d.z, derr.z);
        let mut k = EFloat::from(self.radius) / EFloat::from(self.radius);
        k = k * k;
        let a  = dx * dx + dy * dy - k * dz * dz;
        let b = (dx * ox + dy * oy - k * dz * (oz - self.height)) * 2.0;
        let c = ox * ox + oy * oy - k * (oz - self.height) * (oz - self.height);

        // Solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) { return false; };

        // Check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 { return false; }
        let mut tshape_hit = t0;
        if tshape_hit.lower_bound() <= 0.0 {
            tshape_hit = t1;
            if tshape_hit.upper_bound() > ray.t_max { return false; }
        }

        // Compute cone inverse mapping
        phit = ray.find_point(tshape_hit.into());
        phi = phit.y.atan2(phit.x);
        if phi < 0.0 { phi += 2.0 * PI; }

        // Test cone intersection against clipping parameters
        if phit.z < 0.0 || phit.z > self.height || phi > self.phi_max {
            if tshape_hit == t1 { return false; }
            tshape_hit = t1;
            if t1.upper_bound() > ray.t_max { return false; }
            // Compute cone inverse mapping
            phit = ray.find_point(tshape_hit.into());
            phi = phit.y.atan2(phit.x);
            if phi < 0.0 { phi += 2.0 * PI; }
            if phit.z < 0.0 || phit.z > self.height || phi > self.phi_max { return false; }
        }

        true
    }

    fn area(&self) -> f32 {
        self.radius * ((self.height * self.height) + (self.radius * self.radius)) * self.phi_max / 2.0
    }

    fn sample(&self, _u: &Point2f, _pdf: &mut f32) -> InteractionData {
        error!("Cone::Sample not implemented.");

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

pub fn create_cone_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool,
    params: &ParamSet) -> Arc<Shapes> {
    let radius = params.find_one_float("radius", 1.0);
    let height = params.find_one_float("height", 1.0);
    let phimax = params.find_one_float("phimax", 360.0);

    let c = Cone::new(o2w, w2o, reverse_orientation,
                      height, radius, phimax);

    Arc::new(c.into())
}