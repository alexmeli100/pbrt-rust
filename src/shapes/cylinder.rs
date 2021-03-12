use crate::core::pbrt::{Float, radians, clamp, PI, gamma, lerp};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::interaction::{SurfaceInteraction, InteractionData};
use crate::core::geometry::vector::{Vector3f, Vector3};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::paramset::ParamSet;

#[derive(Debug, Clone)]
pub struct Cylinder {
    radius                   : Float,
    zmax                     : Float,
    zmin                     : Float,
    phi_max                  : Float,
    object_to_world          : Arc<Transform>,
    world_to_object          : Arc<Transform>,
    reverse_orientation      : bool,
    transform_swapshandedness: bool
}

impl Cylinder {
    pub fn new(
        object_to_world: Arc<Transform>, world_to_object: Arc<Transform>, reverse_orientation: bool,
        radius: Float, zmin: Float, zmax: Float, phi_max: Float) -> Self {
        let sh = object_to_world.swaps_handedness();

        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            zmin: zmin.min(zmax),
            zmax: zmax.max(zmin),
            phi_max: (radians(clamp(phi_max, 0.0, 360.0))),
            transform_swapshandedness: sh
        }
    }
}

impl Shape for Cylinder {
    fn object_bound(&self) -> Bounds3f {
        Bounds3f::from_points(
           Point3f::new(-self.radius, -self.radius, self.zmin),
           Point3f::new(self.radius, self.radius, self.zmax))
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        let mut phi;
        let mut p_hit;

        // Transform ray to object space
        let mut o_err = Vector3::default();
        let mut d_err = Vector3::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut o_err, &mut d_err);

        // compute quadratic sphere coefficients
        // Initialize EFloat ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        let _oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        let _dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx + dy * dy;
        let b = (dx * ox + dy * oy) * 2.0;
        let c = ox * ox + oy * oy - EFloat::from(self.radius) * EFloat::from(self.radius);

        // solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return false;
        }

        // check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return false;
        }

        let mut t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;

            if t_shape_hit.upper_bound() > ray.t_max {
                return false;
            }
        }

        // compute cylinder hit position and phi
        p_hit = ray.find_point(t_shape_hit.into());

        // refine cylinder intersection point
        let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        p_hit.x *= self.radius / hit_rad;
        p_hit.y *= self.radius / hit_rad;
        phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 {
            phi *= 2.0 * PI
        }

        // test cylinder intersection against clipping parameters
        if p_hit.z < self.zmin || p_hit.z > self.zmax || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }

            t_shape_hit = t1;

            if t1.upper_bound() > r.t_max {
                return false;
            }

            // compute cylinder hit point and phi
            p_hit = ray.find_point(t_shape_hit.into());

            // refine cylinder intersection point
            let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
            p_hit.x *= self.radius / hit_rad;
            p_hit.y *= self.radius / hit_rad;
            phi = p_hit.y.atan2(p_hit.x);

            if phi < 0.0 {
                phi *= 2.0 * PI
            }

            if p_hit.z < self.zmin || p_hit.z > self.zmax || phi > self.phi_max {
                return false;
            }
        }

        // find parametric representation fo cylinder hit
        let u = phi / self.phi_max;
        let v = (p_hit.z - self.zmin) / (self.zmax - self.zmin);

        // compute cylinder dpdu and dpdv
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(0.0, 0.0, self.zmax - self.zmin);

        // compute cylinder dndu and dndv
        let d2pduu = Vector3f::new(p_hit.x, p_hit.y, 0.0) * self.phi_max * -self.phi_max;
        let d2pduv = Vector3f::new(0.0, 0.0, 0.0);
        let d2pdvv = Vector3f::new(0.0, 0.0, 0.0);

        // compute coefficients for fundamental forms
        let E = dpdu.dot(&dpdu);
        let F = dpdu.dot(&dpdv);
        let G = dpdv.dot(&dpdv);
        let N: Vector3f = (dpdu.cross(&dpdv)).normalize();
        let e = N.dot(&d2pduu);
        let f = N.dot(&d2pduv);
        let g = N.dot(&d2pdvv);

        // compute dndu and dndv from fundamental form coefficients
        let invEGF2 = 1.0 / (E * G - F * F);
        let dndu = Normal3f::from(dpdu * invEGF2 * (f * F - e * G) + dpdv * invEGF2 * (e * F - f * E));
        let dndv = Normal3f::from(dpdu * invEGF2 * (g * F - f * G) + dpdv * invEGF2 * (f * F - g * E));

        // compute error bounds for cylinder intersection
        let p_error = Vector3f::new(p_hit.x, p_hit.y, 0.0).abs() * gamma(3);

        // Initialize SurfaceInteraction from parametric information
        let mut s = SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v), &-ray.d, &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
        *isect = self.object_to_world.transform_surface_interaction(&mut s);

        *t_hit = t_shape_hit.into();

        true

    }

    fn intersect_p(&self, r: &Ray, _test_alpha_texture: bool) -> bool {
        let mut phi;
        let mut p_hit;

        // Transform ray to object space
        let mut o_err = Vector3::default();
        let mut d_err = Vector3::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut o_err, &mut d_err);

        // compute quadratic sphere coefficients
        // Initialize EFloat ray coordinate values
        let ox = EFloat::new(ray.o.x, o_err.x);
        let oy = EFloat::new(ray.o.y, o_err.y);
        //let _oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, d_err.y);
        //let _dz = EFloat::new(ray.d.z, d_err.z);
        let a = dx * dx + dy * dy;
        let b = (dx * ox + dy * oy) * 2.0;
        let c = ox * ox + oy * oy - EFloat::from(self.radius) * EFloat::from(self.radius);

        // solve quadratic equation for t values
        let mut t0 = EFloat::default();
        let mut t1 = EFloat::default();
        if !quadratic(a, b, c, &mut t0, &mut t1) {
            return false;
        }

        // check quadratic shape t0 and t1 for nearest intersection
        if t0.upper_bound() > ray.t_max || t1.lower_bound() <= 0.0 {
            return false;
        }

        let mut t_shape_hit = t0;
        if t_shape_hit.lower_bound() <= 0.0 {
            t_shape_hit = t1;

            if t_shape_hit.upper_bound() > ray.t_max {
                return false;
            }
        }

        // compute cylinder hit position and phi
        p_hit = ray.find_point(t_shape_hit.into());

        // refine cylinder intersection point
        let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        p_hit.x *= self.radius / hit_rad;
        p_hit.y *= self.radius / hit_rad;
        phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 {
            phi *= 2.0 * PI
        }

        // test cylinder intersection against clipping parameters
        if p_hit.z < self.zmin || p_hit.z > self.zmax || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }

            t_shape_hit = t1;

            if t1.upper_bound() > r.t_max {
                return false;
            }

            // compute cylinder hit point and phi
            p_hit = ray.find_point(t_shape_hit.into());

            // refine cylinder intersection point
            let hit_rad = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
            p_hit.x *= self.radius / hit_rad;
            p_hit.y *= self.radius / hit_rad;
            phi = p_hit.y.atan2(p_hit.x);

            if phi < 0.0 {
                phi *= 2.0 * PI
            }

            if p_hit.z < self.zmin || p_hit.z > self.zmax || phi > self.phi_max {
                return false;
            }
        }

        true
    }

    fn area(&self) -> f32 {
        (self.zmax - self.zmin) * self.radius * self.phi_max
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let z = lerp(u[0], self.zmin, self.zmax);
        let phi = u[1] * self.phi_max;
        let mut pobj = Point3f::new(self.radius * phi.cos(), self.radius * phi.sin(), z);
        let mut it = InteractionData::default();
        it.n = (self.object_to_world.transform_normal(&Normal3f::new(pobj.x, pobj.y, 0.0))).normalize();
        if self.reverse_orientation { it.n *= -1.0; }

        // Reproject pobj to cylinder surface and compute pobj_error
        let hitrad = (pobj.x * pobj.x + pobj.y * pobj.y).sqrt();
        pobj.x *= self.radius / hitrad;
        pobj.y *= self.radius / hitrad;
        let pobj_error = Vector3f::new(pobj.x, pobj.y, 0.0).abs() * gamma(3);
        it.p = self.object_to_world.transform_point_abs_error(&pobj, &pobj_error, &mut it.p_error);
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

pub fn create_cylinder_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool,
    params: &ParamSet) -> Arc<Shapes> {
    let radius = params.find_one_float("radius", 1.0);
    let zmin = params.find_one_float("zmin", -1.0);
    let zmax = params.find_one_float("zmax", -1.0);
    let phimax = params.find_one_float("phimax", 360.0);

    let c = Cylinder::new(
        o2w, w2o,
        reverse_orientation, radius, zmin, zmax, phimax);

    Arc::new(c.into())
}