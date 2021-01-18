use crate::core::pbrt::{Float, radians, clamp, quadratic, PI, gamma};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::interaction::{Interaction, SurfaceInteraction, Interactions};
use crate::core::geometry::vector::{Vector3f, Vector3};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use crate::core::efloat::EFloat;
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;

#[derive(Debug, Copy, Clone)]
pub struct Cylinder {
    radius                   : Float,
    zmax                     : Float,
    zmin                     : Float,
    phi_max                  : Float,
    object_to_world          : Transform,
    world_to_object          : Transform,
    reverse_orientation      : bool,
    transform_swapshandedness: bool
}

impl Cylinder {
    pub fn new(
        object_to_world: Transform, world_to_object: Transform, reverse_orientation: bool, 
        radius: Float, zmin: Float, zmax: Float, phi_max: Float) -> Self {
        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            zmin: zmin.min(zmax),
            zmax: zmax.max(zmin),
            phi_max: (radians(clamp(phi_max, 0.0, 360.0))),
            transform_swapshandedness: false
        }
    }
}

impl Shape for Cylinder {
    fn object_bound(&self) -> Bounds3f {
        Bounds3f::from_points(
           Point3f::new(-self.radius, -self.radius, self.zmin),
           Point3f::new(self.radius, self.radius, self.zmax))
    }

    fn intersect(&self, r: &Ray, t_hit: &mut f32, isect: &mut SurfaceInteraction, test_aphatexture: bool) -> bool {
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
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, o_err.y);
        let dz = EFloat::new(ray.d.z, o_err.z);
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
        let N: Vector3f = (dpdv.cross(&dpdv)).normalize();
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
        let shape = Some(Arc::new((*self).into()));
        let mut s = SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v), &-ray.d, &dpdu, &dpdv, &dndu, &dndv, ray.time, shape);
        *isect = self.object_to_world.transform_surface_interaction(&mut s);

        *t_hit = t_shape_hit.into();

        true

    }

    fn intersect_p(&self, r: &Ray, test_alpha_texture: bool) -> bool {
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
        let oz = EFloat::new(ray.o.z, o_err.z);
        let dx = EFloat::new(ray.d.x, d_err.x);
        let dy = EFloat::new(ray.d.y, o_err.y);
        let dz = EFloat::new(ray.d.z, o_err.z);
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

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> Interactions {
        unimplemented!()
    }

    fn sample_interaction(&self, i: &Interactions, u: &Point2f, pdf: &mut Float) -> Interactions {
        unimplemented!()
    }

    fn pdf(&self, i: &Interactions, wi: &Vector3f) -> f32 {
        unimplemented!()
    }

    fn reverse_orientation(&self) -> bool {
        self.reverse_orientation
    }

    fn transform_swapshandedness(&self) -> bool {
        self.transform_swapshandedness
    }

    fn object_to_world(&self) -> Transform {
        self.object_to_world
    }

    fn world_to_object(&self) -> Transform {
        self.world_to_object
    }
}