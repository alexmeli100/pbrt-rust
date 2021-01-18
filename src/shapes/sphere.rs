use crate::core::pbrt::{Float, clamp, radians, quadratic, gamma, PI};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::point::{Point2, Point3, Point2f, Point3f};
use crate::core::interaction::{Interaction, SurfaceInteraction, Interactions};
use crate::core::geometry::bounds::{Bounds3f};
use crate::core::geometry::vector::{Vector3, Vector3f};
use crate::core::geometry::ray::Ray;
use crate::core::efloat::EFloat;
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::paramset::ParamSet;

#[derive(Debug, Copy, Clone)]
pub struct Sphere {
    pub radius: Float,
    pub zmin: Float,
    pub zmax: Float,
    pub theta_min: Float,
    pub theta_max: Float,
    pub phi_max: Float,
    pub object_to_world: Transform,
    pub world_to_object: Transform,
    pub reverse_orientation: bool,
    pub transform_swapshandedness: bool
}

impl Sphere {
    pub fn new(object_to_world: Transform, world_to_object: Transform, reverse_orientation: bool, radius: Float, zmin: Float, zmax: Float, phi_max: Float) -> Self {
        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            zmin: clamp(zmin.min(zmax), -radius, radius),
            zmax: clamp(zmin.max(zmax), -radius, radius),
            theta_min: clamp(zmin / radius, -1.0, 1.0).acos(),
            theta_max: clamp(zmax / radius, -1.0, 1.0).acos(),
            phi_max: radians(clamp(phi_max, 0.0, 360.0)),
            transform_swapshandedness: false
        }
    }
}

impl Shape for Sphere {
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
        let a = dx * dx + dy * dy + dz * dz;
        let b = (dx * ox + dy * oy + dz * oz) * 2.0;
        let c = ox * ox + oy * oy + oz * oz - EFloat::from(self.radius) * EFloat::from(self.radius);

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

        // compute sphere hit position and phi
        p_hit = ray.find_point(t_shape_hit.into());

        // refine sphere intersection point
        p_hit *= p_hit.distance(&Point3::new(0.0, 0.0, 0.0)) / self.radius;

        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1e-5 as Float * self.radius;
        }

        phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 {
            phi += 2.0 * PI;
        }

        // test sphere intersection against clipping parameters
        if (self.zmin > -self.radius && p_hit.z < self.zmin) || (self.zmax < self.radius && p_hit.z > self.zmax) || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }

            if t1.upper_bound() > ray.t_max {
                return false;
            }

            t_shape_hit = t1;

            // compute sphere hit position and phi
            p_hit = ray.find_point(t_shape_hit.into());

            // refine sphere intersection point
            p_hit *= p_hit.distance(&Point3::new(0.0, 0.0, 0.0)) / self.radius;

            if p_hit.x == 0.0 && p_hit.y == 0.0 {
                p_hit.x = 1e-5 as Float * self.radius;
            }

            phi = p_hit.y.atan2(p_hit.x);

            if phi < 0.0 {
                phi += 2.0 * phi;
            }

            if (self.zmin > -self.radius && p_hit.z < self.zmin) || (self.zmax < self.radius && p_hit.z > self.zmax) || phi > self.phi_max {
                return false
            }
        }

        // find parametric representation of sphere hit
        let u = phi / self.phi_max;
        let theta = (clamp(p_hit.z / self.radius, -1.0, 1.0)).acos();
        let v = (theta - self.theta_min) / (self.theta_max - self.theta_min);

        // compute sphere dpdu and dpdv
        let zradius = (p_hit.x * p_hit.x + p_hit.y * p_hit.y).sqrt();
        let inv_radius = 1.0 / zradius;
        let cos_phi = p_hit.x * inv_radius;
        let sin_phi = p_hit.y * inv_radius;
        let dpdu = Vector3f::new(-self.phi_max * p_hit.y, self.phi_max * p_hit.x, 0.0);
        let dpdv = Vector3f::new(p_hit.z * cos_phi, p_hit.z * sin_phi, -self.radius * theta.sin()) * (self.theta_max - self.theta_min);

        // compute sphere dndu and dndv
        let d2pduu =  Vector3f::new(p_hit.x, p_hit.y, 0.0) * self.phi_max * self.phi_max;
        let d2pduv = Vector3f::new(-sin_phi, cos_phi, 0.0) * (self.theta_max - self.theta_min) * p_hit.z * self.phi_max;
        let d2pdvv = Vector3f::new(p_hit.x, p_hit.y, p_hit.z) * (self.theta_max - self.theta_min) * (self.theta_max - self.theta_min);

        // compute coefficeints for fundamental forms
        let E = dpdu.dot(&dpdu);
        let F = dpdu.dot(&dpdv);
        let G = dpdv.dot(&dpdv);
        let N: Vector3f = dpdu.cross(&dpdv).normalize();
        let e = N.dot(&d2pduu);
        let f = N.dot(&d2pduv);
        let g = N.dot(&d2pdvv);

        // compute dndu and dndv from fundamental form coefficents
        let inv_EGF2 = 1.0 / (E * G - F * F);
        let dndu = Normal3f::from(dpdu * inv_EGF2 * (f * F - e * G) + dpdv * inv_EGF2 * (e * F - f * E));
        let dndv = Normal3f::from(dpdu * inv_EGF2 * (g * F - f * G) + dpdv * inv_EGF2 * (f * F - g * E));

        // compute error bounds for sphere intersection
        let p_error = Vector3f::from(p_hit).abs() * gamma(5);

        // Initialize SurfaceInteraction from parametric information
        let shape = Some(Arc::new((*self).into()));
        let mut s = SurfaceInteraction::new(&p_hit, &p_error, &Point2f::new(u, v), &-ray.d, &dpdu, &dpdv, &dndu, &dndv, ray.time, shape);
        *isect = self.object_to_world.transform_surface_interaction(&mut s);

        *t_hit = t_shape_hit.into();
        true
    }

    fn intersect_p(&self, r: &Ray, _test_aphatexture: bool) -> bool {
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
        let a = dx * dx + dy * dy + dz * dz;
        let b = (dx * ox + dy * oy + dz * oz) * 2.0;
        let c = ox * ox + oy * oy + oz * oz - EFloat::from(self.radius) * EFloat::from(self.radius);

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

        // compute sphere hit position and phi
        p_hit = ray.find_point(t_shape_hit.into());

        // refine sphere intersection point
        p_hit *= p_hit.distance(&Point3::new(0.0, 0.0, 0.0)) / self.radius;

        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1e-5 as Float * self.radius;
        }

        phi = p_hit.y.atan2(p_hit.x);

        if phi < 0.0 {
            phi += 2.0 * phi;
        }

        // test sphere intersection against clipping parameters
        if (self.zmin > -self.radius && p_hit.z < self.zmin) || (self.zmax < self.radius && p_hit.z > self.zmax) || phi > self.phi_max {
            if t_shape_hit == t1 {
                return false;
            }

            if t1.upper_bound() > ray.t_max {
                return false;
            }

            t_shape_hit = t1;

            // compute sphere hit position and phi
            p_hit = ray.find_point(t_shape_hit.into());

            // refine sphere intersection point
            p_hit *= p_hit.distance(&Point3::new(0.0, 0.0, 0.0)) / self.radius;

            if p_hit.x == 0.0 && p_hit.y == 0.0 {
                p_hit.x = 1e-5 as Float * self.radius;
            }

            phi = p_hit.y.atan2(p_hit.x);

            if phi < 0.0 {
                phi += 2.0 * phi;
            }

            if (self.zmin > -self.radius && p_hit.z < self.zmin) || (self.zmax < self.radius && p_hit.z > self.zmax) || phi > self.phi_max {
                return false
            }
        }

        true
    }

    fn area(&self) -> f32 {
        self.phi_max * self.radius * (self.zmax - self.zmin)
    }

    fn sample(&self, u: &Point2<f32>) -> Interactions {
        unimplemented!()
    }

    fn sample_interaction(&self, i: &Interactions, u: &Point2<f32>) -> Interactions {
        unimplemented!()
    }

    fn pdf(&self, i: &Interactions, wi: &Vector3<f32>) -> f32 {
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

pub fn create_sphere(o2w: Transform, w2o: Transform, rev_orien: bool, params: &ParamSet) -> Shapes {
    let radius = params.find_one_float("radius", 1.0);
    let zmin = params.find_one_float("zmin", -radius);
    let zmax = params.find_one_float("zmax", radius);
    let phi_max = params.find_one_float("phimax", 360.0);

    Sphere::new(o2w, w2o, rev_orien, radius, zmin, zmax, phi_max).into()
}