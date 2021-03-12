use crate::core::pbrt::{Float, clamp, radians, gamma, PI};
use crate::core::transform::Transform;
use crate::core::shape::{Shape, Shapes, shape_pdfwi};
use crate::core::geometry::point::{Point3, Point2f, Point3f};
use crate::core::interaction::{SurfaceInteraction, InteractionData};
use crate::core::geometry::bounds::{Bounds3f};
use crate::core::geometry::vector::{Vector3, Vector3f, vec3_coordinate_system};
use crate::core::geometry::ray::Ray;
use crate::core::efloat::{EFloat, quadratic};
use crate::core::geometry::normal::Normal3f;
use std::sync::Arc;
use crate::core::paramset::ParamSet;
use crate::core::sampling::{uniform_sample_sphere, uniform_cone_pdf};
use crate::core::geometry::geometry::{offset_ray_origin, spherical_direction_basis};

#[derive(Debug, Clone)]
pub struct Sphere {
    pub radius                   : Float,
    pub zmin                     : Float,
    pub zmax                     : Float,
    pub theta_min                : Float,
    pub theta_max                : Float,
    pub phi_max                  : Float,
    pub object_to_world          : Arc<Transform>,
    pub world_to_object          : Arc<Transform>,
    pub reverse_orientation      : bool,
    pub transform_swapshandedness: bool
}

impl Sphere {
    pub fn new(
        object_to_world: Arc<Transform>, world_to_object: Arc<Transform>,
        reverse_orientation: bool, radius: Float, zmin: Float,
        zmax: Float, phi_max: Float) -> Self {
        let sh = object_to_world.swaps_handedness();

        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            radius,
            zmin                     : clamp(zmin.min(zmax), -radius, radius),
            zmax                     : clamp(zmin.max(zmax), -radius, radius),
            theta_min                : clamp(zmin.min(zmax) / radius, -1.0, 1.0).acos(),
            theta_max                : clamp(zmin.max(zmax) / radius, -1.0, 1.0).acos(),
            phi_max                  : radians(clamp(phi_max, 0.0, 360.0)),
            transform_swapshandedness: sh
        }
    }
}

impl Shape for Sphere {
    fn object_bound(&self) -> Bounds3f {
        Bounds3f::from_points(
            Point3f::new(-self.radius, -self.radius, self.zmin),
            Point3f::new(self.radius, self.radius, self.zmax))
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32,
        isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, _s: Option<Arc<Shapes>>) -> bool {
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
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
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
        p_hit *= self.radius / p_hit.distance(&Point3::new(0.0, 0.0, 0.0));

        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1.0e-5 * self.radius;
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
            p_hit *= self.radius / p_hit.distance(&Point3::new(0.0, 0.0, 0.0));

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
        let d2pduu =  Vector3f::new(p_hit.x, p_hit.y, 0.0) * -self.phi_max * self.phi_max;
        let d2pduv = Vector3f::new(-sin_phi, cos_phi, 0.0) * (self.theta_max - self.theta_min) * p_hit.z * self.phi_max;
        let d2pdvv = Vector3f::new(p_hit.x, p_hit.y, p_hit.z) * -(self.theta_max - self.theta_min) * (self.theta_max - self.theta_min);

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
        let dndu = Normal3f::from(dpdu * (f * F - e * G) * inv_EGF2 + dpdv * (e * F - f * E) * inv_EGF2);
        let dndv = Normal3f::from(dpdu * (g * F - f * G) * inv_EGF2 + dpdv * (f * F - g * E) * inv_EGF2);

        // compute error bounds for sphere intersection
        let p_error = Vector3f::from(p_hit).abs() * gamma(5);

        // Initialize SurfaceInteraction from parametric information
        let mut s = SurfaceInteraction::new(
            &p_hit, &p_error, &Point2f::new(u, v), &-ray.d,
            &dpdu, &dpdv, &dndu, &dndv, ray.time, None);
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
        let dy = EFloat::new(ray.d.y, d_err.y);
        let dz = EFloat::new(ray.d.z, d_err.z);
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
        p_hit *=  self.radius / p_hit.distance(&Point3::new(0.0, 0.0, 0.0));

        if p_hit.x == 0.0 && p_hit.y == 0.0 {
            p_hit.x = 1.0e-5 * self.radius;
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
            p_hit *= self.radius / p_hit.distance(&Point3::new(0.0, 0.0, 0.0));

            if p_hit.x == 0.0 && p_hit.y == 0.0 {
                p_hit.x = 1.0e-5 * self.radius;
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

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let mut pobj = Point3f::new(0.0, 0.0, 0.0) + uniform_sample_sphere(u) * self.radius;
        let mut it = InteractionData {
            n: self.object_to_world.transform_normal(
                &Normal3f::new(pobj.x, pobj.y, pobj.z))
                .normalize(),
            ..Default::default()
        };
        if self.reverse_orientation { it.n *= -1.0; }
        // Reproject pobj to sphere surface and compute pobj_error
        pobj *= self.radius / pobj.distance(&Point3f::new(0.0, 0.0, 0.0));
        let pobj_error = Vector3f::from(pobj).abs() * gamma(5);
        it.p = self.object_to_world.transform_point_abs_error(&pobj, &pobj_error, &mut it.p_error);
        *pdf = 1.0 / self.area();

        it
    }

    fn sample_interaction(&self, i: &InteractionData, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let pcenter = self.object_to_world.transform_point(&Point3f::new(0.0, 0.0, 0.0));

        // Sample uniformly on sphere if pt is inside it
        let porigin = offset_ray_origin(&i.p, &i.p_error, &i.n, &(pcenter - i.p));
        if porigin.distance_squared(&pcenter) <= self.radius * self.radius {
            let intr = self.sample(u, pdf);
            let mut wi = intr.p - i.p;
            if wi.length_squared() == 0.0 {
                *pdf = 0.0;
            } else {
                // Convert from measure returned by Sample() call above to
                // solid angle measure
                wi = wi.normalize();
                *pdf *= i.p.distance_squared(&intr.p) / intr.n.abs_dot_vec(&(-wi));
            }

            if (*pdf).is_infinite() { *pdf = 0.0; }
            return intr;
        }

        // Sample sphere uniformly inside subtended cone

        // Compute coordinate system for sphere sampling
        let dc = i.p.distance(&pcenter);
        let invdc = 1.0 / dc;
        let wc = (pcenter - i.p) * invdc;
        let mut wcx = Vector3f::default();
        let mut wcy = Vector3f::default();
        vec3_coordinate_system(&wc, &mut wcx, &mut wcy);

        // Compute theta and phi values for sample in cone
        let sin_thetamax = self.radius * invdc;
        let sin_thetamax2 = sin_thetamax * sin_thetamax;
        let inv_sin_thetamax = 1.0 / sin_thetamax;
        let cos_thetamax = ((1.0 - sin_thetamax2).max(0.0)).sqrt();

        let mut cos_theta = (cos_thetamax - 1.0) * u[0] + 1.0;
        let mut sin_theta2 = 1.0 - cos_theta * cos_theta;

        if sin_thetamax2 < 0.00068523 {
            /* Fall back to a Taylor series expansion for small angles, where
           the standard approach suffers from severe cancellation errors */
            sin_theta2 = sin_thetamax2 * u[0];
            cos_theta = (1.0 - sin_theta2).sqrt();
        }

        // Compute angle alpha from center of sphere to sampled point of surface
        let cos_alpha = sin_theta2 * inv_sin_thetamax +
            cos_theta * ((1.0 - sin_theta2 * inv_sin_thetamax * inv_sin_thetamax).max(0.0)).sqrt();
        let sin_alpha = ((1.0 - cos_alpha * cos_alpha).max(0.0)).sqrt();
        let phi = u[1] * 2.0 * PI;

        // Compute surface normal and sampled point on sphere
        let nworld = spherical_direction_basis(sin_alpha, cos_alpha, phi, &(-wcx), &(-wcy), &(-wc));
        let pworld = pcenter + Point3f::new(nworld.x, nworld.y, nworld.z) * self.radius;

        // Return Interaction for sampled point on sphere
        let mut it = InteractionData::default();
        it.p = pworld;
        it.p_error = Vector3f::from(pworld).abs() * gamma(5);
        if self.reverse_orientation { it.n *= 1.0; }

        // Uniform cone PDF
        *pdf = 1.0 / (2.0 * PI * (1.0 - cos_thetamax));

        it
    }

    fn pdf_wi(&self, i: &InteractionData, wi: &Vector3<f32>) -> f32 {
        let pcenter = self.object_to_world.transform_point(&Point3f::new(0.0, 0.0, 0.0));
        // Return uniform PDF if point is inside sphere
        let porigin = offset_ray_origin(&i.p, &i.p_error, &i.n, &(pcenter - i.p));
        if porigin.distance_squared(&pcenter) <= self.radius * self.radius {
            return shape_pdfwi(self, i, wi);
        }

        // Compute general sphere pdf
        let sin_thetamax2 = self.radius * self.radius / i.p.distance_squared(&pcenter);
        let cos_thetamax = ((1.0 - sin_thetamax2).max(0.0)).sqrt();

        uniform_cone_pdf(cos_thetamax)
    }

    fn solid_angle(&self, p: &Point3f, _nsamples: usize) -> Float {
        let pcenter = self.object_to_world.transform_point(&Point3f::new(0.0, 0.0, 0.0));

        if p.distance_squared(&pcenter) <= self.radius * self.radius {
            return 4.0 * PI;
        }
        let sin_theta2 = self.radius * self.radius / p.distance_squared(&pcenter);
        let cos_theta = ((1.0 - sin_theta2).max(0.0)).sqrt();

        2.0 * PI * (1.0 - cos_theta)
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

pub fn create_sphere(o2w: Arc<Transform>, w2o: Arc<Transform>, rev_orien: bool, params: &ParamSet) -> Arc<Shapes> {
    let radius = params.find_one_float("radius", 1.0);
    let zmin = params.find_one_float("zmin", -radius);
    let zmax = params.find_one_float("zmax", radius);
    let phi_max = params.find_one_float("phimax", 360.0);

    Arc::new(Sphere::new(o2w, w2o, rev_orien, radius, zmin, zmax, phi_max).into())
}