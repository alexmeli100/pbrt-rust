use nalgebra::{Matrix4};
use crate::core::pbrt::*;
use crate::core::geometry::vector::*;
use crate::core::geometry::point::*;
use crate::core::geometry::normal::Normal3;
use crate::core::geometry::ray::*;
use crate::core::geometry::bounds::Bounds3f;
use super::quaternion::*;
use num::{Zero, One, Signed};
use std::ops::{Mul, Add, Div, Sub};
use std::fmt::Debug;
use crate::core::interaction::SurfaceInteraction;
use std::cell::Cell;
use std::sync::Arc;
use log::error;

pub fn solve_linearsystem_2x2(a: [[Float; 2]; 2], b: [Float; 2],
                              x0: &mut Float, x1: &mut Float) -> bool {
    let det = a[0][0] * a[1][1] - a[0][1] * a[1][0];

    if det.abs() < 1.0e-10 { return false; }

    *x0 = (a[1][1] * b[0] - a[0][1] * b[1]) / det;
    *x1 = (a[0][0] * b[1] - a[1][0] * b[0]) / det;

    if x0.is_nan() || x1.is_nan() { return false; }

    true
}

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct Transform {
    pub m       : Matrix4<Float>,
    pub m_inv   : Matrix4<Float>
}

impl Default for Transform {
    fn default() -> Self {
        Transform {
            m: Matrix4::identity(),
            m_inv: Matrix4::identity()
        }
    }
}

impl Transform {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn from_matrix_slice(m: &[Float]) -> Self {
        let m = Matrix4::from_row_slice(m);
        let m_inv = m.try_inverse().unwrap();

        Self { m, m_inv }
    }

    pub fn from_matrix(m: &Matrix4<Float>) -> Self {
        let inv = m.try_inverse().unwrap();

        Self {
            m: *m,
            m_inv: inv
        }
    }

    pub fn has_scale(&self) -> bool {
        macro_rules! not_one {
            ($x:expr) => { ($x < 0.999 || $x > 1.001) }
        }

        let la2 = self.transform_vector(&Vector3f::new(1.0 , 0.0, 0.0)).length_squared();
        let lb2 = self.transform_vector(&Vector3f::new(0.0 , 1.0, 0.0)).length_squared();
        let lc2 = self.transform_vector(&Vector3f::new(0.0 , 0.0, 1.0)).length_squared();

        not_one!(la2) || not_one!(lb2) || not_one!(lc2)
    }

    pub fn from_matrices(m: &Matrix4<Float>, m_inv: &Matrix4<Float>) -> Self {
        Self { m: *m, m_inv: *m_inv }
    }

    pub fn inverse(t: &Transform) -> Self {
        Self { m: t.m_inv, m_inv: t.m }
    }

    pub fn transpose(t: &Transform) -> Self {
        Self { m: t.m.transpose(), m_inv: t.m_inv.transpose() }
    }

    pub fn is_identity(&self) -> bool {
        self.m.is_identity(1e-10)
    }

    pub fn translate(delta: &Vector3f) -> Self {
        let m = Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, delta.x,
            0.0, 1.0, 0.0, delta.y,
            0.0, 0.0, 1.0, delta.z,
            0.0, 0.0, 0.0, 1.0
        ]);

        let m_inv = Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, -delta.x,
            0.0, 1.0, 0.0, -delta.y,
            0.0, 0.0, 1.0, -delta.z,
            0.0, 0.0, 0.0, 1.0
        ]);

        Self { m, m_inv }
    }

    pub fn scale(x: Float, y: Float, z: Float) -> Self {
        let m = Matrix4::from_row_slice(&[
            x,   0.0, 0.0, 0.0,
            0.0, y,   0.0, 0.0,
            0.0, 0.0, z,   0.0,
            0.0, 0.0, 0.0, 1.0
        ]);

        let m_inv = Matrix4::from_row_slice(&[
            1.0/x, 0.0,   0.0,   0.0,
            0.0,   1.0/y, 0.0,   0.0,
            0.0,   0.0,   1.0/z, 0.0,
            0.0,   0.0,   0.0,   1.0
        ]);

        Self { m, m_inv }
    }

    pub fn rotate_x(theta: Float) -> Self {
        let sin_theta = radians(theta).sin();
        let cos_theta = radians(theta).cos();

        let m = Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, 0.0,
            0.0, cos_theta, -sin_theta, 0.0,
            0.0, cos_theta, sin_theta, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]);

        Self { m, m_inv: m.transpose() }
    }

    pub fn rotate_y(theta: Float) -> Self {
        let sin_theta = radians(theta).sin();
        let cos_theta = radians(theta).cos();

        let m = Matrix4::from_row_slice(&[
            cos_theta, 0.0, sin_theta, 0.0,
            0.0, 1.0, 0.0, 0.0,
            -sin_theta, 0.0, cos_theta, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]);

        Self { m, m_inv: m.transpose() }
    }

    pub fn rotate_z(theta: Float) -> Self {
        let sin_theta = radians(theta).sin();
        let cos_theta = radians(theta).cos();

        let m = Matrix4::from_row_slice(&[
            cos_theta, -sin_theta, 0.0, 0.0,
            sin_theta, cos_theta, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]);

        Self { m, m_inv: m.transpose() }
    }

    pub fn rotate(theta: Float, axis: &Vector3f) -> Self {
        let a = axis.normalize();
        let sin_theta = radians(theta).sin();
        let cos_theta = radians(theta).cos();
        let mut m: Matrix4<Float> = Matrix4::identity();

        m[(0, 0)] = a.x * a.x + (1.0 - a.x * a.x) * cos_theta;
        m[(0, 1)] = a.x * a.y * (1.0 - cos_theta) - a.z * sin_theta;
        m[(0, 2)] = a.x * a.z * (1.0 - cos_theta) + a.y * sin_theta;
        m[(0, 3)] = 0.0;

        m[(1, 0)] = a.x * a.y * (1.0 - cos_theta) + a.z * sin_theta;
        m[(1, 1)] = a.y * a.y + (1.0 - a.y * a.y) * cos_theta;
        m[(1, 2)] = a.y * a.z * (1.0 - cos_theta) - a.x * sin_theta;
        m[(1, 3)] = 0.0;

        m[(2, 0)] = a.x * a.z * (1.0 - cos_theta) - a.y * sin_theta;
        m[(2, 1)] = a.y * a.z * (1.0 - cos_theta) + a.x * sin_theta;
        m[(2, 2)] = a.z * a.z + (1.0 - a.z * a.z) * cos_theta;
        m[(2, 3)] = 0.0;

        Self { m, m_inv: m.transpose()}
    }

    pub fn look_at(pos: &Point3f, look: &Point3f, up: &Vector3f) -> Self {
        let mut camera_to_world: Matrix4<Float> = Matrix4::identity();

        camera_to_world[(0, 3)] = pos.x;
        camera_to_world[(1, 3)] = pos.y;
        camera_to_world[(2, 3)] = pos.z;
        camera_to_world[(3, 3)] = 1.0;

        let dir = (*look - *pos).normalize();

        if (up.normalize().cross(&dir)).length() == 0.0 {
            error!(
                "\"up\" vector ({}, {}, {}) and viewing direction ({}, {}, {}) \
                passed to LookAt are pointing in the same direction. Using\
                the identity transformation.",
                up.x, up.y, up.z, dir.x, dir.y, dir.z);
            return Default::default();
        }

        let right = up.normalize().cross(&dir).normalize();
        let new_up = dir.cross(&right);

        camera_to_world[(0,0)] = right.x;
        camera_to_world[(1,0)] = right.y;
        camera_to_world[(2,0)] = right.z;
        camera_to_world[(3,0)] = 0.0;
        camera_to_world[(0,1)] = new_up.x;
        camera_to_world[(1,1)] = new_up.y;
        camera_to_world[(2,1)] = new_up.z;
        camera_to_world[(3,1)] = 0.0;
        camera_to_world[(0,2)] = dir.x;
        camera_to_world[(1,2)] = dir.y;
        camera_to_world[(2,2)] = dir.z;
        camera_to_world[(3,2)] = 0.0;

        Self { m: camera_to_world.try_inverse().unwrap(), m_inv: camera_to_world}
    }

    pub fn orthographic(znear: Float, zfar: Float) -> Self {
        Transform::scale(1.0, 1.0, 1.0 / (zfar - znear)) * Transform::translate(&Vector3f::new(0.0, 0.0, -znear))
    }

    pub fn perspective(fov: Float, n: Float, f: Float) -> Self {
        // Perform projective divide for projection
        let persp = Matrix4::from_row_slice(&[
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, f / (f - n), -f * n / (f - n),
            0.0, 0.0, 1.0, 0.0
        ]);

        let inv_tan_ang = 1.0 / (radians(fov) / 2.0).tan();

        Transform::scale(inv_tan_ang, inv_tan_ang, 1.0) * Transform::from_matrix(&persp)
    }

    pub fn transform_point(&self, p: &Point3f) -> Point3f
    {
        let x = p.x;
        let y = p.y;
        let z = p.z;

        let xp = x*self.m[(0, 0)] + y*self.m[(0, 1)] + z*self.m[(0, 2)] + self.m[(0, 3)];
        let yp = x*self.m[(1, 0)] + y*self.m[(1, 1)] + z*self.m[(1, 2)] + self.m[(1, 3)];
        let zp = x*self.m[(2, 0)] + y*self.m[(2, 1)] + z*self.m[(2, 2)] + self.m[(2, 3)];
        let wp = x*self.m[(3, 0)] + y*self.m[(3, 1)] + z*self.m[(3, 2)] + self.m[(3, 3)];

        assert_ne!(wp, 0.0);

        if wp == 1.0 {
            Point3::new(xp, yp, zp)
        } else {
            Point3::new(xp, yp, zp) / wp
        }
    }

    pub fn transform_point_error(&self, p: &Point3f, p_error: &mut Vector3f) -> Point3f
    {
        let x = p.x;
        let y = p.y;
        let z = p.z;

        let xp = x*self.m[(0,0)] + y*self.m[(0,1)] + z*self.m[(0,2)] + self.m[(0,3)];
        let yp = x*self.m[(1,0)] + y*self.m[(1,1)] + z*self.m[(1,2)] + self.m[(1,3)];
        let zp = x*self.m[(2,0)] + y*self.m[(2,1)] + z*self.m[(2,2)] + self.m[(2,3)];
        let wp = x*self.m[(3,0)] + y*self.m[(3,1)] + z*self.m[(3,2)] + self.m[(3,3)];

        let x_abs_sum = (x*self.m[(0, 0)]).abs() + (y*self.m[(0, 1)]).abs() + (z*self.m[(0, 2)]).abs() + (self.m[(0, 3)]).abs();
        let y_abs_sum = (x*self.m[(1, 0)]).abs() + (y*self.m[(1, 1)]).abs() + (z*self.m[(1, 2)]).abs() + (self.m[(1, 3)]).abs();
        let z_abs_sum = (x*self.m[(2, 0)]).abs() + (y*self.m[(2, 1)]).abs() + (z*self.m[(2, 2)]).abs() + (self.m[(2, 3)]).abs();

        *p_error = Vector3::new(x_abs_sum, y_abs_sum, z_abs_sum) * gamma(3);
        assert_ne!(wp, 0.0);

        if wp == 1.0 {
            Point3::new(xp, yp, zp)
        } else {
            Point3::new(xp, yp, zp) / wp
        }
    }

    pub fn transform_point_abs_errorf(&self, p: &Point3f, p_error: &Vector3f, abs_error: &mut Vector3f) -> Point3f {
        let x = p.x;
        let y = p.y;
        let z = p.z;

        let xp = x*self.m[(0,0)] + y*self.m[(0,1)] + z*self.m[(0,2)] + self.m[(0,3)];
        let yp = x*self.m[(1,0)] + y*self.m[(1,1)] + z*self.m[(1,2)] + self.m[(1,3)];
        let zp = x*self.m[(2,0)] + y*self.m[(2,1)] + z*self.m[(2,2)] + self.m[(2,3)];
        let wp = x*self.m[(3,0)] + y*self.m[(3,1)] + z*self.m[(3,2)] + self.m[(3,3)];

        abs_error.x =
            (gamma(3) + 1.0) *
                ((self.m[(0, 0)]).abs() * p_error.x + (self.m[(0, 1)]).abs() * p_error.y +
                 (self.m[(0, 2)]).abs() * p_error.x) +
            gamma(3) * (self.m[(0, 0)] * x).abs() + (self.m[(0, 1)] * y).abs() +
                (self.m[(0, 2)] * z).abs() + (self.m[(0, 3)]).abs();
        abs_error.y =
            (gamma(3) + 1.0) *
            ((self.m[(1, 0)]).abs() * p_error.x + (self.m[(1, 1)]).abs() * p_error.y +
                (self.m[(1, 2)]).abs() * p_error.z) +
            gamma(3) * ((self.m[(1, 0)] * x).abs() + (self.m[(1, 1)] * y).abs() +
                (self.m[(1, 2)] * z).abs() + (self.m[(1, 3)]).abs());
        abs_error.z =
            (gamma(3) + 1.0) *
            ((self.m[(2, 0)]).abs() * p_error.x + (self.m[(2, 1)]).abs() * p_error.y +
                (self.m[(2, 2)]).abs() * p_error.z) +
            gamma(3) * ((self.m[(2, 0)] * x).abs() + (self.m[(2, 1)] * y).abs() +
                (self.m[(2, 2)] * z).abs() + (self.m[(2, 3)]).abs());

        assert_ne!(wp, 0.0);

        if wp == 1.0 {
            Point3f::new(xp, yp, zp)
        } else {
            Point3f::new(xp, yp, zp) / wp
        }
    }

    pub fn transform_vector<T>(&self, v: &Vector3<T>) -> Vector3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T> + Copy
    {
        let x = v.x;
        let y = v.y;
        let z = v.z;

        Vector3::new(
            x*self.m[(0, 0)] + y*self.m[(0, 1)] + z*self.m[(0, 2)],
            x*self.m[(1, 0)] + y*self.m[(1, 1)] + z*self.m[(1, 2)],
            x*self.m[(2, 0)] + y*self.m[(2, 1)] + z*self.m[(2, 2)]
        )
    }

    pub fn transform_vector_error<T>(&self, v: &Vector3<T>, abs_error: &mut Vector3<T>) -> Vector3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T> + Add<Float, Output=T> + Div<T, Output=T> + Copy + Signed + One + PartialEq + Zero + Debug + From<Float>

    {
        let x = v.x;
        let y = v.y;
        let z = v.z;

        abs_error.x = T::from(gamma(3)) * (x*self.m[(0, 0)]).abs() + (y*self.m[(0, 1)]).abs() + (z*self.m[(0, 2)]).abs() + (self.m[(0, 3)]).abs();
        abs_error.y = T::from(gamma(3)) * (x*self.m[(1, 0)]).abs() + (y*self.m[(1, 1)]).abs() + (z*self.m[(1, 2)]).abs() + (self.m[(1, 3)]).abs();
        abs_error.z = T::from(gamma(3)) * (x*self.m[(2, 0)]).abs() + (y*self.m[(2, 1)]).abs() + (z*self.m[(2, 2)]).abs() + (self.m[(2, 3)]).abs();

        Vector3::new(
            x*self.m[(0, 0)] + y*self.m[(0, 1)] + z*self.m[(0, 2)],
            x*self.m[(1, 0)] + y*self.m[(1, 1)] + z*self.m[(1, 2)],
            x*self.m[(2, 0)] + y*self.m[(2, 1)] + z*self.m[(2, 2)]
        )
    }

    pub fn transform_normal<T>(&self, n: &Normal3<T>) -> Normal3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T> + Copy
    {
        let x = n.x;
        let y = n.y;
        let z = n.z;

        Normal3::new(
            x*self.m[(0, 0)] + y*self.m[(1, 0)] + z*self.m[(2, 0)],
            x*self.m[(0, 1)] + y*self.m[(1, 1)] + z*self.m[(2, 1)],
            x*self.m[(0, 2)] + y*self.m[(1, 2)] + z*self.m[(2, 2)]
        )
    }

    pub fn transform_ray(&self, r: &Ray) -> Ray {
        let mut o_error = Vector3f::default();
        let mut o = self.transform_point_error(&r.o, &mut o_error);
        let d = self.transform_vector(&r.d);

        let l_squared = d.length_squared();
        let mut t_max = r.t_max;

        if l_squared > 0.0 {
            let dt = d.abs().dot(&o_error) / l_squared;
            o += d * dt;
            t_max -= dt;
        }

        let mut medium = None;
        let mut diff = None;

        if let Some(ref m) = r.medium {
            medium = Some(m.clone());
        }

        if let Some(ref d) = r.diff {
            let di = RayDifferential {
                rx_origin: self.transform_point(&d.rx_origin),
                ry_origin: self.transform_point(&d.ry_origin),
                rx_direction: self.transform_vector(&d.rx_direction),
                ry_direction: self.transform_vector(&d.ry_direction),
                has_differentials: d.has_differentials
            };

            diff = Some(di)
        }

        Ray::new(&o, &d, t_max, r.time, medium, diff)
    }

    pub fn transform_ray_error(&self, r: &Ray, o_error: &mut Vector3f, d_error: &mut Vector3f) -> Ray {
        let mut o = self.transform_point_error(&r.o, o_error);
        let d = self.transform_vector_error(&r.d, d_error);
        let t_max = r.t_max;
        let length_squared = d.length_squared();

        if length_squared > 0.0 {
            let dt = d.abs().dot(o_error) / length_squared;
            o += d * dt;
        }

        Ray::new(&o, &d, t_max, r.time, r.medium.clone(), None)
    }

    pub fn transform_bounds(&self, b: &Bounds3f) -> Bounds3f {
        let mut ret = Bounds3f::from_point(&self.transform_point(&Point3::new(b.p_min.x, b.p_min.y, b.p_min.z)));

        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_max.x, b.p_min.y, b.p_min.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_min.x, b.p_max.y, b.p_min.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_min.x, b.p_min.y, b.p_max.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_min.x, b.p_max.y, b.p_max.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_max.x, b.p_max.y, b.p_min.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_max.x, b.p_min.y, b.p_max.z)));
        ret = ret.union_point(&self.transform_point(&Point3::new(b.p_max.x, b.p_max.y, b.p_max.z)));

        ret
    }

    pub fn transform_surface_interaction<'a>(&self, si: &SurfaceInteraction<'a>) -> SurfaceInteraction<'a> {
        let mut ret = SurfaceInteraction::<'a>::default();
        ret.p = self.transform_point_abs_errorf(&si.p, &si.p_error, &mut ret.p_error);
        ret.n = self.transform_normal(&si.n).normalize();
        ret.wo = self.transform_vector(&si.wo);
        ret.time = si.time;
        ret.uv = si.uv;
        ret.dpdu = self.transform_vector(&si.dpdu);
        ret.dpdv = self.transform_vector(&si.dpdv);
        ret.dndu = self.transform_normal(&si.dndu);
        ret.dndv = self.transform_normal(&si.dndv);
        ret.shading.n = self.transform_normal(&si.shading.n).normalize();
        ret.shading.dpdu = self.transform_vector(&si.shading.dpdu);
        ret.shading.dpdv = self.transform_vector(&si.shading.dpdv);
        ret.shading.dndu = self.transform_normal(&si.shading.dndu);
        ret.shading.dndv = self.transform_normal(&si.shading.dndv);
        ret.shading.n = ret.shading.n.face_foward(&ret.n);
        ret.dudx = Cell::new(si.dudx.get());
        ret.dvdx = Cell::new(si.dvdx.get());
        ret.dudy = Cell::new(si.dudy.get());
        ret.dvdy = Cell::new(si.dvdy.get());
        ret.dpdx = Cell::new(self.transform_vector(&si.dpdx.get()));
        ret.dpdy = Cell::new(self.transform_vector(&si.dpdy.get()));
        ret.shape = si.get_shape();
        ret.primitive = si.get_primitive();
        ret.bsdf = None;
        ret.bssrdf = si.get_bssrdf();

        ret
    }

    pub fn swaps_handedness(&self) -> bool {
        let det = self.m[(0, 0)] * (self.m[(1, 1)] * self.m[(2, 2)] - self.m[(1, 2)] * self.m[(2, 1)]) -
                       self.m[(0, 1)] * (self.m[(1, 0)] * self.m[(2, 2)] - self.m[(1, 2)] * self.m[(2, 0)]) +
                       self.m[(0, 2)] * (self.m[(1, 0)] * self.m[(2, 1)] - self.m[(1, 1)] * self.m[(2, 0)]);

        det < 0.0
    }
}

impl Mul for Transform {
    type Output = Transform;

    fn mul(self, other: Self) -> Self::Output {
        Transform {
            m: self.m * other.m,
            m_inv: other.m_inv * self.m_inv
        }
    }
}

impl From<Quaternion> for Transform {
    fn from(q: Quaternion) -> Self {
        let xx = q.v.x * q.v.x;
        let yy = q.v.y * q.v.y;
        let zz = q.v.z * q.v.z;
        let xy = q.v.x * q.v.y;
        let xz = q.v.x * q.v.z;
        let yz = q.v.y * q.v.z;
        let wx = q.v.x * q.w;
        let wy = q.v.y * q.w;
        let wz = q.v.z * q.w;

        let mut m = Matrix4::identity();

        m[(0, 0)] = 1.0 - 2.0 * (yy + zz);
        m[(0, 1)] = 2.0 * (xy + wz);
        m[(0, 2)] = 2.0 * (xz - wy);
        m[(1, 0)] = 2.0 * (xy - wz);
        m[(1, 1)] = 1.0 - 2.0 * (xx + zz);
        m[(1, 2)] = 2.0 * (yz + wx);
        m[(2, 0)] = 2.0 * (xz + wy);
        m[(2, 1)] = 2.0 * (yz - wx);
        m[(2, 2)] = 1.0 - 2.0 * (xx + yy);

        Transform::from_matrices(&m.transpose(), &m)
    }
}

#[derive(Debug, Clone)]
pub struct AnimatedTransform {
    start_transform     : Arc<Transform>,
    end_transform       : Arc<Transform>,
    start_time          : Float,
    end_time            : Float,
    actually_animated   : bool,
    has_rotation        : bool,
    t                   : [Vector3f; 2],
    r                   : [Quaternion; 2],
    s                   : [Matrix4<Float>; 2],
    c1                  : [DerivativeTerm; 3],
    c2                  : [DerivativeTerm; 3],
    c3                  : [DerivativeTerm; 3],
    c4                  : [DerivativeTerm; 3],
    c5                  : [DerivativeTerm; 3]
}

impl Default for AnimatedTransform {
    fn default() -> Self {
        Self {
            start_transform: Arc::new(Default::default()),
            end_transform: Arc::new(Default::default()),
            start_time: 0.0,
            end_time: 0.0,
            actually_animated: false,
            has_rotation: false,
            t: [Vector3f::default(); 2],
            r: [Quaternion::default(); 2],
            s: [Matrix4::identity(); 2],
            c1: [DerivativeTerm::default(); 3],
            c2: [DerivativeTerm::default(); 3],
            c3: [DerivativeTerm::default(); 3],
            c4: [DerivativeTerm::default(); 3],
            c5: [DerivativeTerm::default(); 3]
        }
    }
}

impl AnimatedTransform {
    pub fn new(start_transform: Arc<Transform>, end_transform: Arc<Transform>, start_time: Float, end_time: Float) -> Self {
        let mut res = AnimatedTransform {
            start_transform: start_transform.clone(),
            end_transform: end_transform.clone(),
            start_time, end_time,
            actually_animated: *start_transform != *end_transform,
            has_rotation: false,
            t: [Vector3f::default(); 2],
            r: [Quaternion::default(); 2],
            s: [Matrix4::identity(); 2],
            c1: [DerivativeTerm::default(); 3],
            c2: [DerivativeTerm::default(); 3],
            c3: [DerivativeTerm::default(); 3],
            c4: [DerivativeTerm::default(); 3],
            c5: [DerivativeTerm::default(); 3]
        };

        //println!("starttime: {}", start_time);

        Self::decompose(&res.start_transform.m, &mut res.t[0], &mut res.r[0], &mut res.s[0]);
        Self::decompose(&res.end_transform.m, &mut res.t[1], &mut res.r[1], &mut res.s[1]);

        //println!("endtime: {}", start_time);
        // Flip R1[1] if needed to select shortest path
        if res.r[0].dot(&res.r[1]) < 0.0 { res.r[1] = -res.r[1]; }
        res.has_rotation = res.r[0].dot(&res.r[1]) < 0.9995;

        if res.has_rotation {
            let cos_theta = res.r[0].dot(&res.r[1]);
            let theta = clamp(cos_theta, -1.0, 1.0).acos();
            let qperp = (res.r[1] - res.r[0] * cos_theta).normalize();

            let t0x = res.t[0].x;
            let t0y = res.t[0].y;
            let t0z = res.t[0].z;
            let t1x = res.t[1].x;
            let t1y = res.t[1].y;
            let t1z = res.t[1].z;
            let q0x = res.r[0].v.x;
            let q0y = res.r[0].v.y;
            let q0z = res.r[0].v.z;
            let q0w = res.r[0].w;
            let qperpx = qperp.v.x;
            let qperpy = qperp.v.y;
            let qperpz = qperp.v.z;
            let qperpw = qperp.w;
            let s000 = res.s[0][(0, 0)];
            let s001 = res.s[0][(0, 1)];
            let s002 = res.s[0][(0, 2)];
            let s010 = res.s[0][(1, 0)];
            let s011 = res.s[0][(1, 1)];
            let s012 = res.s[0][(1, 2)];
            let s020 = res.s[0][(2, 0)];
            let s021 = res.s[0][(2, 1)];
            let s022 = res.s[0][(2, 2)];
            let s100 = res.s[1][(0, 0)];
            let s101 = res.s[1][(0, 1)];
            let s102 = res.s[1][(0, 2)];
            let s110 = res.s[1][(1, 0)];
            let s111 = res.s[1][(1, 1)];
            let s112 = res.s[1][(1, 2)];
            let s120 = res.s[1][(2, 0)];
            let s121 = res.s[1][(2, 1)];
            let s122 = res.s[1][(2, 2)];

            res.c1[0] = DerivativeTerm::new(
                -t0x + t1x,
                (-1.0 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s000 +
                    q0w * q0z * s010 - qperpx * qperpy * s010 +
                    qperpw * qperpz * s010 - q0w * q0y * s020 -
                    qperpw * qperpy * s020 - qperpx * qperpz * s020 + s100 -
                    q0y * q0y * s100 - q0z * q0z * s100 - qperpy * qperpy * s100 -
                    qperpz * qperpz * s100 - q0w * q0z * s110 +
                    qperpx * qperpy * s110 - qperpw * qperpz * s110 +
                    q0w * q0y * s120 + qperpw * qperpy * s120 +
                    qperpx * qperpz * s120 +
                    q0x * (-(q0y * s010) - q0z * s020 + q0y * s110 + q0z * s120),
                (-1.0 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s001 +
                    q0w * q0z * s011 - qperpx * qperpy * s011 +
                    qperpw * qperpz * s011 - q0w * q0y * s021 -
                    qperpw * qperpy * s021 - qperpx * qperpz * s021 + s101 -
                    q0y * q0y * s101 - q0z * q0z * s101 - qperpy * qperpy * s101 -
                    qperpz * qperpz * s101 - q0w * q0z * s111 +
                    qperpx * qperpy * s111 - qperpw * qperpz * s111 +
                    q0w * q0y * s121 + qperpw * qperpy * s121 +
                    qperpx * qperpz * s121 +
                    q0x * (-(q0y * s011) - q0z * s021 + q0y * s111 + q0z * s121),
                (-1.0 + q0y * q0y + q0z * q0z + qperpy * qperpy + qperpz * qperpz) *
                    s002 +
                    q0w * q0z * s012 - qperpx * qperpy * s012 +
                    qperpw * qperpz * s012 - q0w * q0y * s022 -
                    qperpw * qperpy * s022 - qperpx * qperpz * s022 + s102 -
                    q0y * q0y * s102 - q0z * q0z * s102 - qperpy * qperpy * s102 -
                    qperpz * qperpz * s102 - q0w * q0z * s112 +
                    qperpx * qperpy * s112 - qperpw * qperpz * s112 +
                    q0w * q0y * s122 + qperpw * qperpy * s122 +
                    qperpx * qperpz * s122 +
                    q0x * (-(q0y * s012) - q0z * s022 + q0y * s112 + q0z * s122));

            res.c2[0] = DerivativeTerm::new(
                0.0,
                -(qperpy * qperpy * s000) - qperpz * qperpz * s000 +
                    qperpx * qperpy * s010 - qperpw * qperpz * s010 +
                    qperpw * qperpy * s020 + qperpx * qperpz * s020 +
                    q0y * q0y * (s000 - s100) + q0z * q0z * (s000 - s100) +
                    qperpy * qperpy * s100 + qperpz * qperpz * s100 -
                    qperpx * qperpy * s110 + qperpw * qperpz * s110 -
                    qperpw * qperpy * s120 - qperpx * qperpz * s120 +
                    2.0 * q0x * qperpy * s010 * theta -
                    2.0 * q0w * qperpz * s010 * theta +
                    2.0 * q0w * qperpy * s020 * theta +
                    2.0 * q0x * qperpz * s020 * theta +
                    q0y *
                        (q0x * (-s010 + s110) + q0w * (-s020 + s120) +
                            2.0 * (-2.0 * qperpy * s000 + qperpx * s010 + qperpw * s020) *
                                theta) +
                    q0z * (q0w * (s010 - s110) + q0x * (-s020 + s120) -
                        2.0 * (2.0 * qperpz * s000 + qperpw * s010 - qperpx * s020) *
                            theta),
                -(qperpy * qperpy * s001) - qperpz * qperpz * s001 +
                    qperpx * qperpy * s011 - qperpw * qperpz * s011 +
                    qperpw * qperpy * s021 + qperpx * qperpz * s021 +
                    q0y * q0y * (s001 - s101) + q0z * q0z * (s001 - s101) +
                    qperpy * qperpy * s101 + qperpz * qperpz * s101 -
                    qperpx * qperpy * s111 + qperpw * qperpz * s111 -
                    qperpw * qperpy * s121 - qperpx * qperpz * s121 +
                    2.0 * q0x * qperpy * s011 * theta -
                    2.0 * q0w * qperpz * s011 * theta +
                    2.0 * q0w * qperpy * s021 * theta +
                    2.0 * q0x * qperpz * s021 * theta +
                    q0y *
                        (q0x * (-s011 + s111) + q0w * (-s021 + s121) +
                            2.0 * (-2.0 * qperpy * s001 + qperpx * s011 + qperpw * s021) *
                                theta) +
                    q0z * (q0w * (s011 - s111) + q0x * (-s021 + s121) -
                        2.0 * (2.0 * qperpz * s001 + qperpw * s011 - qperpx * s021) *
                            theta),
                -(qperpy * qperpy * s002) - qperpz * qperpz * s002 +
                    qperpx * qperpy * s012 - qperpw * qperpz * s012 +
                    qperpw * qperpy * s022 + qperpx * qperpz * s022 +
                    q0y * q0y * (s002 - s102) + q0z * q0z * (s002 - s102) +
                    qperpy * qperpy * s102 + qperpz * qperpz * s102 -
                    qperpx * qperpy * s112 + qperpw * qperpz * s112 -
                    qperpw * qperpy * s122 - qperpx * qperpz * s122 +
                    2.0 * q0x * qperpy * s012 * theta -
                    2.0 * q0w * qperpz * s012 * theta +
                    2.0 * q0w * qperpy * s022 * theta +
                    2.0 * q0x * qperpz * s022 * theta +
                    q0y *
                        (q0x * (-s012 + s112) + q0w * (-s022 + s122) +
                            2.0 * (-2.0 * qperpy * s002 + qperpx * s012 + qperpw * s022) *
                                theta) +
                    q0z * (q0w * (s012 - s112) + q0x * (-s022 + s122) -
                        2.0 * (2.0 * qperpz * s002 + qperpw * s012 - qperpx * s022) *
                            theta));

            res.c3[0] = DerivativeTerm::new(
                0.,
                -2.0 * (q0x * qperpy * s010 - q0w * qperpz * s010 +
                    q0w * qperpy * s020 + q0x * qperpz * s020 -
                    q0x * qperpy * s110 + q0w * qperpz * s110 -
                    q0w * qperpy * s120 - q0x * qperpz * s120 +
                    q0y * (-2.0 * qperpy * s000 + qperpx * s010 + qperpw * s020 +
                        2.0 * qperpy * s100 - qperpx * s110 - qperpw * s120) +
                    q0z * (-2.0 * qperpz * s000 - qperpw * s010 + qperpx * s020 +
                        2.0 * qperpz * s100 + qperpw * s110 - qperpx * s120)) *
                    theta,
                -2.0 * (q0x * qperpy * s011 - q0w * qperpz * s011 +
                    q0w * qperpy * s021 + q0x * qperpz * s021 -
                    q0x * qperpy * s111 + q0w * qperpz * s111 -
                    q0w * qperpy * s121 - q0x * qperpz * s121 +
                    q0y * (-2.0 * qperpy * s001 + qperpx * s011 + qperpw * s021 +
                        2.0 * qperpy * s101 - qperpx * s111 - qperpw * s121) +
                    q0z * (-2.0 * qperpz * s001 - qperpw * s011 + qperpx * s021 +
                        2.0 * qperpz * s101 + qperpw * s111 - qperpx * s121)) *
                    theta,
                -2.0 * (q0x * qperpy * s012 - q0w * qperpz * s012 +
                    q0w * qperpy * s022 + q0x * qperpz * s022 -
                    q0x * qperpy * s112 + q0w * qperpz * s112 -
                    q0w * qperpy * s122 - q0x * qperpz * s122 +
                    q0y * (-2.0 * qperpy * s002 + qperpx * s012 + qperpw * s022 +
                        2.0 * qperpy * s102 - qperpx * s112 - qperpw * s122) +
                    q0z * (-2.0 * qperpz * s002 - qperpw * s012 + qperpx * s022 +
                        2.0 * qperpz * s102 + qperpw * s112 - qperpx * s122)) *
                    theta);

            res.c4[0] = DerivativeTerm::new(
                0.0,
                -(q0x * qperpy * s010) + q0w * qperpz * s010 - q0w * qperpy * s020 -
                    q0x * qperpz * s020 + q0x * qperpy * s110 -
                    q0w * qperpz * s110 + q0w * qperpy * s120 +
                    q0x * qperpz * s120 + 2.0 * q0y * q0y * s000 * theta +
                    2.0 * q0z * q0z * s000 * theta -
                    2.0 * qperpy * qperpy * s000 * theta -
                    2.0 * qperpz * qperpz * s000 * theta +
                    2.0 * qperpx * qperpy * s010 * theta -
                    2.0 * qperpw * qperpz * s010 * theta +
                    2.0 * qperpw * qperpy * s020 * theta +
                    2.0 * qperpx * qperpz * s020 * theta +
                    q0y * (-(qperpx * s010) - qperpw * s020 +
                        2.0 * qperpy * (s000 - s100) + qperpx * s110 +
                        qperpw * s120 - 2.0 * q0x * s010 * theta -
                        2.0 * q0w * s020 * theta) +
                    q0z * (2.0 * qperpz * s000 + qperpw * s010 - qperpx * s020 -
                        2.0 * qperpz * s100 - qperpw * s110 + qperpx * s120 +
                        2.0 * q0w * s010 * theta - 2.0 * q0x * s020 * theta),
                -(q0x * qperpy * s011) + q0w * qperpz * s011 - q0w * qperpy * s021 -
                    q0x * qperpz * s021 + q0x * qperpy * s111 -
                    q0w * qperpz * s111 + q0w * qperpy * s121 +
                    q0x * qperpz * s121 + 2.0 * q0y * q0y * s001 * theta +
                    2.0 * q0z * q0z * s001 * theta -
                    2.0 * qperpy * qperpy * s001 * theta -
                    2.0 * qperpz * qperpz * s001 * theta +
                    2.0 * qperpx * qperpy * s011 * theta -
                    2.0 * qperpw * qperpz * s011 * theta +
                    2.0 * qperpw * qperpy * s021 * theta +
                    2.0 * qperpx * qperpz * s021 * theta +
                    q0y * (-(qperpx * s011) - qperpw * s021 +
                        2.0 * qperpy * (s001 - s101) + qperpx * s111 +
                        qperpw * s121 - 2.0 * q0x * s011 * theta -
                        2.0 * q0w * s021 * theta) +
                    q0z * (2.0 * qperpz * s001 + qperpw * s011 - qperpx * s021 -
                        2.0 * qperpz * s101 - qperpw * s111 + qperpx * s121 +
                        2.0 * q0w * s011 * theta - 2.0 * q0x * s021 * theta),
                -(q0x * qperpy * s012) + q0w * qperpz * s012 - q0w * qperpy * s022 -
                    q0x * qperpz * s022 + q0x * qperpy * s112 -
                    q0w * qperpz * s112 + q0w * qperpy * s122 +
                    q0x * qperpz * s122 + 2.0 * q0y * q0y * s002 * theta +
                    2.0 * q0z * q0z * s002 * theta -
                    2.0 * qperpy * qperpy * s002 * theta -
                    2.0 * qperpz * qperpz * s002 * theta +
                    2.0 * qperpx * qperpy * s012 * theta -
                    2.0 * qperpw * qperpz * s012 * theta +
                    2.0 * qperpw * qperpy * s022 * theta +
                    2.0 * qperpx * qperpz * s022 * theta +
                    q0y * (-(qperpx * s012) - qperpw * s022 +
                        2.0 * qperpy * (s002 - s102) + qperpx * s112 +
                        qperpw * s122 - 2.0 * q0x * s012 * theta -
                        2.0 * q0w * s022 * theta) +
                    q0z * (2.0 * qperpz * s002 + qperpw * s012 - qperpx * s022 -
                        2.0 * qperpz * s102 - qperpw * s112 + qperpx * s122 +
                        2.0 * q0w * s012 * theta - 2.0 * q0x * s022 * theta));

            res.c5[0] = DerivativeTerm::new(
                0.0,
                2.0 * (qperpy * qperpy * s000 + qperpz * qperpz * s000 -
                    qperpx * qperpy * s010 + qperpw * qperpz * s010 -
                    qperpw * qperpy * s020 - qperpx * qperpz * s020 -
                    qperpy * qperpy * s100 - qperpz * qperpz * s100 +
                    q0y * q0y * (-s000 + s100) + q0z * q0z * (-s000 + s100) +
                    qperpx * qperpy * s110 - qperpw * qperpz * s110 +
                    q0y * (q0x * (s010 - s110) + q0w * (s020 - s120)) +
                    qperpw * qperpy * s120 + qperpx * qperpz * s120 +
                    q0z * (-(q0w * s010) + q0x * s020 + q0w * s110 - q0x * s120)) *
                    theta,
                2.0 * (qperpy * qperpy * s001 + qperpz * qperpz * s001 -
                    qperpx * qperpy * s011 + qperpw * qperpz * s011 -
                    qperpw * qperpy * s021 - qperpx * qperpz * s021 -
                    qperpy * qperpy * s101 - qperpz * qperpz * s101 +
                    q0y * q0y * (-s001 + s101) + q0z * q0z * (-s001 + s101) +
                    qperpx * qperpy * s111 - qperpw * qperpz * s111 +
                    q0y * (q0x * (s011 - s111) + q0w * (s021 - s121)) +
                    qperpw * qperpy * s121 + qperpx * qperpz * s121 +
                    q0z * (-(q0w * s011) + q0x * s021 + q0w * s111 - q0x * s121)) *
                    theta,
                2.0 * (qperpy * qperpy * s002 + qperpz * qperpz * s002 -
                    qperpx * qperpy * s012 + qperpw * qperpz * s012 -
                    qperpw * qperpy * s022 - qperpx * qperpz * s022 -
                    qperpy * qperpy * s102 - qperpz * qperpz * s102 +
                    q0y * q0y * (-s002 + s102) + q0z * q0z * (-s002 + s102) +
                    qperpx * qperpy * s112 - qperpw * qperpz * s112 +
                    q0y * (q0x * (s012 - s112) + q0w * (s022 - s122)) +
                    qperpw * qperpy * s122 + qperpx * qperpz * s122 +
                    q0z * (-(q0w * s012) + q0x * s022 + q0w * s112 - q0x * s122)) *
                    theta);

            res.c1[1] = DerivativeTerm::new(
                -t0y + t1y,
                -(qperpx * qperpy * s000) - qperpw * qperpz * s000 - s010 +
                    q0z * q0z * s010 + qperpx * qperpx * s010 +
                    qperpz * qperpz * s010 - q0y * q0z * s020 +
                    qperpw * qperpx * s020 - qperpy * qperpz * s020 +
                    qperpx * qperpy * s100 + qperpw * qperpz * s100 +
                    q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) + s110 -
                    q0z * q0z * s110 - qperpx * qperpx * s110 -
                    qperpz * qperpz * s110 +
                    q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
                    q0y * q0z * s120 - qperpw * qperpx * s120 +
                    qperpy * qperpz * s120,
                -(qperpx * qperpy * s001) - qperpw * qperpz * s001 - s011 +
                    q0z * q0z * s011 + qperpx * qperpx * s011 +
                    qperpz * qperpz * s011 - q0y * q0z * s021 +
                    qperpw * qperpx * s021 - qperpy * qperpz * s021 +
                    qperpx * qperpy * s101 + qperpw * qperpz * s101 +
                    q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) + s111 -
                    q0z * q0z * s111 - qperpx * qperpx * s111 -
                    qperpz * qperpz * s111 +
                    q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
                    q0y * q0z * s121 - qperpw * qperpx * s121 +
                    qperpy * qperpz * s121,
                -(qperpx * qperpy * s002) - qperpw * qperpz * s002 - s012 +
                    q0z * q0z * s012 + qperpx * qperpx * s012 +
                    qperpz * qperpz * s012 - q0y * q0z * s022 +
                    qperpw * qperpx * s022 - qperpy * qperpz * s022 +
                    qperpx * qperpy * s102 + qperpw * qperpz * s102 +
                    q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) + s112 -
                    q0z * q0z * s112 - qperpx * qperpx * s112 -
                    qperpz * qperpz * s112 +
                    q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
                    q0y * q0z * s122 - qperpw * qperpx * s122 +
                    qperpy * qperpz * s122);

            res.c2[1] = DerivativeTerm::new(
                0.0,
                qperpx * qperpy * s000 + qperpw * qperpz * s000 + q0z * q0z * s010 -
                    qperpx * qperpx * s010 - qperpz * qperpz * s010 -
                    q0y * q0z * s020 - qperpw * qperpx * s020 +
                    qperpy * qperpz * s020 - qperpx * qperpy * s100 -
                    qperpw * qperpz * s100 + q0x * q0x * (s010 - s110) -
                    q0z * q0z * s110 + qperpx * qperpx * s110 +
                    qperpz * qperpz * s110 + q0y * q0z * s120 +
                    qperpw * qperpx * s120 - qperpy * qperpz * s120 +
                    2.0 * q0z * qperpw * s000 * theta +
                    2.0 * q0y * qperpx * s000 * theta -
                    4.0 * q0z * qperpz * s010 * theta +
                    2.0 * q0z * qperpy * s020 * theta +
                    2.0 * q0y * qperpz * s020 * theta +
                    q0x * (q0w * s020 + q0y * (-s000 + s100) - q0w * s120 +
                        2.0 * qperpy * s000 * theta - 4.0 * qperpx * s010 * theta -
                        2.0 * qperpw * s020 * theta) +
                    q0w * (-(q0z * s000) + q0z * s100 + 2.0 * qperpz * s000 * theta -
                        2.0 * qperpx * s020 * theta),
                qperpx * qperpy * s001 + qperpw * qperpz * s001 + q0z * q0z * s011 -
                    qperpx * qperpx * s011 - qperpz * qperpz * s011 -
                    q0y * q0z * s021 - qperpw * qperpx * s021 +
                    qperpy * qperpz * s021 - qperpx * qperpy * s101 -
                    qperpw * qperpz * s101 + q0x * q0x * (s011 - s111) -
                    q0z * q0z * s111 + qperpx * qperpx * s111 +
                    qperpz * qperpz * s111 + q0y * q0z * s121 +
                    qperpw * qperpx * s121 - qperpy * qperpz * s121 +
                    2.0 * q0z * qperpw * s001 * theta +
                    2.0 * q0y * qperpx * s001 * theta -
                    4.0 * q0z * qperpz * s011 * theta +
                    2.0 * q0z * qperpy * s021 * theta +
                    2.0 * q0y * qperpz * s021 * theta +
                    q0x * (q0w * s021 + q0y * (-s001 + s101) - q0w * s121 +
                        2.0 * qperpy * s001 * theta - 4.0 * qperpx * s011 * theta -
                        2.0 * qperpw * s021 * theta) +
                    q0w * (-(q0z * s001) + q0z * s101 + 2.0 * qperpz * s001 * theta -
                        2.0 * qperpx * s021 * theta),
                qperpx * qperpy * s002 + qperpw * qperpz * s002 + q0z * q0z * s012 -
                    qperpx * qperpx * s012 - qperpz * qperpz * s012 -
                    q0y * q0z * s022 - qperpw * qperpx * s022 +
                    qperpy * qperpz * s022 - qperpx * qperpy * s102 -
                    qperpw * qperpz * s102 + q0x * q0x * (s012 - s112) -
                    q0z * q0z * s112 + qperpx * qperpx * s112 +
                    qperpz * qperpz * s112 + q0y * q0z * s122 +
                    qperpw * qperpx * s122 - qperpy * qperpz * s122 +
                    2.0 * q0z * qperpw * s002 * theta +
                    2.0 * q0y * qperpx * s002 * theta -
                    4.0 * q0z * qperpz * s012 * theta +
                    2.0 * q0z * qperpy * s022 * theta +
                    2.0 * q0y * qperpz * s022 * theta +
                    q0x * (q0w * s022 + q0y * (-s002 + s102) - q0w * s122 +
                        2.0 * qperpy * s002 * theta - 4.0 * qperpx * s012 * theta -
                        2.0 * qperpw * s022 * theta) +
                    q0w * (-(q0z * s002) + q0z * s102 + 2.0 * qperpz * s002 * theta -
                        2.0 * qperpx * s022 * theta));

            res.c3[1] = DerivativeTerm::new(
                0., 2.0 * (-(q0x * qperpy * s000) - q0w * qperpz * s000 +
                    2.0 * q0x * qperpx * s010 + q0x * qperpw * s020 +
                    q0w * qperpx * s020 + q0x * qperpy * s100 +
                    q0w * qperpz * s100 - 2.0 * q0x * qperpx * s110 -
                    q0x * qperpw * s120 - q0w * qperpx * s120 +
                    q0z * (2.0 * qperpz * s010 - qperpy * s020 +
                        qperpw * (-s000 + s100) - 2.0 * qperpz * s110 +
                        qperpy * s120) +
                    q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
                        qperpz * s120)) *
                    theta,
                2.0 * (-(q0x * qperpy * s001) - q0w * qperpz * s001 +
                    2.0 * q0x * qperpx * s011 + q0x * qperpw * s021 +
                    q0w * qperpx * s021 + q0x * qperpy * s101 +
                    q0w * qperpz * s101 - 2.0 * q0x * qperpx * s111 -
                    q0x * qperpw * s121 - q0w * qperpx * s121 +
                    q0z * (2.0 * qperpz * s011 - qperpy * s021 +
                        qperpw * (-s001 + s101) - 2.0 * qperpz * s111 +
                        qperpy * s121) +
                    q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
                        qperpz * s121)) *
                    theta,
                2.0 * (-(q0x * qperpy * s002) - q0w * qperpz * s002 +
                    2.0 * q0x * qperpx * s012 + q0x * qperpw * s022 +
                    q0w * qperpx * s022 + q0x * qperpy * s102 +
                    q0w * qperpz * s102 - 2.0 * q0x * qperpx * s112 -
                    q0x * qperpw * s122 - q0w * qperpx * s122 +
                    q0z * (2.0 * qperpz * s012 - qperpy * s022 +
                        qperpw * (-s002 + s102) - 2.0 * qperpz * s112 +
                        qperpy * s122) +
                    q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
                        qperpz * s122)) *
                    theta);

            res.c4[1] = DerivativeTerm::new(
                0.0,
                -(q0x * qperpy * s000) - q0w * qperpz * s000 +
                    2.0 * q0x * qperpx * s010 + q0x * qperpw * s020 +
                    q0w * qperpx * s020 + q0x * qperpy * s100 +
                    q0w * qperpz * s100 - 2.0 * q0x * qperpx * s110 -
                    q0x * qperpw * s120 - q0w * qperpx * s120 +
                    2.0 * qperpx * qperpy * s000 * theta +
                    2.0 * qperpw * qperpz * s000 * theta +
                    2.0 * q0x * q0x * s010 * theta + 2.0 * q0z * q0z * s010 * theta -
                    2.0 * qperpx * qperpx * s010 * theta -
                    2.0 * qperpz * qperpz * s010 * theta +
                    2.0 * q0w * q0x * s020 * theta -
                    2.0 * qperpw * qperpx * s020 * theta +
                    2.0 * qperpy * qperpz * s020 * theta +
                    q0y * (-(qperpx * s000) - qperpz * s020 + qperpx * s100 +
                        qperpz * s120 - 2.0 * q0x * s000 * theta) +
                    q0z * (2.0 * qperpz * s010 - qperpy * s020 +
                        qperpw * (-s000 + s100) - 2.0 * qperpz * s110 +
                        qperpy * s120 - 2.0 * q0w * s000 * theta -
                        2.0 * q0y * s020 * theta),
                -(q0x * qperpy * s001) - q0w * qperpz * s001 +
                    2.0 * q0x * qperpx * s011 + q0x * qperpw * s021 +
                    q0w * qperpx * s021 + q0x * qperpy * s101 +
                    q0w * qperpz * s101 - 2.0 * q0x * qperpx * s111 -
                    q0x * qperpw * s121 - q0w * qperpx * s121 +
                    2.0 * qperpx * qperpy * s001 * theta +
                    2.0 * qperpw * qperpz * s001 * theta +
                    2.0 * q0x * q0x * s011 * theta + 2.0 * q0z * q0z * s011 * theta -
                    2.0 * qperpx * qperpx * s011 * theta -
                    2.0 * qperpz * qperpz * s011 * theta +
                    2.0 * q0w * q0x * s021 * theta -
                    2.0 * qperpw * qperpx * s021 * theta +
                    2.0 * qperpy * qperpz * s021 * theta +
                    q0y * (-(qperpx * s001) - qperpz * s021 + qperpx * s101 +
                        qperpz * s121 - 2.0 * q0x * s001 * theta) +
                    q0z * (2.0 * qperpz * s011 - qperpy * s021 +
                        qperpw * (-s001 + s101) - 2.0 * qperpz * s111 +
                        qperpy * s121 - 2.0 * q0w * s001 * theta -
                        2.0 * q0y * s021 * theta),
                -(q0x * qperpy * s002) - q0w * qperpz * s002 +
                    2.0 * q0x * qperpx * s012 + q0x * qperpw * s022 +
                    q0w * qperpx * s022 + q0x * qperpy * s102 +
                    q0w * qperpz * s102 - 2.0 * q0x * qperpx * s112 -
                    q0x * qperpw * s122 - q0w * qperpx * s122 +
                    2.0 * qperpx * qperpy * s002 * theta +
                    2.0 * qperpw * qperpz * s002 * theta +
                    2.0 * q0x * q0x * s012 * theta + 2.0 * q0z * q0z * s012 * theta -
                    2.0 * qperpx * qperpx * s012 * theta -
                    2.0 * qperpz * qperpz * s012 * theta +
                    2.0 * q0w * q0x * s022 * theta -
                    2.0 * qperpw * qperpx * s022 * theta +
                    2.0 * qperpy * qperpz * s022 * theta +
                    q0y * (-(qperpx * s002) - qperpz * s022 + qperpx * s102 +
                        qperpz * s122 - 2.0 * q0x * s002 * theta) +
                    q0z * (2.0 * qperpz * s012 - qperpy * s022 +
                        qperpw * (-s002 + s102) - 2.0 * qperpz * s112 +
                        qperpy * s122 - 2.0 * q0w * s002 * theta -
                        2.0 * q0y * s022 * theta));

            res.c5[1] = DerivativeTerm::new(
                0., -2.0 * (qperpx * qperpy * s000 + qperpw * qperpz * s000 +
                    q0z * q0z * s010 - qperpx * qperpx * s010 -
                    qperpz * qperpz * s010 - q0y * q0z * s020 -
                    qperpw * qperpx * s020 + qperpy * qperpz * s020 -
                    qperpx * qperpy * s100 - qperpw * qperpz * s100 +
                    q0w * q0z * (-s000 + s100) + q0x * q0x * (s010 - s110) -
                    q0z * q0z * s110 + qperpx * qperpx * s110 +
                    qperpz * qperpz * s110 +
                    q0x * (q0y * (-s000 + s100) + q0w * (s020 - s120)) +
                    q0y * q0z * s120 + qperpw * qperpx * s120 -
                    qperpy * qperpz * s120) *
                    theta,
                -2.0 * (qperpx * qperpy * s001 + qperpw * qperpz * s001 +
                    q0z * q0z * s011 - qperpx * qperpx * s011 -
                    qperpz * qperpz * s011 - q0y * q0z * s021 -
                    qperpw * qperpx * s021 + qperpy * qperpz * s021 -
                    qperpx * qperpy * s101 - qperpw * qperpz * s101 +
                    q0w * q0z * (-s001 + s101) + q0x * q0x * (s011 - s111) -
                    q0z * q0z * s111 + qperpx * qperpx * s111 +
                    qperpz * qperpz * s111 +
                    q0x * (q0y * (-s001 + s101) + q0w * (s021 - s121)) +
                    q0y * q0z * s121 + qperpw * qperpx * s121 -
                    qperpy * qperpz * s121) *
                    theta,
                -2.0 * (qperpx * qperpy * s002 + qperpw * qperpz * s002 +
                    q0z * q0z * s012 - qperpx * qperpx * s012 -
                    qperpz * qperpz * s012 - q0y * q0z * s022 -
                    qperpw * qperpx * s022 + qperpy * qperpz * s022 -
                    qperpx * qperpy * s102 - qperpw * qperpz * s102 +
                    q0w * q0z * (-s002 + s102) + q0x * q0x * (s012 - s112) -
                    q0z * q0z * s112 + qperpx * qperpx * s112 +
                    qperpz * qperpz * s112 +
                    q0x * (q0y * (-s002 + s102) + q0w * (s022 - s122)) +
                    q0y * q0z * s122 + qperpw * qperpx * s122 -
                    qperpy * qperpz * s122) *
                    theta);

            res.c1[2] = DerivativeTerm::new(
                -t0z + t1z, qperpw * qperpy * s000 - qperpx * qperpz * s000 -
                    q0y * q0z * s010 - qperpw * qperpx * s010 -
                    qperpy * qperpz * s010 - s020 + q0y * q0y * s020 +
                    qperpx * qperpx * s020 + qperpy * qperpy * s020 -
                    qperpw * qperpy * s100 + qperpx * qperpz * s100 +
                    q0x * q0z * (-s000 + s100) + q0y * q0z * s110 +
                    qperpw * qperpx * s110 + qperpy * qperpz * s110 +
                    q0w * (q0y * (s000 - s100) + q0x * (-s010 + s110)) +
                    q0x * q0x * (s020 - s120) + s120 - q0y * q0y * s120 -
                    qperpx * qperpx * s120 - qperpy * qperpy * s120,
                qperpw * qperpy * s001 - qperpx * qperpz * s001 -
                    q0y * q0z * s011 - qperpw * qperpx * s011 -
                    qperpy * qperpz * s011 - s021 + q0y * q0y * s021 +
                    qperpx * qperpx * s021 + qperpy * qperpy * s021 -
                    qperpw * qperpy * s101 + qperpx * qperpz * s101 +
                    q0x * q0z * (-s001 + s101) + q0y * q0z * s111 +
                    qperpw * qperpx * s111 + qperpy * qperpz * s111 +
                    q0w * (q0y * (s001 - s101) + q0x * (-s011 + s111)) +
                    q0x * q0x * (s021 - s121) + s121 - q0y * q0y * s121 -
                    qperpx * qperpx * s121 - qperpy * qperpy * s121,
                qperpw * qperpy * s002 - qperpx * qperpz * s002 -
                    q0y * q0z * s012 - qperpw * qperpx * s012 -
                    qperpy * qperpz * s012 - s022 + q0y * q0y * s022 +
                    qperpx * qperpx * s022 + qperpy * qperpy * s022 -
                    qperpw * qperpy * s102 + qperpx * qperpz * s102 +
                    q0x * q0z * (-s002 + s102) + q0y * q0z * s112 +
                    qperpw * qperpx * s112 + qperpy * qperpz * s112 +
                    q0w * (q0y * (s002 - s102) + q0x * (-s012 + s112)) +
                    q0x * q0x * (s022 - s122) + s122 - q0y * q0y * s122 -
                    qperpx * qperpx * s122 - qperpy * qperpy * s122);

            res.c2[2] = DerivativeTerm::new(
                0.,
                q0w * q0y * s000 - q0x * q0z * s000 - qperpw * qperpy * s000 +
                    qperpx * qperpz * s000 - q0w * q0x * s010 - q0y * q0z * s010 +
                    qperpw * qperpx * s010 + qperpy * qperpz * s010 +
                    q0x * q0x * s020 + q0y * q0y * s020 - qperpx * qperpx * s020 -
                    qperpy * qperpy * s020 - q0w * q0y * s100 + q0x * q0z * s100 +
                    qperpw * qperpy * s100 - qperpx * qperpz * s100 +
                    q0w * q0x * s110 + q0y * q0z * s110 - qperpw * qperpx * s110 -
                    qperpy * qperpz * s110 - q0x * q0x * s120 - q0y * q0y * s120 +
                    qperpx * qperpx * s120 + qperpy * qperpy * s120 -
                    2.0 * q0y * qperpw * s000 * theta + 2.0 * q0z * qperpx * s000 * theta -
                    2.0 * q0w * qperpy * s000 * theta + 2.0 * q0x * qperpz * s000 * theta +
                    2.0 * q0x * qperpw * s010 * theta + 2.0 * q0w * qperpx * s010 * theta +
                    2.0 * q0z * qperpy * s010 * theta + 2.0 * q0y * qperpz * s010 * theta -
                    4.0 * q0x * qperpx * s020 * theta - 4.0 * q0y * qperpy * s020 * theta,
                q0w * q0y * s001 - q0x * q0z * s001 - qperpw * qperpy * s001 +
                    qperpx * qperpz * s001 - q0w * q0x * s011 - q0y * q0z * s011 +
                    qperpw * qperpx * s011 + qperpy * qperpz * s011 +
                    q0x * q0x * s021 + q0y * q0y * s021 - qperpx * qperpx * s021 -
                    qperpy * qperpy * s021 - q0w * q0y * s101 + q0x * q0z * s101 +
                    qperpw * qperpy * s101 - qperpx * qperpz * s101 +
                    q0w * q0x * s111 + q0y * q0z * s111 - qperpw * qperpx * s111 -
                    qperpy * qperpz * s111 - q0x * q0x * s121 - q0y * q0y * s121 +
                    qperpx * qperpx * s121 + qperpy * qperpy * s121 -
                    2.0 * q0y * qperpw * s001 * theta + 2.0 * q0z * qperpx * s001 * theta -
                    2.0 * q0w * qperpy * s001 * theta + 2.0 * q0x * qperpz * s001 * theta +
                    2.0 * q0x * qperpw * s011 * theta + 2.0 * q0w * qperpx * s011 * theta +
                    2.0 * q0z * qperpy * s011 * theta + 2.0 * q0y * qperpz * s011 * theta -
                    4.0 * q0x * qperpx * s021 * theta - 4.0 * q0y * qperpy * s021 * theta,
                q0w * q0y * s002 - q0x * q0z * s002 - qperpw * qperpy * s002 +
                    qperpx * qperpz * s002 - q0w * q0x * s012 - q0y * q0z * s012 +
                    qperpw * qperpx * s012 + qperpy * qperpz * s012 +
                    q0x * q0x * s022 + q0y * q0y * s022 - qperpx * qperpx * s022 -
                    qperpy * qperpy * s022 - q0w * q0y * s102 + q0x * q0z * s102 +
                    qperpw * qperpy * s102 - qperpx * qperpz * s102 +
                    q0w * q0x * s112 + q0y * q0z * s112 - qperpw * qperpx * s112 -
                    qperpy * qperpz * s112 - q0x * q0x * s122 - q0y * q0y * s122 +
                    qperpx * qperpx * s122 + qperpy * qperpy * s122 -
                    2.0 * q0y * qperpw * s002 * theta + 2.0 * q0z * qperpx * s002 * theta -
                    2.0 * q0w * qperpy * s002 * theta + 2.0 * q0x * qperpz * s002 * theta +
                    2.0 * q0x * qperpw * s012 * theta + 2.0 * q0w * qperpx * s012 * theta +
                    2.0 * q0z * qperpy * s012 * theta + 2.0 * q0y * qperpz * s012 * theta -
                    4.0 * q0x * qperpx * s022 * theta -
                    4.0 * q0y * qperpy * s022 * theta);

            res.c3[2] = DerivativeTerm::new(
                0., -2.0 * (-(q0w * qperpy * s000) + q0x * qperpz * s000 +
                    q0x * qperpw * s010 + q0w * qperpx * s010 -
                    2.0 * q0x * qperpx * s020 + q0w * qperpy * s100 -
                    q0x * qperpz * s100 - q0x * qperpw * s110 -
                    q0w * qperpx * s110 +
                    q0z * (qperpx * s000 + qperpy * s010 - qperpx * s100 -
                        qperpy * s110) +
                    2.0 * q0x * qperpx * s120 +
                    q0y * (qperpz * s010 - 2.0 * qperpy * s020 +
                        qperpw * (-s000 + s100) - qperpz * s110 +
                        2.0 * qperpy * s120)) *
                    theta,
                -2.0 * (-(q0w * qperpy * s001) + q0x * qperpz * s001 +
                    q0x * qperpw * s011 + q0w * qperpx * s011 -
                    2.0 * q0x * qperpx * s021 + q0w * qperpy * s101 -
                    q0x * qperpz * s101 - q0x * qperpw * s111 -
                    q0w * qperpx * s111 +
                    q0z * (qperpx * s001 + qperpy * s011 - qperpx * s101 -
                        qperpy * s111) +
                    2.0 * q0x * qperpx * s121 +
                    q0y * (qperpz * s011 - 2.0 * qperpy * s021 +
                        qperpw * (-s001 + s101) - qperpz * s111 +
                        2.0 * qperpy * s121)) *
                    theta,
                -2.0 * (-(q0w * qperpy * s002) + q0x * qperpz * s002 +
                    q0x * qperpw * s012 + q0w * qperpx * s012 -
                    2.0 * q0x * qperpx * s022 + q0w * qperpy * s102 -
                    q0x * qperpz * s102 - q0x * qperpw * s112 -
                    q0w * qperpx * s112 +
                    q0z * (qperpx * s002 + qperpy * s012 - qperpx * s102 -
                        qperpy * s112) +
                    2.0 * q0x * qperpx * s122 +
                    q0y * (qperpz * s012 - 2.0 * qperpy * s022 +
                        qperpw * (-s002 + s102) - qperpz * s112 +
                        2.0 * qperpy * s122)) *
                    theta);

            res.c4[2] = DerivativeTerm::new(
                0.,
                q0w * qperpy * s000 - q0x * qperpz * s000 - q0x * qperpw * s010 -
                    q0w * qperpx * s010 + 2.0 * q0x * qperpx * s020 -
                    q0w * qperpy * s100 + q0x * qperpz * s100 +
                    q0x * qperpw * s110 + q0w * qperpx * s110 -
                    2.0 * q0x * qperpx * s120 - 2.0 * qperpw * qperpy * s000 * theta +
                    2.0 * qperpx * qperpz * s000 * theta -
                    2.0 * q0w * q0x * s010 * theta +
                    2.0 * qperpw * qperpx * s010 * theta +
                    2.0 * qperpy * qperpz * s010 * theta +
                    2.0 * q0x * q0x * s020 * theta + 2.0 * q0y * q0y * s020 * theta -
                    2.0 * qperpx * qperpx * s020 * theta -
                    2.0 * qperpy * qperpy * s020 * theta +
                    q0z * (-(qperpx * s000) - qperpy * s010 + qperpx * s100 +
                        qperpy * s110 - 2.0 * q0x * s000 * theta) +
                    q0y * (-(qperpz * s010) + 2.0 * qperpy * s020 +
                        qperpw * (s000 - s100) + qperpz * s110 -
                        2.0 * qperpy * s120 + 2.0 * q0w * s000 * theta -
                        2.0 * q0z * s010 * theta),
                q0w * qperpy * s001 - q0x * qperpz * s001 - q0x * qperpw * s011 -
                    q0w * qperpx * s011 + 2.0 * q0x * qperpx * s021 -
                    q0w * qperpy * s101 + q0x * qperpz * s101 +
                    q0x * qperpw * s111 + q0w * qperpx * s111 -
                    2.0 * q0x * qperpx * s121 - 2.0 * qperpw * qperpy * s001 * theta +
                    2.0 * qperpx * qperpz * s001 * theta -
                    2.0 * q0w * q0x * s011 * theta +
                    2.0 * qperpw * qperpx * s011 * theta +
                    2.0 * qperpy * qperpz * s011 * theta +
                    2.0 * q0x * q0x * s021 * theta + 2.0 * q0y * q0y * s021 * theta -
                    2.0 * qperpx * qperpx * s021 * theta -
                    2.0 * qperpy * qperpy * s021 * theta +
                    q0z * (-(qperpx * s001) - qperpy * s011 + qperpx * s101 +
                        qperpy * s111 - 2.0 * q0x * s001 * theta) +
                    q0y * (-(qperpz * s011) + 2.0 * qperpy * s021 +
                        qperpw * (s001 - s101) + qperpz * s111 -
                        2.0 * qperpy * s121 + 2.0 * q0w * s001 * theta -
                        2.0 * q0z * s011 * theta),
                q0w * qperpy * s002 - q0x * qperpz * s002 - q0x * qperpw * s012 -
                    q0w * qperpx * s012 + 2.0 * q0x * qperpx * s022 -
                    q0w * qperpy * s102 + q0x * qperpz * s102 +
                    q0x * qperpw * s112 + q0w * qperpx * s112 -
                    2.0 * q0x * qperpx * s122 - 2.0 * qperpw * qperpy * s002 * theta +
                    2.0 * qperpx * qperpz * s002 * theta -
                    2.0 * q0w * q0x * s012 * theta +
                    2.0 * qperpw * qperpx * s012 * theta +
                    2.0 * qperpy * qperpz * s012 * theta +
                    2.0 * q0x * q0x * s022 * theta + 2.0 * q0y * q0y * s022 * theta -
                    2.0 * qperpx * qperpx * s022 * theta -
                    2.0 * qperpy * qperpy * s022 * theta +
                    q0z * (-(qperpx * s002) - qperpy * s012 + qperpx * s102 +
                        qperpy * s112 - 2.0 * q0x * s002 * theta) +
                    q0y * (-(qperpz * s012) + 2.0 * qperpy * s022 +
                        qperpw * (s002 - s102) + qperpz * s112 -
                        2.0 * qperpy * s122 + 2.0 * q0w * s002 * theta -
                        2.0 * q0z * s012 * theta));

            res.c5[2] = DerivativeTerm::new(
                0., 2.0 * (qperpw * qperpy * s000 - qperpx * qperpz * s000 +
                    q0y * q0z * s010 - qperpw * qperpx * s010 -
                    qperpy * qperpz * s010 - q0y * q0y * s020 +
                    qperpx * qperpx * s020 + qperpy * qperpy * s020 +
                    q0x * q0z * (s000 - s100) - qperpw * qperpy * s100 +
                    qperpx * qperpz * s100 +
                    q0w * (q0y * (-s000 + s100) + q0x * (s010 - s110)) -
                    q0y * q0z * s110 + qperpw * qperpx * s110 +
                    qperpy * qperpz * s110 + q0y * q0y * s120 -
                    qperpx * qperpx * s120 - qperpy * qperpy * s120 +
                    q0x * q0x * (-s020 + s120)) *
                    theta,
                2.0 * (qperpw * qperpy * s001 - qperpx * qperpz * s001 +
                    q0y * q0z * s011 - qperpw * qperpx * s011 -
                    qperpy * qperpz * s011 - q0y * q0y * s021 +
                    qperpx * qperpx * s021 + qperpy * qperpy * s021 +
                    q0x * q0z * (s001 - s101) - qperpw * qperpy * s101 +
                    qperpx * qperpz * s101 +
                    q0w * (q0y * (-s001 + s101) + q0x * (s011 - s111)) -
                    q0y * q0z * s111 + qperpw * qperpx * s111 +
                    qperpy * qperpz * s111 + q0y * q0y * s121 -
                    qperpx * qperpx * s121 - qperpy * qperpy * s121 +
                    q0x * q0x * (-s021 + s121)) *
                    theta,
                2.0 * (qperpw * qperpy * s002 - qperpx * qperpz * s002 +
                    q0y * q0z * s012 - qperpw * qperpx * s012 -
                    qperpy * qperpz * s012 - q0y * q0y * s022 +
                    qperpx * qperpx * s022 + qperpy * qperpy * s022 +
                    q0x * q0z * (s002 - s102) - qperpw * qperpy * s102 +
                    qperpx * qperpz * s102 +
                    q0w * (q0y * (-s002 + s102) + q0x * (s012 - s112)) -
                    q0y * q0z * s112 + qperpw * qperpx * s112 +
                    qperpy * qperpz * s112 + q0y * q0y * s122 -
                    qperpx * qperpx * s122 - qperpy * qperpy * s122 +
                    q0x * q0x * (-s022 + s122)) *
                    theta);
        }

        res
    }

    pub fn decompose(m: &Matrix4<Float>, t: &mut Vector3f, rquat: &mut Quaternion, s: &mut Matrix4<Float>) {
        // Extract translation
        t.x = m[(0, 3)];
        t.y = m[(1, 3)];
        t.z = m[(2, 3)];
        // Compute new transformation matrix without translation
        let mut new_m = *m;

        for i in 0..3 {
            new_m[(i, 3)] = 0.0;
            new_m[(3, i)] = 0.0;
        }

        new_m[(3, 3)] = 1.0;

        // Extract rotation R from transformation matrix
        let mut norm: Float;
        let mut counter = 0;
        let mut r = new_m;

        loop {
            let mut rnext = Matrix4::<Float>::identity();
            let r_it = r.transpose().try_inverse().unwrap();

            for i in 0..4 {
                for j in 0..4 {
                    rnext[(i, j)] = 0.5 * (r[(i, j)] + r_it[(i, j)]);
                }
            }

            norm = 0.0;
            for i in 0..3 {
                let n = (r[(i, 0)] - rnext[(i, 0)]).abs() +
                        (r[(i, 1)] - rnext[(i, 1)]).abs() +
                        (r[(i, 2)] - rnext[(i, 2)]).abs();
                norm = norm.max(n);
            }

            r = rnext;
            counter += 1;

            if counter >= 100 || norm <= 0.0001 {
                break;
            }
        }

        *rquat = Transform::from_matrix(&r).into();
        *s = r.try_inverse().unwrap() * m;
    }

    pub fn interpolate(&self, time: Float, t: &mut Transform) {
        if !self.actually_animated || time <= self.start_time {
            *t = *self.start_transform;
            return;
        }

        if time >= self.end_time {
            *t = *self.end_transform;
            return;
        }

        let dt = (time - self.start_time) / (self.end_time - self.start_time);

        // Interpolate translation at dt
        //let trans = lerp(dt, self.t[0], self.t[1]);
        let trans = self.t[0] * (1.0 - dt) + self.t[1] * dt;

        // Interpolate rotation at dt
        let rotate = slerp(&self.r[0], &self.r[1], dt);

        // Interpolate scale at dt
        let mut scale = Matrix4::<Float>::identity();

        for i in 0..3 {
            for j in 0..3 {
                scale[(i, j)] = lerp(dt, self.s[0][(i, j)], self.s[1][(i, j)]);
            }
        }

        // Compute interpolated matrix as product of interpolated components
        *t = Transform::translate(&trans) * rotate.into() * Transform::from_matrix(&scale);
    }

    pub fn transform_point(&self, time: Float, p: &Point3f) -> Point3f {
        if !self.actually_animated || time <= self.start_time {
            return self.start_transform.transform_point(p);
        } else if time >= self.end_time {
            return self.end_transform.transform_point(p);
        }

        let mut t = Transform::default();
        self.interpolate(time, &mut t);

        t.transform_point(p)
    }

    pub fn transform_vector(&self, time: Float, v: &Vector3f) -> Vector3f {
        if !self.actually_animated || time <= self.start_time {
            return self.start_transform.transform_vector(v);
        } else if time >= self.end_time {
            return self.end_transform.transform_vector(v);
        }

        let mut t = Transform::default();
        self.interpolate(time, &mut t);

        t.transform_vector(v)
    }

    pub fn transform_ray(&self, r: &Ray) -> Ray {
        if !self.actually_animated || r.time <= self.start_time {
            return self.start_transform.transform_ray(r);
        } else if r.time >= self.end_time {
            return self.end_transform.transform_ray(r);
        }

        let mut t = Transform::default();
        self.interpolate(r.time, &mut t);

        t.transform_ray(r)
    }

    pub fn motion_bounds(&self, b: &Bounds3f) -> Bounds3f {
        if !self.actually_animated {
            return self.start_transform.transform_bounds(b);
        }

        if !self.has_rotation {
            return self.start_transform.transform_bounds(b).union_bounds(&self.end_transform.transform_bounds(b));
        }

        let mut bounds = Bounds3f::default();
        for i in 0..8 {
            bounds = bounds.union_bounds(&self.bound_point_motion(&b.corner(i)))
        }

        bounds
    }

    pub fn bound_point_motion(&self, p: &Point3f) -> Bounds3f {
        if !self.actually_animated {
            return Bounds3f::from_point(&self.start_transform.transform_point(p));
        }

        let mut bounds = Bounds3f::from_points(self.start_transform.transform_point(p), self.end_transform.transform_point(p));
        let cos_theta = self.r[0].dot(&self.r[1]);
        let theta = clamp(cos_theta, -1.0, 1.0).acos();

        for c in 0..3 {
            let mut zeros = [0.0; 8];
            let mut nzero = 0;

            interval_find_zeros(self.c1[c].eval(p), self.c2[c].eval(p), self.c3[c].eval(p),
                                self.c4[c].eval(p), self.c5[c].eval(p), theta,
                                Interval::new(0.0, 1.0), &mut zeros, &mut nzero, 8);

            println!("{}", nzero);

            for i in 0..nzero {
                let pz = self.transform_point(lerp(zeros[i], self.start_time, self.end_time), p);
                bounds = bounds.union_point(&pz);
            }
        }

        bounds
    }
}

#[derive(Debug, Copy, Clone, Default)]
struct DerivativeTerm {
    kc: Float,
    kx: Float,
    ky: Float,
    kz: Float
}

impl DerivativeTerm {
    fn new(kc: Float, kx: Float, ky: Float, kz: Float) -> Self {
        Self { kc, kx, ky, kz }
    }

    fn eval(&self, p: &Point3f) -> Float {
        self.kc + self.kx * p.x + self.ky * p.y + self.kz * p.z
    }
}

#[derive(Debug, Copy, Clone)]
struct Interval {
    low: Float,
    high: Float
}

impl Interval {
    fn new(v0: Float, v1: Float) -> Self {
        Interval { low: v0.min(v1), high: v0.max(v1)}
    }

    #[inline(always)]
    fn sin(&self) -> Self {
        assert!(self.low >= 0.0);
        assert!(self.high <= 2.0001 * PI);

        let mut sin_low = self.low.sin();
        let mut sin_high = self.high.sin();

        if sin_low > sin_high { std::mem::swap(&mut sin_low, &mut sin_high); }

        let pi = std::f32::consts::PI as Float;

        if self.low < pi / 2.0 && self.high > pi / 2.0 { sin_high = 1.0; }

        if self.low < (3.0 / 2.0) * pi && self.high > (3.0 / 2.0) * pi { sin_low = -1.0; }

        Self::new(sin_low, sin_high)
    }

    #[inline(always)]
    fn cos(&self) -> Self {
        assert!(self.low >= 0.0);
        assert!(self.high <= 2.0001 * PI);

        let mut cos_low = self.low.cos();
        let mut cos_high = self.high.cos();

        if cos_low > cos_high { std::mem::swap(&mut cos_low, &mut cos_high); }

        let pi = std::f32::consts::PI as Float;
        if self.low < pi && self.high > pi { cos_low = -1.0; }

        Self::new(cos_low, cos_high)
    }
}

impl From<Float> for Interval {
    fn from(f: Float) -> Self {
        Interval { low: f, high: f}
    }
}

impl Add for Interval {
    type Output = Self;

    fn add(self, i: Self) -> Self::Output {
        Self::new(self.low + i.low, self.high + i.high)
    }
}

impl Sub for Interval {
    type Output = Self;

    fn sub(self, i: Self) -> Self::Output {
        Self::new(self.low - i.high, self.high - i.low)
    }
}

impl Mul for Interval {
    type Output = Self;

    fn mul(self, i: Self) -> Self::Output {
        let low = ((self.low * i.low).min(self.high * i.low)).min((self.low * i.high).min(self.high * i.high));
        let high = ((self.low * i.low).max(self.high * i.low)).max((self.low * i.high).max(self.high * i.high));

        Self::new(low, high)
    }
}

fn interval_find_zeros(c1: Float, c2: Float, c3: Float, c4: Float, c5: Float, theta: Float, i: Interval, zeros: &mut [Float], zero_count: &mut usize, depth: isize) {
    type I = Interval;

    //println!("{}", c1);

    let c = I::from(2.0 * theta) * i;
    let range = I::from(c1) + (I::from(c2) + I::from(c3) * i) * c.cos() + (I::from(c4) + I::from(c5) * i) * c.sin();

    if range.low > 0.0 || range.high < 0.0 || range.low == range.high {
        return
    }

    if depth > 0 {
        let mid = (i.low + i.high) * 0.5;
        interval_find_zeros(c1, c2, c3, c4, c5, theta, Interval::new(i.low, mid), zeros, zero_count, depth - 1);
        interval_find_zeros(c1, c2, c3, c4, c5, theta, Interval::new(mid, i.high), zeros, zero_count, depth - 1);
    } else {
        let mut t_newton = (i.low + i.high) * 0.5;

        for _ in 0..4 {
            let f_newton = c1 + (c2 + c3 * t_newton) * (2.0 * theta * t_newton).cos() + (c4 + c5 * t_newton) * (2.0 * theta * t_newton).sin();
            let fprime_newton = (c3 + 2.0 * (c4 + c5 * t_newton) * theta) * (2.0 * t_newton * theta).cos() +
                                     (c5 - 2.0 * (c2 + c3 * t_newton) * theta) * (2.0 * t_newton * theta).sin();

            if f_newton == 0.0 || fprime_newton == 0.0 {
                break;
            }

            t_newton -= f_newton / fprime_newton;
        }

        if t_newton >= i.low - 1.0e-3 && t_newton < i.high + 1.0e-3 {
            zeros[*zero_count] = t_newton;
            *zero_count += 1;
        }
    }
}
