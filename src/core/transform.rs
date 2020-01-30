use nalgebra::{Matrix4};
use crate::core::pbrt::*;
use crate::core::geometry::vector::*;
use crate::core::geometry::point::*;
use crate::core::geometry::normal::Normal3;
use crate::core::geometry::ray::*;
use crate::core::geometry::bounds::Bounds3f;
use num::{Zero, One};
use std::ops::{Mul, Add, Div};

#[derive(Debug, PartialEq, PartialOrd, Clone, Copy)]
pub struct Transform {
    m: Matrix4<Float>,
    m_inv: Matrix4<Float>
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
            x, 0.0, 0.0, 0.0,
            0.0, y, 0.0, 0.0,
            0.0, 0.0, z, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]);

        let m_inv = Matrix4::from_row_slice(&[
            1.0/x, 0.0, 0.0, 0.0,
            0.0, 1.0/y, 0.0, 0.0,
            0.0, 0.0, 1.0/z, 0.0,
            0.0, 0.0, 0.0, 1.0
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
        let mut m: Matrix4<Float> = Matrix4::zero();

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
        let mut camera_to_world: Matrix4<Float> = Matrix4::zero();

        camera_to_world[(0, 3)] = pos.x;
        camera_to_world[(1, 3)] = pos.y;
        camera_to_world[(2, 3)] = pos.z;
        camera_to_world[(3, 3)] = 1.0;

        let dir = (*look - *pos).normalize();
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

    pub fn transform_point<T>(&self, p: &Point3<T>) -> Point3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T> + Add<Float, Output=T> + Div<T, Output=T> + One + PartialEq + Copy + Zero
    {
        let x = p.x;
        let y = p.y;
        let z = p.z;

        let xp = x*self.m[(0, 0)] + y*self.m[(0, 1)] + z*self.m[(0, 2)] + self.m[(0, 3)];
        let yp = x*self.m[(1, 0)] + y*self.m[(1, 1)] + z*self.m[(1, 2)] + self.m[(1, 3)];
        let zp = x*self.m[(2, 0)] + y*self.m[(2, 1)] + z*self.m[(2, 2)] + self.m[(2, 3)];
        let wp = x*self.m[(3, 0)] + y*self.m[(3, 1)] + z*self.m[(3, 2)] + self.m[(3, 3)];

        if wp == T::one() {
            Point3::new(xp, yp, zp)
        } else {
            Point3::new(xp, yp, zp) / wp
        }
    }

    pub fn transform_point_error<T>(&self, p: &Point3<T>, o_error: &mut Vector3<T>) -> Point3<T> {
        unimplemented!()
    }

    pub fn transform_vector<T>(&self, v: &Vector3<T>) -> Vector3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T>
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

    pub fn transform_normal<T>(&self, n: &Normal3<T>) -> Normal3<T>
        where T: Mul<Float, Output=T> + Add<T, Output=T>
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

    pub fn transform_ray<'a>(&self, r: &'a Ray<'a>) -> Ray<'a> {
        let mut o_error = Vector3f::default();
        let mut o = self.transform_point_error(&r.o(), &mut o_error);
        let d = self.transform_vector(&r.d());

        let l_squared = d.length_squared();
        let t_max = r.t_max();

        if l_squared > 0.0 {
            let dt = d.abs().dot(&o_error) / l_squared;
            o += d * dt;
            t_max -= dt;
        }

        Ray::new(&o, &d, t_max, r.time(), r.medium())
    }

    pub fn transform_bounds(&self, b: Bounds3f) -> Bounds3f {
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

    fn swaps_handedness(&self) -> bool {
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




