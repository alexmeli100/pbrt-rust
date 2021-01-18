use crate::core::geometry::point::Point3f;
use crate::core::geometry::vector::{Vector3f, Vector3};
use crate::core::geometry::normal::Normal3f;
use crate::core::pbrt::*;

pub fn offset_ray_origin(p: &Point3f, p_error: &Vector3f, n: &Normal3f, w: &Vector3f) -> Point3f {
    let d = n.abs().dot_vec(p_error);
    let mut offset: Vector3f = Vector3f::from(*n) * d;

    if w.dot_norm(n) < 0.0 {
        offset = -offset;
    }

    let mut po = *p + offset;
    for i in 0..3 {
        if offset[i] > 0.0 {
            po[i] = next_float_up(po[i]);
        } else if offset[i] < 0.0 {
            po[i] = next_float_down(po[i]);
        }
    }

    po
}

#[inline]
pub fn spherical_direction(sin_theta: Float, cos_theta: Float, phi: Float) -> Vector3f {
    Vector3f::new(
        sin_theta * phi.cos(),
        sin_theta * phi.sin(),
        cos_theta
    )
}

#[inline]
pub fn spherical_direction_basis(sin_theta: Float, cos_theta: Float, phi: Float, x: &Vector3f, y: &Vector3f, z: &Vector3f) -> Vector3f {
    *x * sin_theta * phi.cos() + *y * sin_theta * phi.sin() + *z * cos_theta
}
#[inline]
pub fn spherical_theta(v: &Vector3f) -> Float {
    clamp(v.z, -1.0, -1.0).acos()
}

#[inline]
pub fn spherical_phi(v: &Vector3f) -> Float {
    let p = v.y.atan2(v.x);

    if p < 0.0 {
        p + 2.0 * PI
    } else {
        p
    }
}