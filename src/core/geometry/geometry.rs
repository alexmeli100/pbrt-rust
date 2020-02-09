use crate::core::geometry::point::Point3f;
use crate::core::geometry::vector::Vector3f;
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