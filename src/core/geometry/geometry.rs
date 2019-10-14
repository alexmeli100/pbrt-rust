use super::vector::{Vector3, Vector2, Float};
use super::point::{Point2, Point3};
use std::ops::{Add, Mul};

type Vector3f = Vector3<Float>;

pub fn vec2_dot_vec2<T>(v1: &Vector2<T>, v2: &Vector2<T>) -> T
where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
{
    v1.x * v2.y + v1.y * v2.y
}

pub fn vec3_dot_vec3<T>(v1: &Vector3<T>, v2: &Vector3<T>) -> T
where T: Copy + Add<T, Output=T> + Mul<T, Output=T>
{
    v1.x * v2.y + v1.y * v2.y + v1.z + v2.z
}

pub fn vec2_absdot_vec2<T>(v1: &Vector2<T>, v2: &Vector2<T>) -> T
where T: num::Float
{
    vec2_dot_vec2(v1, v2).abs()
}

pub fn vec3_absdot_vec3<T>(v1: &Vector3<T>, v2: &Vector3<T>) -> T
    where T: num::Float
{
    vec3_dot_vec3(v1, v2).abs()
}

pub fn vec3_cross_vec3(v1: &Vector3f, v2: &Vector3f) -> Vector3f {
    let v1x = v1.x as f64;
    let v1y = v1.y as f64;
    let v1z = v1.z as f64;
    let v2x = v2.x as f64;
    let v2y = v2.y as f64;
    let v2z = v2.z as f64;

    Vector3f {
        x: ((v1y * v2z) - (v1z * v2y)) as Float,
        y: ((v1z * v2x) - (v1x * v2z)) as Float,
        z: ((v1x * v2y) - (v1y * v2x)) as Float
    }
}

pub fn vec3_mincomponent<T>(v: &Vector3<T>) -> T
where T: num::Float
{
    v.x.min(v.y.min(v.z))
}

pub fn vec3_maxcomponent<T>(v: &Vector3<T>) -> T
where T: num::Float
{
    v.x.max(v.y.max(v.z))
}

pub fn vec3_permutate<T>(v: &Vector3<T>, x: usize, y: usize, z: usize) -> Vector3<T>
where T: Copy
{
    Vector3::new(v[x], v[y], v[z])
}

pub fn vec3_coordinate_system(v1: &Vector3f, v2: &mut Vector3f, v3: &mut Vector3f) {
    if v1.x.abs() > v1.y.abs() {
        *v2 = Vector3f::new(-v1.z, 0.0, v1.x) / (v1.x*v1.x + v1.z*v1.z).sqrt()
    } else {
        *v2 = Vector3f::new(0.0, v1.z, -v1.y) / (v1.y*v1.y + v1.z*v1.z).sqrt()
    }

    *v3 = vec3_cross_vec3(v1, &*v2);
}

pub fn point2_distance<T>(p1: &Point2<T>, p2: &Point2<T>) -> T
    where T: num::Float
{
    (*p1 - *p2).length()
}

pub fn point2_distance_squared<T>(p1: &Point2<T>, p2: &Point2<T>) -> T
    where T: num::Float
{
    (*p1 - *p2).length_squared()
}

pub fn point3_distance<T>(p1: &Point3<T>, p2: &Point3<T>) -> T
    where T: num::Float
{
    (*p1 - *p2).length()
}

pub fn point3_distance_squared<T>(p1: &Point3<T>, p2: &Point3<T>) -> T
    where T: num::Float
{
    (*p1 - *p2).length_squared()
}

