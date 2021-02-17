
#[cfg(test)]
mod animated_transform {
    use pbrt_rust::core::rng::RNG;
    use pbrt_rust::core::transform::{Transform, AnimatedTransform};
    use pbrt_rust::core::pbrt::Float;
    use pbrt_rust::core::geometry::vector::Vector3f;
    use pbrt_rust::core::sampling::uniform_sample_sphere;
    use pbrt_rust::core::geometry::point::{Point2f, Point3f};
    use std::sync::Arc;
    use pbrt_rust::core::geometry::bounds::Bounds3f;
    use nalgebra::base::storage::Storage;

    fn random_transform(rng: &mut RNG) -> Transform {
        let mut t = Transform::default();

        macro_rules! r {
            () => {{
                -10.0 + 20.0 * rng.uniform_float()
            }}
        }

        for i in 0..10 {
            t = match rng.uniform_int32_2(3) {
                0 => t * Transform::scale(r!().abs(), r!().abs(), r!().abs()),
                1 => t * Transform::translate(&Vector3f::new(r!(), r!(), r!())),
                2 => t * Transform::rotate(
                    r!() * 20.0,
                    &uniform_sample_sphere(&Point2f::new(rng.uniform_float(), rng.uniform_float()))),
                _ => t
            }
        }

        t
    }

    #[test]
    fn randoms() {
        let mut rng = RNG::default();

        macro_rules! r {
            () => {{
                -10.0 + 20.0 * rng.uniform_float()
            }}
        }

        for i in 0..200 {
            // Generate a pair of random transformation matrices
            let t0 = Arc::new(random_transform(&mut rng));
            let t1 = Arc::new(random_transform(&mut rng));


            let at = AnimatedTransform::new(t0, t1, 0.0,  1.0);

            for j in 0..5 {
                // Generate a random bounding box and find the bounds of its motion.
                let bounds = Bounds3f::from_points(Point3f::new(r!(), r!(), r!()), Point3f::new(r!(), r!(), r!()));
                let motion_bounds = at.motion_bounds(&bounds);
                let mut t = 0.0;

                while t <= 1.0 {
                    // Now, interpolate the transformations at a bunch of times
                    // along the time range and then transform the bounding box
                    // with the result.
                    let mut tr = Transform::default();
                    at.interpolate(t, &mut tr);
                    let mut tb = tr.transform_bounds(&bounds);

                    // Add a little slop to allow for floating-point round-off
                    // error in computing the motion extrema times.
                    tb.p_min += tb.diagonal() * 1.0e-4;
                    tb.p_max -= tb.diagonal() * 1.0e-4;

                    // Now, the transformed bounds should be inside the motion
                    // bounds.
                    assert!(tb.p_min.x >= motion_bounds.p_min.x);
                    assert!(tb.p_max.x <= motion_bounds.p_max.x);
                    assert!(tb.p_min.y >= motion_bounds.p_min.y);
                    assert!(tb.p_max.y <= motion_bounds.p_max.y);
                    assert!(tb.p_min.z >= motion_bounds.p_min.z);
                    assert!(tb.p_max.z <= motion_bounds.p_max.z);

                    t += 1.0e-3 * rng.uniform_float();
                }
            }
        }
    }
}