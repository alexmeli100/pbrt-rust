
#[cfg(test)]
mod bounds {
    use pbrt_rust::core::geometry::bounds::{Bounds3f, Bounds2f};
    use pbrt_rust::core::geometry::point::{Point3f, Point2f};

    #[test]
    fn bounds2_union() {
        let a = Bounds2f::new(
            &Point2f::new(-10.0, -10.0),
            &Point2f::new(0.0, 20.0));
        let b = Bounds2f::default();
        let c = a.union_boundsf(&b);
        assert_eq!(a, c);
        assert_eq!(b, b.union_boundsf(&b));

        let d = Bounds2f::from_point(&Point2f::new(-15.0, 10.0));
        let e = a.union_boundsf(&d);
        assert_eq!(Bounds2f::new(&Point2f::new(-15.0, -10.0), &Point2f::new(0.0, 20.0)), e);
    }

    #[test]
    fn bounds3_union() {
        let a = Bounds3f::from_points(
            Point3f::new(-10.0, -10.0, 5.0),
            Point3f::new(0.0, 20.0, 10.0));
        let b = Bounds3f::default();
        let c = a.union_bounds(&b);
        assert_eq!(a, c);
        assert_eq!(b, b.union_bounds(&b));

        let d = Bounds3f::from_point(&Point3f::new(-15.0, 10.0, 30.0));
        let e = a.union_bounds(&d);
        assert_eq!(Bounds3f::from_points(Point3f::new(-15.0, -10.0, 5.0), Point3f::new(0.0, 20.0, 30.0)), e);
    }
}