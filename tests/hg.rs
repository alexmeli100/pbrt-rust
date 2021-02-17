
#[cfg(test)]
mod henyey_greenstein {
    use pbrt_rust::core::rng::RNG;
    use pbrt_rust::core::medium::{HenyeyGreenstein, PhaseFunction};
    use pbrt_rust::core::geometry::vector::Vector3f;
    use pbrt_rust::core::sampling::uniform_sample_sphere;
    use pbrt_rust::core::geometry::point::Point2f;
    use approx::relative_eq;
    use pbrt_rust::core::pbrt::{PI, Float};


    #[test]
    fn sampling_match() {
        let mut rng = RNG::default();
        let mut g = -0.75;

        while g <= 0.75 {
            let hg = HenyeyGreenstein::new(g);

            for i in 0..100 {
                let wo = uniform_sample_sphere(
                    &Point2f::new(rng.uniform_float(), rng.uniform_float()));
                let mut wi = Vector3f::default();
                let u = Point2f::new(rng.uniform_float(), rng.uniform_float());
                let p0 = hg.sample_p(&wo, &mut wi, &u);
                // Phase function is normalized and the sampling method should be exact
                relative_eq!(p0, hg.p(&wo, &wi), epsilon = 1.0e-4);
            }

            g += 0.25;
        }
    }

    #[test]
    fn sampling_orientation_forward() {
        let mut rng = RNG::default();
        let hg = HenyeyGreenstein::new(0.95);
        let wo = Vector3f::new(-1.0, 0.0, 0.0);
        let mut nforward = 0;
        let mut nbackward = 0;

        for i in 0..100 {
            let u = Point2f::new(rng.uniform_float(), rng.uniform_float());
            let mut wi = Vector3f::default();
            hg.sample_p(&wo, &mut wi, &u);

            if wi.x > 0.0 {
                nforward += 1;
            } else {
                nbackward += 1;
            }
        }

        assert!(nforward > 10 * nbackward);
    }

    #[test]
    fn sample_orientation_backward() {
        let mut rng = RNG::default();
        let hg = HenyeyGreenstein::new(-0.95);
        let wo = Vector3f::new(-1.0, 0.0, 0.0);
        let mut nforward = 0;
        let mut nbackward = 0;

        for i in 0..100 {
            let u = Point2f::new(rng.uniform_float(), rng.uniform_float());
            let mut wi = Vector3f::default();
            hg.sample_p(&wo, &mut wi, &u);

            if wi.x > 0.0 {
                nforward += 1
            } else {
                nbackward += 1;
            }
        }

        assert!(nbackward > 10 * nforward);
    }

    #[test]
    fn normalized() {
        let mut rng = RNG::default();
        let mut g = -0.75;

        while g <= 0.75 {
            let hg = HenyeyGreenstein::new(g);
            let wo =
                uniform_sample_sphere(&Point2f::new(rng.uniform_float(), rng.uniform_float()));
            let mut sum = 0.0;
            let nsamples = 100000;

            for i in 0..nsamples {
                let wi =
                    uniform_sample_sphere(&Point2f::new(rng.uniform_float(), rng.uniform_float()));
                sum += hg.p(&wo, &wi);
            }

            relative_eq!(sum / nsamples as Float, 1.0 / (4.0 * PI), epsilon = 1.0e-3);
            g += 0.25;
        }
    }
}