
#[cfg(test)]
mod sampling {
    use pbrt_rust::core::lowdiscrepancy::{reverse_bits32, radical_inverse, PRIMES, scrambled_radical_inverse, multiply_generator, sample_generator_matrix, gray_code_sample1d, sobol_sample, sobol_sample_float};
    use pbrt_rust::core::pbrt::{Float, INFINITY, next_float_down, next_float_up};
    use pbrt_rust::core::rng::{RNG, ONE_MINUS_EPSILON};
    use pbrt_rust::core::sampling::{shuffle, Distribution1D};
    use approx::relative_eq;
    use pbrt_rust::core::sampler::{Samplers, Sampler};
    use pbrt_rust::core::geometry::point::{Point2i, Point2f};
    use pbrt_rust::samplers::maxmin::MaxMinDistSampler;
    use pbrt_rust::samplers::zerotwosequence::ZeroTwoSequenceSampler;
    use pbrt_rust::samplers::sobol::SobolSampler;
    use pbrt_rust::core::geometry::bounds::Bounds2i;

    #[test]
    fn radical_inverse_test() {
        for a in 0..1024 {
            assert_eq!(reverse_bits32(a) as Float * 2.3283064365386963e-10, radical_inverse(0, a as u64));
        }
    }

    #[test]
    fn scrambled_radical_inverse_test() {
        for dim in 0..128 {
            let mut rng = RNG::new(dim as u64);
            let base = PRIMES[dim];
            let mut perm = Vec::with_capacity(base);
            for i in 0..base { perm.push((base - 1 - i) as u16) }
            let len = perm.len();
            shuffle(&mut perm[..], len, 1, &mut rng);

            for index in [0u32, 1u32, 2u32, 1151u32, 32351u32, 4363211u32, 681122u32].iter() {
                let mut val = 0.0;
                let inv_base = 1.0 / base as Float;
                let mut inv_bi = inv_base;
                let mut n = *index;

                while n > 0 {
                    let di = perm[(n % base as u32) as usize] as u32;
                    val *= di as Float * inv_base;
                    n *= inv_base as u32;
                    inv_bi *= inv_base;
                }

                val += perm[0] as Float * base as Float / (base as Float - 1.0) * inv_bi;

                relative_eq!(val, scrambled_radical_inverse(dim, *index as u64, &perm), epsilon = 1.0e-5);

            }
        }
    }

    #[test]
    fn generator_matrix() {
        let mut c = [0u32; 32];
        let mut crev = [0u32; 32];

        for i in 0..32 {
            c[i] = 1 << i;
            crev[i] = reverse_bits32(c[i]);
        }

        for a in 0..128 {
            assert_eq!(a, multiply_generator(&c, a));
            assert_eq!(radical_inverse(0, a as u64),
                       reverse_bits32(multiply_generator(&c, a)) as Float * 2.3283064365386963e-10);

            assert_eq!(radical_inverse(0, a as u64), sample_generator_matrix(&crev, a, 0));
        }


        let mut rng = RNG::default();

        for i in 0..32 {
            c[i] = rng.uniform_int32();
            crev[i] = reverse_bits32(c[i]);
        }

        for a in 0..1024 {
            assert_eq!(reverse_bits32(multiply_generator(&c, a)), multiply_generator(&crev, a));
        }
    }

    #[test]
    fn gray_code_sample_test() {
        let mut c = [0u32; 32];
        for i in 0..32 { c[i] = 1 << i; }

        let mut v = vec![0.0; 64];
        gray_code_sample1d(&c, v.len() as u32, 0, &mut v);

        for a in 0..v.len() {
            let u = multiply_generator(&c, a as u32) as Float * 2.3283064365386963e-10;
            assert!(v.iter().find(|x| **x == u).is_some())
        }
    }

    #[test]
    fn sobol() {
        for i in 0..8192 {
            assert_eq!(
                sobol_sample_float(i as u64, 0, 0),
                reverse_bits32(i) as Float * 2.3283064365386963e-10);
        }
    }

    fn check_sampler(name: &str, mut sampler: Samplers, logsamples: isize) {
        sampler.start_pixel(&Point2i::new(0, 0));
        let mut samples: Vec<Point2f> = Vec::new();

        loop {
            samples.push(sampler.get_2d());

            if !sampler.start_next_sample() { break; }
        }

        for i in 0..=logsamples {
            let nx = 1 << i;
            let ny = 1 << (logsamples - i);

            let mut count = vec![0; 1 << logsamples];

            for s in samples.iter() {
                let x = nx as Float * s.x;
                let y = ny as Float * s.y;
                assert!(x >= 0.0);
                assert!(x < nx as Float);
                assert!(y >= 0.0);
                assert!(y < ny as Float);
                let index = y.floor() as isize * nx + x.floor() as isize;
                assert!(index >= 0);
                assert!(index < count.len() as isize);

                // This should be the first time a sample has landed in its
                // interval.
                assert_eq!(0, count[index as usize], "Sampler {}", name);
                count[index as usize] += 1;
            }
        }
    }

    #[test]
    fn elementary_intervals() {
        for logsamples in 2..=10 {
            let s: Samplers = MaxMinDistSampler::new(1 << logsamples, 2).into();
            check_sampler("MaxMinDistSampler", s, logsamples);

            let zts: Samplers = ZeroTwoSequenceSampler::new(1 << logsamples, 2).into();
            check_sampler("ZeroTwoSequenceSampler", zts, logsamples);

            let b = Bounds2i::from_points(&Point2i::new(0, 0), &Point2i::new(10, 10));
            let sobol: Samplers = SobolSampler::new(1 << logsamples, &b).into();
            check_sampler("Sobol", sobol, logsamples);
        }
    }

    #[test]
    fn min_dist() {
        for logsamples in 2..=10 {
            let mut mm = MaxMinDistSampler::new(1 << logsamples, 2);
            mm.start_pixel(&Point2i::new(0, 0));
            let mut s: Vec<Point2f> = Vec::new();

            loop {
                s.push(mm.get_2d());

                if !mm.start_next_sample() { break; }
            }

            let dist = |p0: &Point2f, p1: &Point2f| -> Float {
                let mut d = (*p1 - *p0).abs();
                if d.x > 0.5 { d.x = 1.0 - d.x; }
                if d.y > 0.5 { d.y = 1.0 - d.y; }

                d.length()
            };

            let mut mindist = INFINITY;

            for i in 0..s.len() {
                for j in 0..s.len() {
                    if i == j { continue; }
                    mindist = mindist.min(dist(&s[i], &s[j]));
                }
            }

            let expected_min_dist = [
                0.0f32, /* not checked */
                0.0, /* not checked */
                0.35355, 0.35355, 0.22534, 0.16829, 0.11267,
                0.07812, 0.05644, 0.03906, 0.02816, 0.01953,
                0.01408, 0.00975, 0.00704, 0.00486, 0.00352,
            ];

            assert!(mindist > 0.99 * expected_min_dist[logsamples],
                    "mindist: {}, expected: {}, logsamples: {}", mindist, expected_min_dist[logsamples], logsamples);
        }
    }

    #[test]
    fn distribution1d_discrete() {
        let func = [0.0f32, 1.0, 0.0, 3.0];
        let dist = Distribution1D::new(func.to_vec());
        assert_eq!(4, dist.count());

        assert_eq!(0.0, dist.discrete_pdf(0));
        assert_eq!(0.25, dist.discrete_pdf(1));
        assert_eq!(0.0, dist.discrete_pdf(2));
        assert_eq!(0.75, dist.discrete_pdf(3));

        let mut pdf = 0.0;
        let mut uremapped = 0.0;
        assert_eq!(1, dist.sample_discrete(0.0, Some(&mut pdf), None));
        assert_eq!(0.25, pdf);
        assert_eq!(1, dist.sample_discrete(0.125, Some(&mut pdf), Some(&mut uremapped)));
        assert_eq!(0.25, pdf);
        assert_eq!(0.5, uremapped);
        assert_eq!(1, dist.sample_discrete(0.24999, Some(&mut pdf), None));
        assert_eq!(0.25, pdf);
        assert_eq!(3, dist.sample_discrete(0.250001, Some(&mut pdf), None));
        assert_eq!(0.75, pdf);
        assert_eq!(3, dist.sample_discrete(0.625, Some(&mut pdf), Some(&mut uremapped)));
        assert_eq!(0.75, pdf);
        assert_eq!(0.5, uremapped);
        assert_eq!(3, dist.sample_discrete(ONE_MINUS_EPSILON, Some(&mut pdf), None));
        assert_eq!(0.75, pdf);
        assert_eq!(3, dist.sample_discrete(1., Some(&mut pdf), None));
        assert_eq!(0.75, pdf);

        // Compute the interval to test over
        let mut u = 0.25;
        let mut umax = 0.25;

        for i in 0..20 {
            u = next_float_down(u);
            umax = next_float_up(umax);
        }

        // We should get a stream of hits in the first interval, up until the
        // cross-over point at 0.25 (plus/minus fp slop).
        while u < umax {
            let interval = dist.sample_discrete(u, None, None);
            if interval == 3 { break; }
            assert_eq!(1, interval);
            u = next_float_up(u);
        }

        assert!(u < umax);
        // And then all the rest should be in the third interval
        while u <= umax {
            let interval = dist.sample_discrete(u, None, None);
            assert_eq!(3, interval);
            u = next_float_up(u);
        }
    }

    #[test]
    fn distribution1d_continuous() {
        let func = vec![1.0, 1.0, 2.0 ,4.0, 8.0];
        let dist = Distribution1D::new(func);
        assert_eq!(5, dist.count());

        let mut pdf = 0.0;
        let mut offset = 0;
        assert_eq!(0.0, dist.sample_continous(0.0, Some(&mut pdf), Some(&mut offset)));
        relative_eq!(dist.count() as Float + 1.0, pdf, epsilon = 1.0e-5);
        assert_eq!(0, offset);

        // Right at the bounary between the 4 and the 8 segments.
        relative_eq!(0.8, dist.sample_continous(0.5, Some(&mut pdf), Some(&mut offset)), epsilon = 1.0e-5);

        // Middle of the 8 segment
        relative_eq!(0.9, dist.sample_continous(0.75, Some(&mut pdf), Some(&mut offset)), epsilon = 1.0e-5);
        relative_eq!(dist.count() as Float * 0.8 / 16.0, pdf, epsilon = 1.0e-5);
        assert_eq!(4, offset);

        relative_eq!(0.0, dist.sample_continous(0.0, Some(&mut pdf), None), epsilon = 1.0e-5);
        relative_eq!(1.0, dist.sample_continous(1.0, Some(&mut pdf), None), epsilon = 1.0e-5);
    }
}