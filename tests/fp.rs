
#[cfg(test)]
mod fp_tests {
    use pbrt_rust::core::rng::RNG;
    use pbrt_rust::core::pbrt::{Float, bits_to_float, next_float_up, next_float_down, INFINITY, float_to_bits, lerp};
    use float_next_after::NextAfter;
    use pbrt_rust::core::parallel::AtomicFloat;
    use pbrt_rust::core::efloat::EFloat;

    fn get_float(rng: &mut RNG) -> Float {
        let mut f: Float;

        loop {
            f = bits_to_float(rng.uniform_int32());

            if !f.is_nan() { break; }
        }

        f
    }


    #[test]
    fn next_float_up_down() {
        assert!(next_float_up(-0.0) > 0.0);
        assert!(next_float_down(0.0) < 0.0);

        assert_eq!(next_float_up(INFINITY), INFINITY);
        assert!(next_float_down(INFINITY) < INFINITY);

        assert_eq!(next_float_down(-INFINITY), -INFINITY);
        assert!(next_float_up(-INFINITY) > -INFINITY);

        let mut rng = RNG::default();

        for i in 0..100000 {
            let f = get_float(&mut rng);
            if f.is_infinite() { continue; }

            assert_eq!(f.next_after(INFINITY), next_float_up(f));
            assert_eq!(f.next_after(-INFINITY), next_float_down(f));
        }

    }

    #[test]
    fn float_bits() {
        let mut rng = RNG::new(1);

        for i in 0..100000 {
            let ui = rng.uniform_int32();
            let f = bits_to_float(ui);
            if f.is_nan() { continue; }

            assert_eq!(ui, float_to_bits(f));
        }
    }

    #[test]
    fn atomic_float() {
        let af = AtomicFloat::new(0.0);
        let mut f = 0.0;
        assert_eq!(f, Float::from(af.clone()));
        af.add(1.0251);
        f += 1.0251;
        assert_eq!(f, Float::from(af.clone()));
        af.add(2.0);
        f += 2.0;
        assert_eq!(f, Float::from(af))
    }

    fn get_efloat(rng: &mut RNG, min_exp: Float, max_exp: Float) -> EFloat {
        let logu = lerp(rng.uniform_float(), min_exp, max_exp);
        let val = 10.0f32.powf(logu);

        // Choose a random error bound
        let mut err = 0.0;

        err = match rng.uniform_int32_2(4) {
            1 => {
                let ulp_err = rng.uniform_int32_2(1024);
                let offset = bits_to_float(float_to_bits(val) + ulp_err);

                (offset - val).abs()
            },
            2 => {
                let ulp_err = rng.uniform_int32_2(1024 * 1024);
                let offset = bits_to_float(float_to_bits(val) + ulp_err);

                (offset - val).abs()
            },
            3 => (4.0 * rng.uniform_float()) * val.abs(),
            _ => err
        };

        let sign = if rng.uniform_float() < 0.5 { -1.0 } else { 1.0 };

        EFloat::new(sign * val, err)
    }

    fn get_precise(ef: &EFloat, rng: &mut RNG) -> f64 {
        match rng.uniform_int32_2(3) {
            0 => ef.lower_bound() as f64,
            1 => ef.upper_bound() as f64,
            2 => {
                let t = rng.uniform_float();
                let mut p = (1.0 - t as f64) * ef.lower_bound() as f64 + t as f64 * ef.upper_bound() as f64;
                if p > ef.upper_bound() as f64 { p = ef.upper_bound() as f64; }
                if p < ef.lower_bound() as f64 { p = ef.lower_bound() as f64; }

                p
            }
            _ => Float::from(*ef) as f64
        }
    }

    const KE_FLOAT_ITERS: usize = 1000000;

    macro_rules! get_efloat {
        ($rng:expr) => {{
            get_efloat($rng, -6.0, 6.0)
        }}
    }

    #[test]
    fn efloat_abs() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = get_efloat!(&mut rng);
            let precise = get_precise(&ef, &mut rng);

            let ef_result = ef.abs();
            let precise_res = precise.abs();

            assert!(precise_res >= ef_result.lower_bound() as f64);
            assert!(precise_res <= ef_result.upper_bound() as f64);
        }
    }

    #[test]
    fn efloat_sqrt() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = get_efloat!(&mut rng);
            let precise = get_precise(&ef, &mut rng);

            if ef.get_absolute_error() > 0.25 * ef.lower_bound().abs() { continue; }

            let ef_res = ef.abs().sqrt();
            let presice_res = precise.abs().sqrt();

            assert!(presice_res >= ef_res.lower_bound() as f64);
            assert!(presice_res <= ef_res.upper_bound() as f64);
        }
    }

    #[test]
    fn add() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = [get_efloat!(&mut rng), get_efloat!(&mut rng)];
            let precise = [get_precise(&ef[0], &mut rng), get_precise(&ef[1], &mut rng)];

            let ef_res = ef[0] + ef[1];
            let precise_res = precise[0] + precise[1];

            assert!(precise_res >= ef_res.lower_bound() as f64);
            assert!(precise_res <= ef_res.upper_bound() as f64);
        }
    }

    #[test]
    fn sub() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = [get_efloat!(&mut rng), get_efloat!(&mut rng)];
            let precise = [get_precise(&ef[0], &mut rng), get_precise(&ef[1], &mut rng)];

            let ef_res = ef[0] - ef[1];
            let precise_res = precise[0] - precise[1];

            assert!(precise_res >= ef_res.lower_bound() as f64);
            assert!(precise_res <= ef_res.upper_bound() as f64);
        }
    }

    #[test]
    fn mul() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = [get_efloat!(&mut rng), get_efloat!(&mut rng)];
            let precise = [get_precise(&ef[0], &mut rng), get_precise(&ef[1], &mut rng)];

            let ef_res = ef[0] * ef[1];
            let precise_res = precise[0] * precise[1];

            assert!(precise_res >= ef_res.lower_bound() as f64);
            assert!(precise_res <= ef_res.upper_bound() as f64);
        }
    }

    #[test]
    fn div() {
        for trial in 0..KE_FLOAT_ITERS {
            let mut rng = RNG::new(trial as u64);

            let ef = [get_efloat!(&mut rng), get_efloat!(&mut rng)];
            let precise = [get_precise(&ef[0], &mut rng), get_precise(&ef[1], &mut rng)];

            if ef[1].lower_bound() * ef[1].upper_bound() < 0.0 ||
               ef[1].get_absolute_error() > 0.25 * ef[1].lower_bound().abs() {
                continue;
            }

            let ef_res = ef[0] / ef[1];
            let precise_res = precise[0] / precise[1];

            assert!(precise_res >= ef_res.lower_bound() as f64);
            assert!(precise_res <= ef_res.upper_bound() as f64);
        }
    }
}