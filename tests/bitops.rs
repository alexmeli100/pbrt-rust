
#[cfg(test)]
mod bitops_test {
    use pbrt_rust::core::pbrt::{log2_int, log2_int64, is_power_of2, round_up_pow2_32, round_up_pow2_64};

    #[test]
    fn log2() {
        for i in 0..32 {
            let ui = 1u32 << i;
            assert_eq!(i, log2_int(ui as i32));
            assert_eq!(i as i64, log2_int64(ui as i64));
        }

        for i in 1..32 {
            let ui = 1u32 << i;
            assert_eq!(i, log2_int(ui as i32 + 1));
            assert_eq!(i as i64, log2_int64(ui as i64 + 1));
        }

        for i in 0..64 {
            let ui = 1u64 << i;
            assert_eq!(i as i64, log2_int64(ui as i64));
        }

        for i in 1..64 {
            let ui = 1u64 << i;
            assert_eq!(i as i64, log2_int64(ui as i64 + 1));
        }
    }

    #[test]
    fn round_up_pow2() {
        assert_eq!(round_up_pow2_32(7), 8);

        let mut i = 1;
        while i < (1 << 24) {
            if is_power_of2(i) {
                assert_eq!(round_up_pow2_32(i), i);
            } else {
                assert_eq!(round_up_pow2_32(i), 1 << (log2_int(i) + 1));
            }

            i += 1;
        }

        let mut i = 1i64;

        while i < (1 << 24) {
            if is_power_of2(i) {
                assert_eq!(round_up_pow2_64(i), i);
            } else {
                assert_eq!(round_up_pow2_64(i), 1 << (log2_int64(i) + 1));
            }

            i += 1
        }

        for i in 0..30i32 {
            let v = 1i32 << i;
            assert_eq!(round_up_pow2_32(v), v);
            if v > 2 { assert_eq!(round_up_pow2_32(v - 1), v); }
            assert_eq!(round_up_pow2_32(v + 1), 2 * v)
        }
    }
}