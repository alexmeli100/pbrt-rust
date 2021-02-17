
#[cfg(test)]
mod find_interval {
    use pbrt_rust::core::pbrt::{find_interval, Float};

    #[test]
    fn find_interval_test() {
        let a = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0];

        // Check clamping for out of range
        assert_eq!(0, find_interval(a.len() as i32, |i| { a[i as usize] <= -1.0 }));
        assert_eq!((a.len() - 2) as i32, find_interval(a.len() as i32, |i| { a[i as usize] <= 100.0 }));

        for i in 0..(a.len() - 1) {
            assert_eq!(i as i32, find_interval(a.len() as i32, |j| a[j as usize] <= i as Float));
            assert_eq!(i as i32, find_interval(a.len() as i32, |j| a[j as usize] <= i as Float + 0.5));

            if i > 0 {
                assert_eq!( (i - 1) as i32, find_interval(a.len() as i32, |j| a[j as usize] <= i as Float - 0.5));
            }
        }
    }
}