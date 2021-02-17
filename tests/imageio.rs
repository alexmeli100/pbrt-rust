
#[cfg(test)]
mod imageio {
    use pbrt_rust::core::geometry::point::Point2i;
    use pbrt_rust::core::pbrt::{Float, inverse_gamma_correct};
    use pbrt_rust::core::imageio::{write_image, read_image};
    use pbrt_rust::core::geometry::bounds::Bounds2i;
    use pbrt_rust::core::fileutil::has_extension;

    fn test_round_trip(fname: &str, gamma: bool) {
        let res = Point2i::new(16, 29);
        let mut pixels = vec![0.0; 3 * res.x as usize * res.y as usize];

        for y in 0..res[1] {
            for x in 0..res[0] {
                let offset = 3 * (y * res[0] + x) as usize;
                pixels[offset] = x as Float / (res[0] - 1) as Float;
                pixels[offset + 1] = y as Float / (res[1] - 1) as Float;
                pixels[offset + 2] = -1.5;
            }
        }

        let obounds = Bounds2i::from_points(&Point2i::default(), &res);
        let r = write_image(fname, &pixels, &obounds, &res);
        assert!(r.is_ok());

        let r = read_image(fname);
        assert!(r.is_ok());
        let (readpixs, readres) = r.unwrap();
        assert_eq!(readres, res);

        for y in 0..res[1] {
            for x in 0..res[0] {
                let mut rgb = readpixs[(y * res[0] + x) as usize].to_rgb();

                for c in 0..3 {
                    if gamma { rgb[c] = inverse_gamma_correct(rgb[c]) }

                    let wrote = pixels[3 * (y * res[0] + x) as usize + c];
                    let delta = wrote - rgb[c];

                    if has_extension(fname, "pfm") {
                        assert_eq!(
                            0.0, delta,
                            "{}: ({}, {}) c = {} wrote {}, read {}, delta = {}",
                            fname, x, y, c, wrote, rgb[c], delta);
                    } else if has_extension(fname, "exr") {
                        if c == 2 {
                            assert_eq!(
                                0.0, delta,
                                "({}, {}) c = {} wrote {}, read {}, delta = {}",
                                x, y, c, wrote, rgb[c], delta);
                        } else {
                            assert!(
                                delta.abs() < 0.001,
                                "{}: ({}, {}) c = {} wrote {}, read {}, delta = {}",
                                fname, x, y, c, wrote, rgb[c], delta);
                        }
                    } else {

                        if c == 2 {
                            assert_eq!(
                                0.0, rgb[c],
                                "({}, {}) c = {} wrote {}, read {}, (expected 0 back)",
                                x, y, c, wrote, rgb[c]);
                        } else {
                            // Allow a fair amount of slop, since there's an sRGB
                            // conversion before quantization to 8-bits...
                            assert!(delta.abs() < 0.02,
                                    "{}: ({}, {}) c = {} wrote {}, read {}, delta = {}",
                                    fname, x, y, c, wrote, rgb[c], delta)
                        }
                    }
                }
            }
        }

        let r = std::fs::remove_file(fname);
        assert!(r.is_ok());
    }

    #[test]
    fn roundtrip_exr() {
        test_round_trip("out.exr", false);
    }

    #[test]
    fn roundtrip_pfm() {
        test_round_trip("out.pfm", false);
    }

    #[test]
    fn roundtrip_tga() {
        test_round_trip("out.tga", true);
    }

    #[test]
    fn roundtrip_png() {
        test_round_trip("out.png", true);
    }
}