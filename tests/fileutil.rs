
#[cfg(test)]
mod fileutil {
    use pbrt_rust::core::fileutil::has_extension;

    #[test]
    fn has_extention() {
        assert!(has_extension("foo.exr", "exr"));
        assert!(has_extension("foo.png", "png"));
        assert!(has_extension("foo.tga", "tga"));
        assert!(has_extension("foo.pfm", "pfm"));
    }
}