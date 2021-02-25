
#[cfg(test)]
mod fileutil {
    use pbrt_rust::core::fileutil::{has_extension, set_search_directory, resolve_filename, directory_containing, absolute_path};

    #[test]
    fn has_extention() {
        assert!(has_extension("foo.exr", "exr"));
        assert!(has_extension("foo.png", "png"));
        assert!(has_extension("foo.tga", "tga"));
        assert!(has_extension("foo.pfm", "pfm"));
    }

    #[test]
    fn test_utils() {
        let path = "C:\\Users\\alexm\\Documents\\code\\Rust\\pbrt-rust\\src\\scenes\\spheres-differentials-texfilt.pbrt";
        set_search_directory(directory_containing(path));
        let f = absolute_path(&resolve_filename("textures\\lines.png"));

        println!("{}", f);
    }
}