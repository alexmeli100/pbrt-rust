
#[cfg(test)]
mod parser {
    use pbrt_rust::pbrtparser::pbrtparser::{PARSER, parse_file};
    use pbrt_rust::pbrtparser::lexer::Lexer;

    #[test]
    fn test_parser() {
        let input = r#"
Integrator "path" "integer maxdepth" [4]

Sampler "halton" "integer pixelsamples" [512]

PixelFilter "gaussian"

# Film "image" "integer xresolution" [1440] "integer yresolution" [1440]
#     "string filename" "ganesha.exr"
Film "image" "integer xresolution" [720] "integer yresolution" [720]
    "string filename" "ganesha.exr"

Scale -1 1 1
LookAt 328.0 40.282 245.0 328.0 10.0 0.0 -0.00212272 0.998201 -0.0599264
Camera "perspective" "float fov" [30.0]

WorldBegin
Shape "trianglemesh" "point P" [-672.0 -41.99995803833008 1000.0 1328.0 -41.99995803833008 1000.0 1328.0 -42.00004196166992 -1000.0 -672.0 -42.00004196166992 -1000.0] "integer indices" [0 1 2 0 2 3]

AttributeBegin
    Transform [0 0 -1 0 1 0 0 0 0 1 0 0 0 0 0 1]
    LightSource "infinite" "string mapname" ["textures/sky.hdr"] "color scale" [0.1 0.1 0.1]
AttributeEnd

AttributeBegin
    AreaLightSource "area" "color L" [15.258818626403809 12.083925247192383 9.589462280273438]
    Shape "trianglemesh" "point P" [220.0 61.36730194091797 -343.6283264160156 400.0 61.36730194091797 -343.6283264160156 400.0 238.6326904296875 -312.3716735839844 220.0 238.6326904296875 -312.3716735839844] "integer indices" [0 1 2 0 2 3]
AttributeEnd

AttributeBegin
    AreaLightSource "area" "color L" [15.258818626403809 12.083925247192383 9.589462280273438]
    ReverseOrientation
    Shape "trianglemesh" "point P" [220.0 61.36730194091797 343.6283264160156 400.0 61.36730194091797 343.6283264160156 400.0 238.6326904296875 312.3716735839844 220.0 238.6326904296875 312.3716735839844] "integer indices" [0 1 2 0 2 3]
AttributeEnd

Texture "tmap" "color" "imagemap" "string filename" ["textures/ganesha.png"]

AttributeBegin
    Material "substrate" "texture Kd" "tmap" "color Ks" [0.04 0.04 0.04]
        "float uroughness" [0.01] "float vroughness" [0.01]
        "bool remaproughness" ["false"]
    Shape "plymesh" "string filename" ["geometry/ganesha.ply"]
AttributeEnd
WorldEnd

"#;

        let res = PARSER.parse(Lexer::new(input));
        assert!(res.is_ok());

        let commands = res.unwrap();

        for c in commands.iter() {
            println!("{:?}", c);
        }

    }

    #[test]
    fn test_parser_from_file() {
        let file = "killeroo.pbrt";
        let res = parse_file(file);
        println!("{:?}", res);
        assert!(res.is_ok());

        let commands = res.unwrap();

        for c in commands.iter() {
            println!("{:?}", c);
        }
    }


}