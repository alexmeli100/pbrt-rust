use log::{warn, error};
use crate::core::paramset::ParamSet;
use crate::core::geometry::normal::Normal3f;
use crate::core::pbrt::Float;
use crate::core::geometry::vector::{Vector3f, Vector2f};
use crate::core::geometry::point::{Point3f, Point2f};
use std::fmt::{Display, Formatter};
use std::fmt;
use anyhow::{Result, anyhow};
use lazy_static::lazy_static;
use lalrpop_util::lalrpop_mod;
use crate::pbrtparser::syntax::PBRTCommands;
use std::fs::File;
use std::io::prelude::Read;
use crate::pbrtparser::lexer::Lexer;
use crate::core::api::API;

lalrpop_mod!(pub commands);

lazy_static! {
    static ref PARSER: commands::CommandsParser = commands::CommandsParser::new();
}

pub fn parse(name: &str, api: &mut API) -> Result<()> {
    let commands = parse_file(name)?;

    for c in commands.into_iter() {
        match c {
            PBRTCommands::All                           => api.active_transform_all(),
            PBRTCommands::StartTime                     => api.active_transform_starttime(),
            PBRTCommands::EndTime                       => api.active_transform_endtime(),
            PBRTCommands::AttributeBegin                => api.attribute_begin(),
            PBRTCommands::AttributeEnd                  => api.attribute_end(),
            PBRTCommands::TransformBegin                => api.transform_begin(),
            PBRTCommands::TransformEnd                  => api.transform_end(),
            PBRTCommands::ObjectEnd                     => api.object_end(),
            PBRTCommands::WorldBegin                    => api.world_begin(),
            PBRTCommands::WorldEnd                      => api.world_end(),
            PBRTCommands::ReverseOrientation            => api.reverse_orientation(),
            PBRTCommands::Accelerator((s, p))           => api.accelerator(&s, p),
            PBRTCommands::ObjectBegin(s)                => api.object_begin(&s),
            PBRTCommands::ObjectInstance(s)             => api.object_instance(&s),
            PBRTCommands::LookAt(d)                     => api.lookat(d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7], d[8]),
            PBRTCommands::CoordSys(s)                   => api.coordinate_system(&s),
            PBRTCommands::Camera((s, p))                => api.camera(&s, p),
            PBRTCommands::Film((s, p))                  => api.film(&s, p),
            PBRTCommands::Integrator((s, p))            => api.integrator(&s, p),
            PBRTCommands::AreaLight((s, p))             => api.area_lightsource(&s, &p),
            PBRTCommands::LightSource((s, p))           => api.light_source(&s, &p),
            PBRTCommands::Material((s, p))              => api.material(&s, p),
            PBRTCommands::MakeNamedMaterial((s, p))     => api.make_named_material(&s, p),
            PBRTCommands::MakeNamedMedium((s, p))     => api.make_named_medium(&s, p),
            PBRTCommands::NamedMaterial(s)              => api.named_material(&s),
            PBRTCommands::Sampler((s, p))               => api.sampler(&s, p),
            PBRTCommands::Shape((s, p))                 => api.shape(&s, &p),
            PBRTCommands::Filter((s, p))                => api.pixel_filter(&s, p),
            PBRTCommands::Scale([x, y, z])              => api.scale(x, y, z),
            PBRTCommands::Rotate([a, x, y, z])          => api.rotate(a, x, y, z),
            PBRTCommands::Translate([x, y, z])          => api.translate(x, y, z),
            PBRTCommands::Texture(s)                    => api.texture(&s.name, &s.ty, &s.texname, &s.params),
            PBRTCommands::ConcatTransform(v)            => api.concat_transform(v),
            PBRTCommands::Transform(v)                  => api.transform(v),
            PBRTCommands::Include(s)                    => api.include(&s)?,
            PBRTCommands::CoordTransform(s)       => api.coord_sys_transform(&s),
            PBRTCommands::MediumInterface((s1, s2))       => {
                let empty = r#""""#;
                let x = if s1 == empty { "".to_owned() } else { s1 };
                let y = if s2 == empty { "".to_owned() } else { s2 };

                api.medium_interface(&x, &y);
            }

        }
    }

    Ok(())
}

pub fn parse_file(name: &str) -> Result<Vec<PBRTCommands>> {
    let mut file = File::open(name)?;
    let mut s = String::new();
    file.read_to_string(&mut s)?;

   PARSER.parse(Lexer::new(&s))
       .map_err(|e| anyhow!("error parsing file \"{}\"", name))
}

#[derive(Debug, PartialEq, Copy, Clone)]
pub enum ParamType {
    Int,
    Bool,
    Float,
    Vector2,
    Vector3,
    Point2,
    Point3,
    Normal,
    RGB,
    XYZ,
    BlackBody,
    Spectrum,
    String,
    Texture
}

pub enum ArrayVals {
    STR(Vec<String>),
    NUM(Vec<Float>)
}

impl Display for ParamType {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            ParamType::RGB => write!(f, "rgb/color"),
            _ => {
                let s = format!("{:?}", self).to_lowercase();
                write!(f, "{}", s)
            }
        }
    }
}

#[derive(Debug)]
pub struct ParamListItem {
    param_type    : ParamType,
    name          : String,
    double_values : Option<Vec<Float>>,
    string_values : Option<Vec<String>>,

}

impl ParamListItem {
    fn new(ty: ParamType, name: String, d: Option<Vec<Float>>, s: Option<Vec<String>>) -> Self {
        Self {
            name,
            param_type    : ty,
            double_values : d,
            string_values : s
        }
    }
}

pub fn params(params: Vec<ParamListItem>) -> ParamSet {
    let mut p = ParamSet::default();

    params.into_iter().for_each(|x| add_params(&mut p, x));

    p
}

pub fn param_item(s: String, vals: ArrayVals) -> ParamListItem {
    let mut split = s.split_whitespace();
    let typ = split.next().unwrap();
    let name = split.next().unwrap();

    let ty = match typ {
        "int"       => ParamType::Int,
        "bool"      => ParamType::Bool,
        "float"     => ParamType::Float,
        "vector2"   => ParamType::Vector2,
        "vector3"   => ParamType::Vector3,
        "point2"    => ParamType::Point2,
        "point3"    => ParamType::Point3,
        "normal"    => ParamType::Normal,
        "rgb/color" => ParamType::RGB,
        "xyz"       => ParamType::XYZ,
        "blackbody" => ParamType::BlackBody,
        "spectrum"  => ParamType::Spectrum,
        "string"    => ParamType::String,
        "texture"   => ParamType::Texture,
        _           => panic!("unknown parameter type {}", typ)
    };

    match vals {
        ArrayVals::STR(s)  => ParamListItem::new(ty, name.to_owned(), None, Some(s)),
        ArrayVals::NUM(n)  => ParamListItem::new(ty, name.to_owned(), Some(n), None)
    }
}


fn add_ints(p: &mut ParamSet, name: &str, ints: Vec<Float>) {
    let ins = ints.iter().map(|x| *x as isize).collect::<Vec<isize>>();
    let size = ins.len();

    p.add_int(name, ins, size);

}

fn add_bools(p: &mut ParamSet, name: &str, bools: Vec<String>) {
    let b = bools.iter().map(|x| {
        match x.as_str() {
            "true" => true,
            "false" => false,
            _ => {
                warn!("Value \"{}\" unknown for Boolean Parameter \"{}\". Using \"false\"", x, name);
                false
            }
        }
    });

    p.add_bool(name, b.collect(), bools.len())
}

fn add_float(p: &mut ParamSet, name: &str, floats: Vec<Float>) {
    let size = floats.len();
    p.add_float(name, floats, size);
}

fn add_point2(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    if floats.len() % 2 != 0 {
        warn!(
            "Excess value given with point2 parameter \"{}\". \
            Ignoring last one of them.",
            name);
        floats.pop();
    }

    let f = floats
        .chunks(2)
        .map(|fs| Point2f::new(fs[0], fs[1]))
        .collect::<Vec<Point2f>>();

    let size = f.len();
    p.add_point2f(name, f, size);
}

fn add_point3(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    let excess = floats.len() % 3;

    if excess != 0 {
        warn!(
            "Excess values given with point3 parameter \"{}\". \
            Ignoring last {} of them.",
            name, excess);


        floats.truncate(floats.len() - excess);
    }

    let f = floats
        .chunks(3)
        .map(|fs| Point3f::new(fs[0], fs[1], fs[2]))
        .collect::<Vec<Point3f>>();

    let size = f.len();
    p.add_point3f(name, f, size);
}

fn add_vector2(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    if floats.len() % 2 != 0 {
        warn!(
            "Excess value given with vector2 parameter \"{}\". \
            Ignoring last one of them.",
            name);
        floats.pop();
    }

    let f = floats
        .chunks(2)
        .map(|fs| Vector2f::new(fs[0], fs[1]))
        .collect::<Vec<Vector2f>>();

    let size = f.len();
    p.add_vector2f(name, f, size);
}

fn add_vector3(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    let excess = floats.len() % 3;

    if excess != 0 {
        warn!(
            "Excess values given with vector3 parameter \"{}\". \
            Ignoring last {} of them.",
            name, excess);


        floats.truncate(floats.len() - excess);
    }

    let f = floats
        .chunks(3)
        .map(|fs| Vector3f::new(fs[0], fs[1], fs[2]))
        .collect::<Vec<Vector3f>>();

    let size = f.len();
    p.add_vector3f(name, f, size);
}

fn add_normal(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    let excess = floats.len() % 3;

    if excess != 0 {
        warn!(
            "Excess values given with normal parameter \"{}\". \
            Ignoring last {} of them.",
            name, excess);

        floats.truncate(floats.len() - excess);
    }

    let f = floats
        .chunks(3)
        .map(|fs| Normal3f::new(fs[0], fs[1], fs[2]))
        .collect::<Vec<Normal3f>>();

    let size = f.len();
    p.add_normal3f(name, f, size);
}

fn add_rgb(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    let excess = floats.len() % 3;

    if excess != 0 {
        warn!(
            "Excess RGB values given with parameter \"{}\". \
            Ignoring last {} of them.",
            name, excess);


        floats.truncate(floats.len() - excess);
    }

    let size = floats.len();
    p.add_rgb_spectrum(name, floats, size);
}

fn add_xyz(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    let excess = floats.len() % 3;

    if excess != 0 {
        warn!(
            "Excess XYZ values given with parameter \"{}\". \
            Ignoring last {} of them.",
            name, excess);

        floats.truncate(floats.len() - excess);
    }

    let size = floats.len();
    p.add_xyz_spectrum(name, floats, size);
}

fn add_blackbody(p: &mut ParamSet, name: &str, mut floats: Vec<Float>) {
    if floats.len() % 2 != 0 {
        warn!(
            "Excess values given with parameter \"{}\". \
            Ignoring last one of them.",
            name);

        floats.pop();
    }

    let size = floats.len();
    p.add_blackbody_spectrum(name, &floats, size);
}

fn add_spectrum(p: &mut ParamSet, name: &str, mut item: ParamListItem) {
    if item.string_values.is_some() {
        let files = item.string_values.unwrap();
        p.add_sampled_spectrum_files(name, &files, files.len());
        return
    }

    let mut floats = item.double_values.unwrap();

    if floats.len() % 2 != 0 {
        warn!(
            "Non-even number of values given with sampled spectrum.  \
             parameter \"{}\". Ignoring extra",
            name);

        floats.pop();
    }

    let size = floats.len();
    p.add_sampled_spectrum(name, &floats, size);
}

fn add_strings(p: &mut ParamSet, name: &str, strings: Vec<String>) {
    let size = strings.len();
    p.add_string(name, strings, size);
}

fn add_texture(p: &mut ParamSet, name: &str, items: Vec<String>) {
    if items.len() != 1 {
        error!(
            "Only one parameter allowed for \"texture\" parameter \"{}\"",
            name
        )
    }

    p.add_texture(name, &items[0]);
}

fn add_params(p: &mut ParamSet, item: ParamListItem) {
    let ty = item.param_type;

    match ty {
        ParamType::Texture | ParamType::String | ParamType::Bool => {
            error!(
                "Expected string parameter value for parameter \
                \"{}\" with type \"{}\"",
                item.name, ty);
            return;
        }
        _ if ty != ParamType::Spectrum && item.string_values.is_some() => {
            error!(
                "Expected numeric parameter \
                value for parameter \"{}\" with type \"{}\"",
                item.name, ty);
            return;
        }
        _ => {}
    }

    match ty {
        ParamType::Spectrum   => add_spectrum(p, &item.name.to_owned(), item),
        ParamType::Int        => add_ints(p, &item.name, item.double_values.unwrap()),
        ParamType::Float      => add_float(p, &item.name, item.double_values.unwrap()),
        ParamType::String     => add_strings(p, &item.name, item.string_values.unwrap()),
        ParamType::Bool       => add_bools(p, &item.name, item.string_values.unwrap()),
        ParamType::Point2     => add_point2(p, &item.name, item.double_values.unwrap()),
        ParamType::Point3     => add_point3(p, &item.name, item.double_values.unwrap()),
        ParamType::Vector2    => add_vector2(p, &item.name, item.double_values.unwrap()),
        ParamType::Vector3    => add_vector3(p, &item.name, item.double_values.unwrap()),
        ParamType::Normal     => add_normal(p, &item.name, item.double_values.unwrap()),
        ParamType::XYZ        => add_xyz(p, &item.name, item.double_values.unwrap()),
        ParamType::RGB        => add_rgb(p, &item.name, item.double_values.unwrap()),
        ParamType::BlackBody  => add_blackbody(p, &item.name, item.double_values.unwrap()),
        ParamType::Texture    => add_texture(p, &item.name, item.string_values.unwrap())
    }
}