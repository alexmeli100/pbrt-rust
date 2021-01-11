use crate::core::lexer::Tokens;
use anyhow::{Result, Context};
use combine::{Stream, token, Parser, satisfy_map, ParseError, choice, attempt, between, many1, value, many, stream::position, eof, StreamOnce};
use combine::parser::char::{spaces, string};
use crate::core::lexer;
use crate::core::api::API;
use crate::core::pbrt::Float;
use crate::core::paramset::ParamSet;
use crate::core::fileutil;
use log::{warn, error};
use std::fmt::Display;
use static_assertions::_core::fmt::Formatter;
use std::fmt;
use std::fs::File;
use std::io::prelude::*;
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::geometry::normal::Normal3f;
use combine::parser::combinator::{AndThen, AnyPartialState, and_then};
use combine::stream::StreamErrorFor;
use combine::error::{StreamError, UnexpectedParse};
use combine::lib::path::Path;
use std::rc::Rc;
use std::cell::RefCell;

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

enum ArrayVals {
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
struct ParamListItem {
    param_type: ParamType,
    name: String,
    double_values: Option<Vec<Float>>,
    string_values: Option<Vec<String>>,

}

impl ParamListItem {
    fn new(ty: ParamType, name: String, d: Option<Vec<Float>>, s: Option<Vec<String>>) -> Self {
        Self {
            param_type: ty,
            name,
            double_values: d,
            string_values: s
        }
    }
}

pub fn tokenize_file<P: AsRef<Path>>(file: P) -> Result<Vec<Tokens>> {
    let filename = fileutil::resolve_filename(file.as_ref().to_str().unwrap());
    let mut f = File::open(&filename)
        .with_context(|| format!("Failed to open scene file \"{}\"", filename))?;
    let mut contents = String::new();
    f.read_to_string(&mut contents)
        .with_context(|| format!("Failed to read content of scene file \"{}\"", filename))?;

    let tokens = lexer::tokenize(position::Stream::new(&*contents))
        .with_context(|| format!("Failed to tokenize scene file \"{}\"", filename))?;

    let filtered_tokens = tokens.0
        .into_iter()
        .filter(|x| *x != Tokens::Comment)
        .collect();

    Ok(filtered_tokens)
}

pub fn parse<I: Stream<Token=Tokens>>(input: I, a: Rc<RefCell<&mut API>>) -> Result<()>
where
    I: StreamOnce,
    I::Position: Default,
    I::Error: ParseError<I::Token, I::Range, I::Position>
{
    let accelerator = (token(Tokens::Accelerator), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().accelerator(&name, p)));


    let attribute_begin = token(Tokens::AttributeBegin).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().attribute_begin()));


    let attribute_end = token(Tokens::AttributeEnd).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().attribute_end()));


    let transform_begin = token(Tokens::TransformBegin).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().transform_begin()));


    let transform_end = token(Tokens::TransformEnd).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().transform_end()));


    let object_begin = (token(Tokens::ObjectBegin), string_())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| Ok(a.clone().borrow_mut().object_begin(&name)));


    let object_end = token(Tokens::ObjectEnd).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().object_end()));

    let object_instance = (token(Tokens::ObjectInstance), string_())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| Ok(a.clone().borrow_mut().object_instance(&name)));


    let world_begin = token(Tokens::WorldBegin).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().world_begin()));


    let world_end = token(Tokens::WorldEnd).and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().world_end()));


    let lookat = (
        token(Tokens::LookAt),
        num(), num(), num(), num(), num(), num(), num(), num(), num()
    )
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, ex, ey, ez, lx, ly, lz, ux, uy, uz)|
            Ok(a.clone().borrow_mut().lookat(ex, ey, ez, lx, ly, lz, ux, uy, uz)));

    let coord_sys = (token(Tokens::CoordinateSystem), string_())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| Ok(a.clone().borrow_mut().coord_sys_transform(&name)));

    let coord_transform = (token(Tokens::CoordSysTransform), string_())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| Ok(a.clone().borrow_mut().coord_sys_transform(&name)));

    let camera = (token(Tokens::Camera), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().camera(&name, p)));

    let film = (token(Tokens::Film), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().film(&name, p)));

    let include = (token(Tokens::Include), string_()).and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| {
        match tokenize_file(&name) {
            Ok(ts) => {
                match parse(&ts[..],a.clone()) {
                    Ok(_) => Ok(()),
                    Err(e) => panic!("Failed to parse included file: {}", e)
                }
            },
            Err(e) => panic!("{:?}", e)
        }
    });

    let integrator = (token(Tokens::Integrator), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().integrator(&name, p)));

    let area_light = (token(Tokens::AreaLightSource), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().area_lightsource(&name, &p)));

    let light_source = (token(Tokens::LightSource), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().light_source(&name, &p)));

    let material = (token(Tokens::Material), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, params)| Ok(a.clone().borrow_mut().material(&name, params)));

    let make_material = (token(Tokens::MakeNamedMaterial), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().make_named_material(&name, p)));

    let named_material = (token(Tokens::NamedMaterial), string_())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name)| Ok(a.clone().borrow_mut().named_material(&name)));

    let sampler = (token(Tokens::Sampler), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().sampler(&name, p)));

    let shape = (token(Tokens::Shape), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().shape(&name, &p)));

    let filter = (token(Tokens::PixelFilter), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, p)| Ok(a.clone().borrow_mut().pixel_filter(&name, p)));

    let reverse_orientation = (token(Tokens::ReverseOrientation))
        .and_then::<_, _, StreamErrorFor::<I>>(|_| Ok(a.clone().borrow_mut().reverse_orientation()));

    let scale = (token(Tokens::Scale), num(), num(), num())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, x, y, z)| Ok(a.clone().borrow_mut().scale(x, y, z)));

    let translate = (token(Tokens::Translate), num(), num(), num())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, dx, dy, dz)| Ok(a.clone().borrow_mut().translate(dx, dy, dz)));

    let rotate = (token(Tokens::Rotate), num(), num(), num(), num())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, angle, dx, dy, dz)| Ok(a.clone().borrow_mut().rotate(angle, dx, dy, dz)));

    let texture = (token(Tokens::Texture), string_(), string_(), string_(), params())
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, name, ty, texname, p)| Ok(a.clone().borrow_mut().texture(&name, &ty, &texname, &p)));

    let concat_transform = (token(Tokens::ConcatTransform), val_array(num))
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, nums)| Ok(a.clone().borrow_mut().concat_transform(nums)));

    let transform = (token(Tokens::Transform), val_array(num))
        .and_then::<_, _, StreamErrorFor::<I>>(|(_, nums)| Ok(a.clone().borrow_mut().transform(nums)));


    let p1 = choice((
        attempt(accelerator),
        attempt(attribute_begin),
        attempt(attribute_end),
        attempt(transform_begin),
        attempt(transform_end),
        attempt(object_instance),
        attempt(object_begin),
        attempt(object_end),
        attempt(world_begin),
        attempt(world_end),
        attempt(lookat),
        attempt(coord_sys),
        attempt(coord_transform),
        attempt(camera),
        attempt(film),
        attempt(filter),
        attempt(include),
        attempt(integrator),
        attempt(area_light),
        attempt(light_source),
        attempt(material),
        attempt(texture),
        attempt(translate),
        attempt(concat_transform),
        attempt(transform)
    ));

    let p2 = choice((
        attempt(make_material),
        attempt(named_material),
        attempt(sampler),
        attempt(shape),
        attempt(reverse_orientation),
        attempt(scale),
        attempt(rotate),
    ));

    let parsers = many1::<Vec<_>, _, _>(choice((p1, p2)));

    (parsers, eof()).parse(input)
        .map(|(_, _)| ())
        .map_err(|e: <I as StreamOnce>::Error| ParseError::into_other::<UnexpectedParse>(e))
        .with_context(|| format!("Failed to parse scene"))?;

    Ok(())
}

pub fn num<I>() -> impl Parser<I, Output = Float>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    satisfy_map(|tok| match tok {
        Tokens::Number(n) => Some(n),
        _ => None
    })
}

pub fn string_<I>() -> impl Parser<I, Output = String>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    satisfy_map(|tok| match tok {
        Tokens::STR(s) => Some(s),
        _ => None
    })
}

pub fn val_array<I, O, F, R>(p: F) -> impl Parser<I, Output = Vec<O>>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>,
          R: Parser<I, Output = O>,
          F: Fn() -> R
{
    choice!(
        attempt(between(
            token(Tokens::LeftBracket),
            token(Tokens::RightBracket),
            many1(p())
        )),
        attempt(p().map(|x| vec![x]))
    )
}

fn array<I>() -> impl Parser<I, Output = ArrayVals>
    where
        I: Stream<Token = Tokens>,
        I::Error: ParseError<I::Token, I::Range, I::Position>,
{
    choice!(
        attempt(val_array(num).map(ArrayVals::NUM)),
        attempt(val_array(string_).map(ArrayVals::STR))
    )
}

fn param_type<I>() -> impl Parser<I, Output = ParamType>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    choice!(
    attempt(string("integer").with(value(ParamType::Int))),
    attempt(string("bool").with(value(ParamType::Bool))),
    attempt(string("float").with(value(ParamType::Float))),
    attempt(string("string").with(value(ParamType::String))),
    attempt(string("point2").with(value(ParamType::Point2))),
    attempt(string("point3").with(value(ParamType::Point3))),
    attempt(string("vector2").with(value(ParamType::Vector2))),
    attempt(string("vector3").with(value(ParamType::Vector3))),
    attempt(string("normal").with(value(ParamType::Normal))),
    attempt(string("rgb/color").with(value(ParamType::RGB))),
    attempt(string("xyz").with(value(ParamType::XYZ))),
    attempt(string("blackbody").with(value(ParamType::BlackBody))),
    attempt(string("spectrum").with(value(ParamType::Spectrum))),
    attempt(string("texture").with(value(ParamType::Texture))))
}

type Header = (ParamType, String);

fn param_header<I>() -> impl Parser<I, Output = Header>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>,
{
    string_()
        .and_then(|s| param_type().skip(spaces()).parse(&s[..])
            .map(|(ty, name)| (ty, name.to_string()))
            .map_err(StreamErrorFor::<I>::other))

}

fn param_list<I>() -> impl Parser<I, Output = ParamListItem>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    (param_header(), array())
        .map(|(header, array)| {

            let (ty, name) = header;

            match array {
                ArrayVals::STR(s) => ParamListItem::new(ty, name, None, Some(s)),
                ArrayVals::NUM(n) => ParamListItem::new(ty, name, Some(n), None)
            }
        })
}

fn params<I>() -> impl Parser<I, Output = ParamSet>
    where I: Stream<Token = Tokens>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    many::<Vec<ParamListItem>, _, _>(param_list())
        .map(|s| {
            let mut p = ParamSet::default();

            for item in s {
                add_params(&mut p, item)
            }

            p
        })
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
