use std::fmt;

use combine::parser::char::{char, digit, spaces, string};
use combine::{between, choice, eof, many, none_of, optional, satisfy, skip_many, token, attempt, ParseError, Parser, Stream, skip_many1};
use crate::core::pbrt::Float;
use combine::parser::combinator::recognize;
use std::str;
use std::fmt::{Display, Formatter};
use std::iter::FromIterator;

#[derive(Debug, Clone, PartialEq)]
pub enum Tokens {
    AttributeBegin,
    AttributeEnd,
    ActiveTransform,
    All,
    EndTime,
    StartTime,
    AreaLightSource,
    Accelerator,
    ConcatTransform,
    CoordinateSystem,
    CoordSysTransform,
    Camera,
    Film,
    Integrator,
    Include,
    Identity,
    LightSource,
    LookAt,
    MakeNamedMaterial,
    Material,
    MediumInterface,
    NamedMaterial,
    ObjectBegin,
    ObjectEnd,
    ObjectInstance,
    PixelFilter,
    ReverseOrientation,
    Rotate,
    Shape,
    Sampler,
    Scale,
    TransformBegin,
    TransformEnd,
    Transform,
    Translate,
    TransformTimes,
    Texture,
    WorldBegin,
    WorldEnd,
    LeftBracket,
    RightBracket,
    Number(Float),
    STR(String),
    Comment
}

impl Display for Tokens {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self)
    }
}

pub fn tokenize<I>(input: I) -> Result<(Vec<Tokens>, I), I::Error>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    let mut parsers = vec![
        parse_token(Tokens::AttributeBegin, "AttributeBegin"),
        parse_token(Tokens::AttributeEnd, "AttributeEnd"),
        parse_token(Tokens::ActiveTransform, "ActiveTransform"),
        parse_token(Tokens::All, "All"),
        parse_token(Tokens::EndTime, "EndTime"),
        parse_token(Tokens::StartTime, "StartTime"),
        parse_token(Tokens::AreaLightSource, "AreaLightSource"),
        parse_token(Tokens::Accelerator, "Accelator"),
        parse_token(Tokens::ConcatTransform, "ConcatTransform"),
        parse_token(Tokens::CoordinateSystem, "CoordinateSystem"),
        parse_token(Tokens::CoordSysTransform, "CoordSysTransform"),
        parse_token(Tokens::Camera, "Camera"),
        parse_token(Tokens::Film, "Film"),
        parse_token(Tokens::Integrator, "Integrator"),
        parse_token(Tokens::Include, "Include"),
        parse_token(Tokens::Identity, "Identity"),
        parse_token(Tokens::LightSource, "LightSource"),
        parse_token(Tokens::LookAt, "LookAt"),
        parse_token(Tokens::MakeNamedMaterial, "MakeNamedMaterial"),
        parse_token(Tokens::Material, "Material"),
        parse_token(Tokens::MediumInterface, "MediumInterface"),
        parse_token(Tokens::NamedMaterial, "NamedMaterial"),
        parse_token(Tokens::ObjectBegin, "ObjectBegin"),
        parse_token(Tokens::ObjectEnd, "ObjectEnd"),
        parse_token(Tokens::ObjectInstance, "ObjectInstance"),
        parse_token(Tokens::PixelFilter, "PixelFilter"),
        parse_token(Tokens::ReverseOrientation, "ReverseOrientation"),
        parse_token(Tokens::Rotate, "Rotate"),
        parse_token(Tokens::Shape, "Shape"),
        parse_token(Tokens::Sampler, "Sampler"),
        parse_token(Tokens::Scale, "Scale"),
        parse_token(Tokens::TransformBegin, "TransformBegin"),
        parse_token(Tokens::TransformEnd, "TransformEnd"),
        parse_token(Tokens::Transform, "Transform"),
        parse_token(Tokens::Translate, "Translate"),
        parse_token(Tokens::TransformTimes, "TransformTimes"),
        parse_token(Tokens::Texture, "Texture"),
        parse_token(Tokens::WorldBegin, "WorldBegin"),
        parse_token(Tokens::WorldEnd, "WorldEnd"),
        parse_token(Tokens::LeftBracket, "LeftBracket"),
        parse_token(Tokens::RightBracket, "RightBracket"),
        //parse_float()
    ].into_iter().
        map(attempt).
        collect::<Vec<_>>();

    let parser = choice!(
        choice(parsers.as_mut_slice()),
        attempt(parse_float()),
        attempt(parse_string()),
        attempt(parse_comment())
    );

    (spaces().with(many(parser)), eof())
        .map(|(res, _)| res)
        .parse(input)
}

fn parse_token<I>(t: Tokens, s: &'static str) -> impl Parser<I, Output=Tokens>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    string(s).skip(spaces()).map(move |_| t.clone())
}

fn parse_float<I>() -> impl Parser<I, Output=Tokens>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>

{
    let float_parser = recognize((
        optional(char('-').or(char('+'))),
        skip_many1(digit()),
        optional((char('.'), skip_many(digit()))),
        optional((
            char('e').or(char('E')),
            optional(char('e').or(char('E'))),
            skip_many1(digit())))
    ))
        .skip(spaces())
        .map(|res: Vec<char>| {
            let s: String = res.into_iter().collect();
            let num = s.parse::<Float>().unwrap();

            Tokens::Number(num)
        });

    float_parser
}



fn parse_string<I>() -> impl Parser<I, Output = Tokens>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    between(
        token('"'),
        token('"'),
        many(none_of("\"".chars()))
    )
        .skip(spaces())
        .map(|chars: Vec<char>| {
            let s = String::from_iter(chars);
            Tokens::STR(s)
        })
}

fn parse_comment<I>() -> impl Parser<I, Output = Tokens>
    where I: Stream<Token = char>,
          I::Error: ParseError<I::Token, I::Range, I::Position>
{
    token('#')
        .and(skip_many(satisfy(|c| c != '\n')))
        .skip(spaces())
        .map(|_| Tokens::Comment)
}

