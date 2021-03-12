#![cfg_attr(feature = "cargo-clippy", allow(clippy::trivial_regex))]

use regex::Regex;
use lazy_static::lazy_static;
use crate::pbrtparser::tokens::*;

pub type Spanned<T> = (usize, T, usize);

lazy_static! {
    static ref ATTRIBUTEBEGIN       : Regex = Regex::new(r"^AttributeBegin"    ).unwrap();
    static ref ATTRIBUTEEND         : Regex = Regex::new(r"^AttributeEnd"      ).unwrap();
    static ref ACTIVETRANSFORM      : Regex = Regex::new(r"^ActiveTransform"   ).unwrap();
    static ref ALL                  : Regex = Regex::new(r"^All"               ).unwrap();
    static ref STARTTIME            : Regex = Regex::new(r"^StartTime"         ).unwrap();
    static ref ENDTIME              : Regex = Regex::new(r"^EndTime"           ).unwrap();
    static ref AREALIGHTSOURCE      : Regex = Regex::new(r"^AreaLightSource"   ).unwrap();
    static ref ACCELERATOR          : Regex = Regex::new(r"^Accelerator"       ).unwrap();
    static ref CONCATTRANSFORM      : Regex = Regex::new(r"^ConcatTransform"   ).unwrap();
    static ref COORDINATESYSTEM     : Regex = Regex::new(r"^CoordinateSystem"  ).unwrap();
    static ref COORDSYSTRANSFORM    : Regex = Regex::new(r"^CoordSysTransform" ).unwrap();
    static ref CAMERA               : Regex = Regex::new(r"^Camera"            ).unwrap();
    static ref FILM                 : Regex = Regex::new(r"^Film"              ).unwrap();
    static ref INTEGRATOR           : Regex = Regex::new(r"^Integrator"        ).unwrap();
    static ref INCLUDE              : Regex = Regex::new(r"^Include"           ).unwrap();
    static ref IDENTITY             : Regex = Regex::new(r"^Identity"          ).unwrap();
    static ref LIGHTSOURCE          : Regex = Regex::new(r"^LightSource"       ).unwrap();
    static ref LOOKAT               : Regex = Regex::new(r"^LookAt"            ).unwrap();
    static ref MATERIAL             : Regex = Regex::new(r"^Material"          ).unwrap();
    static ref MAKENAMEDMATERIAL    : Regex = Regex::new(r"^MakeNamedMaterial" ).unwrap();
    static ref NAMEDMATERIAL        : Regex = Regex::new(r"^NamedMaterial"     ).unwrap();
    static ref MEDIUMINTERFACE      : Regex = Regex::new(r"^MediumInterface"   ).unwrap();
    static ref OBJECTBEGIN          : Regex = Regex::new(r"^ObjectBegin"       ).unwrap();
    static ref OBJECTEND            : Regex = Regex::new(r"^ObjectEnd"         ).unwrap();
    static ref OBJECTINSTANCE       : Regex = Regex::new(r"^ObjectInstance"    ).unwrap();
    static ref PIXELFILTER          : Regex = Regex::new(r"^PixelFilter"       ).unwrap();
    static ref REVERSEORIENTATION   : Regex = Regex::new(r"^ReverseOrientation").unwrap();
    static ref ROTATE               : Regex = Regex::new(r"^Rotate"            ).unwrap();
    static ref SHAPE                : Regex = Regex::new(r"^Shape"             ).unwrap();
    static ref SAMPLER              : Regex = Regex::new(r"^Sampler"           ).unwrap();
    static ref SCALE                : Regex = Regex::new(r"^Scale"             ).unwrap();
    static ref TRANSFORMBEGIN       : Regex = Regex::new(r"^TransformBegin"    ).unwrap();
    static ref TRANSFORMEND         : Regex = Regex::new(r"^TransformEnd"      ).unwrap();
    static ref TRANSFORM            : Regex = Regex::new(r"^Transform"         ).unwrap();
    static ref TRANSLATE            : Regex = Regex::new(r"^Translate"         ).unwrap();
    static ref TRANSFORMTIMES       : Regex = Regex::new(r"^TransformTimes"    ).unwrap();
    static ref WORLDBEGIN           : Regex = Regex::new(r"^WorldBegin"        ).unwrap();
    static ref WORLDEND             : Regex = Regex::new(r"^WorldEnd"          ).unwrap();
    static ref TEXTURE              : Regex = Regex::new(r"^Texture"           ).unwrap();
    static ref LEFTBRACKET          : Regex = Regex::new(r"^\["                ).unwrap();
    static ref RIGHTBRACKET         : Regex = Regex::new(r"^\]"                ).unwrap();
    static ref STR                  : Regex = Regex::new(r#"^"[^"]*""#         ).unwrap();
    static ref NUMBER               : Regex = Regex::new(r"^[+-]?(\d+([.]\d*)?([eE][+-]?\d+)?|[.]\d+([eE][+-]?\d+)?)"       ).unwrap();
    static ref COMMENT              : Regex = Regex::new(r#"^#[^\n]*\n"#       ).unwrap();
    static ref NEWLINE              : Regex = Regex::new(r"^\n"                ).unwrap();
    static ref WS                   : Regex = Regex::new(r#"^[[:space:]]"#             ).unwrap();
}

pub struct Lexer<'input> {
    text : &'input str,
    line : usize,
    pos  : usize
}

impl<'input> Lexer<'input> {
    pub fn new(text: &'input str) -> Lexer<'input> {
        Lexer {
            text,
            line : 1,
            pos  : 0
        }
    }

    fn match_and_consume<F>(
        text: &mut &'input str, pos: &mut usize,
        re: &Regex, action: F) -> Option<Tokens> where F: Fn(&'input str) -> Tokens {
        if let Some(mat) = re.find(text) {
            *pos += mat.end();
            let ret = Some(action(&text[mat.start()..mat.end()]));
            *text = &text[mat.end()..];
            ret
        } else {
            None
        }
    }

    pub fn next_token(&mut self) -> Option<Tokens> {
        loop {
            if let Some(mat) = COMMENT.find(self.text) {
                self.line += 1;
                self.pos += mat.end();
                self.text = &self.text[mat.end()..];
                continue;
            } else if let Some(mat) = NEWLINE.find(self.text) {
                self.line += 1;
                self.pos += mat.end();
                self.text = &self.text[mat.end()..];
                continue;
            } else if let Some(mat) = WS.find(self.text) {
                self.pos += mat.end();
                self.text = &self.text[mat.end()..];
                continue
            }

            break;
        }

        macro_rules! actions {
            ($($x:expr => $y:expr),*) => {
                if false { None }
                $(
                    else if let t@Some(_) = Lexer::match_and_consume(&mut self.text, &mut self.pos, &$x, $y) { t }
                )*
                else { None }
            }
        }

        actions![
            NUMBER              => |s: &'input str| Tokens::Number(s.parse::<crate::core::pbrt::Float>().unwrap()),
            STR                 => |s: &'input str| Tokens::STR(s[1..s.len() - 1].to_owned()),
            ATTRIBUTEBEGIN      => |_| Tokens::AttributeBegin,
            ATTRIBUTEEND        => |_| Tokens::AttributeEnd,
            ACTIVETRANSFORM     => |_| Tokens::ActiveTransform,
            ALL                 => |_| Tokens::All,
            STARTTIME           => |_| Tokens::StartTime,
            ENDTIME             => |_| Tokens::EndTime,
            AREALIGHTSOURCE     => |_| Tokens::AreaLightSource,
            ACCELERATOR         => |_| Tokens::Accelerator,
            CONCATTRANSFORM     => |_| Tokens::ConcatTransform,
            COORDINATESYSTEM    => |_| Tokens::CoordinateSystem,
            COORDSYSTRANSFORM   => |_| Tokens::CoordSysTransform,
            CAMERA              => |_| Tokens::Camera,
            FILM                => |_| Tokens::Film,
            INTEGRATOR          => |_| Tokens::Integrator,
            INCLUDE             => |_| Tokens::Include,
            IDENTITY            => |_| Tokens::Identity,
            LIGHTSOURCE         => |_| Tokens::LightSource,
            LOOKAT              => |_| Tokens::LookAt,
            MATERIAL            => |_| Tokens::Material,
            MAKENAMEDMATERIAL   => |_| Tokens::MakeNamedMaterial,
            NAMEDMATERIAL       => |_| Tokens::NamedMaterial,
            MEDIUMINTERFACE     => |_| Tokens::MediumInterface,
            OBJECTBEGIN         => |_| Tokens::ObjectBegin,
            OBJECTEND           => |_| Tokens::ObjectEnd,
            OBJECTINSTANCE      => |_| Tokens::ObjectInstance,
            PIXELFILTER         => |_| Tokens::PixelFilter,
            REVERSEORIENTATION  => |_| Tokens::ReverseOrientation,
            ROTATE              => |_| Tokens::Rotate,
            SHAPE               => |_| Tokens::Shape,
            SAMPLER             => |_| Tokens::Sampler,
            SCALE               => |_| Tokens::Scale,
            TRANSFORMBEGIN      => |_| Tokens::TransformBegin,
            TRANSFORMEND        => |_| Tokens::TransformEnd,
            TRANSFORM           => |_| Tokens::Transform,
            TRANSLATE           => |_| Tokens::Translate,
            TRANSFORMTIMES      => |_| Tokens::TransformTimes,
            WORLDBEGIN          => |_| Tokens::WorldBegin,
            WORLDEND            => |_| Tokens::WorldEnd,
            TEXTURE             => |_| Tokens::Texture,
            LEFTBRACKET         => |_| Tokens::LeftBracket,
            RIGHTBRACKET        => |_| Tokens::RightBracket
        ]
    }
}

impl<'input> Iterator for Lexer<'input> {
    type Item = Result<Spanned<Tokens>, Error>;

    fn next(&mut self) -> Option<Self::Item> {
        match self.next_token() {
            Some(t) => Some(Ok((0, t, 0))),
            None    => None
        }
    }
}

