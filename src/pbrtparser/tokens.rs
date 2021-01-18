use crate::core::pbrt::Float;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ErrorCode {
    UnrecognizedToken,
    UnterminatedStringLiteral,
    ExpectedStringLiteral
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Error {
    pub location : usize,
    pub code     : ErrorCode
}

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
    MakeNamedMedium,
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