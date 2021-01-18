use crate::core::paramset::ParamSet;
use crate::core::pbrt::Float;

pub enum PBRTCommands {
    All,
    StartTime,
    EndTime,
    Accelerator(StringParams),
    AttributeBegin,
    AttributeEnd,
    TransformBegin,
    TransformEnd,
    ObjectBegin(String),
    ObjectEnd,
    ObjectInstance(String),
    WorldBegin,
    WorldEnd,
    LookAt([Float; 9]),
    CoordSys(String),
    CoordTransform(String),
    Camera(StringParams),
    Film(StringParams),
    Include(String),
    Integrator(StringParams),
    AreaLight(StringParams),
    LightSource(StringParams),
    Material(StringParams),
    MakeNamedMaterial(StringParams),
    MakeNamedMedium(StringParams),
    NamedMaterial(String),
    MediumInterface((String, String)),
    Sampler(StringParams),
    Shape(StringParams),
    Filter(StringParams),
    ReverseOrientation,
    Scale([Float; 3]),
    Translate([Float; 3]),
    Rotate([Float; 4]),
    Texture(TextureInfo),
    ConcatTransform(Vec<Float>),
    Transform(Vec<Float>)

}

pub struct TextureInfo {
    pub name    : String,
    pub ty      : String,
    pub texname : String,
    pub params  : ParamSet
}

type StringParams = (String, ParamSet);
