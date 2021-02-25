use crate::core::texture::{TextureMapping2Ds, Texture, TextureMapping2D, TextureFloat, get_mapping2d, TextureSpec, UVMapping2D};
use std::sync::Arc;
use std::any::Any;
use log::warn;
use std::boxed::Box;
use std::collections::BTreeMap;
use lazy_static::lazy_static;
use crate::core::mipmap::{MIPMap, ImageWrap, Clampable};
use crate::core::interaction::SurfaceInteraction;
use std::sync::Mutex;
use ordered_float::OrderedFloat;
use crate::core::pbrt::{Float, inverse_gamma_correct};
use crate::core::spectrum::{RGBSpectrum};
use crate::core::fileutil::has_extension;
use crate::core::geometry::point::{Point2i};
use crate::core::imageio::read_image;
use crate::core::geometry::vector::Vector2f;
use std::ops::{Mul, AddAssign, Div};
use crate::core::transform::Transform;
use crate::core::paramset::TextureParams;

lazy_static! {
    static ref TEXTURES: Mutex<BTreeMap<TexInfo, Box<dyn Any + Send + Sync>>> = Mutex::new(BTreeMap::new());
}

pub fn clear_cache() {
    let c = TEXTURES.lock();
    let mut cache = c.unwrap();
    cache.clear();
}

pub type ImageTextureFloat = ImageTexture<Float>;
pub type ImageTextureRGB = ImageTexture<RGBSpectrum>;


#[derive(PartialEq, Eq, Ord, PartialOrd)]
pub struct TexInfo {
    filename     : String,
    do_trilinear : bool,
    max_aniso    : OrderedFloat<Float>,
    scale        : OrderedFloat<Float>,
    gamma        : bool,
    wrap_mode    : ImageWrap
}

impl TexInfo {
    pub fn new(
        filename: String, do_trilinear: bool,
        max_aniso: Float, scale: Float,
        gamma: bool, wrap_mode: ImageWrap) -> Self {
        Self {
            filename, do_trilinear,
            gamma, wrap_mode,
            max_aniso: OrderedFloat::from(max_aniso),
            scale: OrderedFloat::from(scale)
        }
    }
}

pub struct ImageTexture<Tmemory>
where Tmemory: num::Zero + Copy + Send + Sync
{
    mapping : TextureMapping2Ds,
    mipmap  : Arc<MIPMap<Tmemory>>,
}

pub trait ConvertFrom<From> {
    fn convert_from(r: &RGBSpectrum, scale: Float, gamma: bool) -> Self;
}

impl ConvertFrom<RGBSpectrum> for Float {
    fn convert_from(r: &RGBSpectrum, scale: Float, gamma: bool) -> Self {
        let g = if gamma {
            inverse_gamma_correct(r.y())
        } else {
            r.y()
        };

        scale * g
    }
}

impl ConvertFrom<RGBSpectrum> for RGBSpectrum {

    fn convert_from(r: &RGBSpectrum, scale: f32, gamma: bool) -> Self {
        let g = if gamma {
            r.inverse_gamma_correct()
        } else {
            *r
        };

        g * scale
    }
}

impl<Tmemory: 'static> ImageTexture<Tmemory>
where Tmemory: num::Zero + Clone + Copy + ConvertFrom<RGBSpectrum> + Mul<Float, Output=Tmemory> + Div<Float, Output=Tmemory> + Clampable + AddAssign + Send + Sync
{
    pub fn new(
        mapping: TextureMapping2Ds, file: &str, do_trilinear: bool,
        max_aniso: Float, wrap: ImageWrap, scale: Float, gamma: bool) -> Self {
        let mipmap = ImageTexture::<Tmemory>::get_texture(file, do_trilinear, max_aniso, wrap, scale, gamma);

        Self { mipmap, mapping }
    }

    fn get_texture(
        filename: &str, do_trilinear: bool,
        max_aniso: Float, wrap: ImageWrap,
        scale: Float, gamma: bool) -> Arc<MIPMap<Tmemory>>  {
        let texinfo = TexInfo::new(
            filename.to_string(), do_trilinear,
            max_aniso, scale, gamma, wrap);

        let c = TEXTURES.lock();
        let mut cache = c.unwrap();
        let m= cache.get(&texinfo);

        if m.is_some() {
            let mip = m.as_ref().unwrap()
                .downcast_ref::<Arc<MIPMap<Tmemory>>>()
                .unwrap()
                .clone();

            return mip;
        }


        let (mut texels, res) = match read_image(filename) {
            Ok(res) => res,
            _ => {
                warn!("Creating a constant grey texture to replace \"{}\".", filename);
                (vec![RGBSpectrum::new(0.5); 1], Point2i::new(1, 1))
            }
        };

        // Flip image in y; texture coordinate space has (0, 0) at the lower
        // left corner
        for y in 0..res.y / 2 {
            for x in 0..res.x {
                let o1 = y * res.x + x;
                let o2 = (res.y - 1 - y) * res.x + x;
                texels.swap(o1 as usize, o2 as usize);
            }
        }

        let converted = texels.iter()
            .map(|spec| {
                let to: Tmemory = Tmemory::convert_from(spec, scale, gamma);
                to
            })
            .collect::<Vec<Tmemory>>();
        let mipmap = MIPMap::new(&res, converted, do_trilinear, max_aniso, wrap);

        let res = Arc::new(mipmap);
        cache.insert(texinfo, Box::new(res.clone()));

        res
    }
}

impl<Tmemory, Treturn> Texture<Treturn> for ImageTexture<Tmemory>
where Treturn: num::Zero + Clone + From<Tmemory>,
      Tmemory: num::Zero + Copy + Send + Sync + Mul<Float, Output=Tmemory> + Clampable + AddAssign + Div<Float, Output=Tmemory>
{
    fn evaluate(&self, s: &SurfaceInteraction) -> Treturn {
        let mut dstdx = Vector2f::default();
        let mut dstdy = Vector2f::default();
        let st = self.mapping.map(s, &mut dstdx, &mut dstdy);
        let mem = self.mipmap.lookup2(&st, dstdx, dstdy);

        Treturn::from(mem)
    }
}

pub fn create_image_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let map = get_mapping2d(t2w, tp);

    // Initialize ImageTexture parameters
    let max_aniso = tp.find_float("maxanisotropy", 8.0);
    let trilerp = tp.find_bool("trilinear", false);
    let wrap = tp.find_string("wrap", "repeat");

    let mode = match wrap.as_str() {
        "black" => ImageWrap::Black,
        "clamp" => ImageWrap::Clamp,
        _       => ImageWrap::Repeat
    };
    let scale = tp.find_float("scale", 1.0);
    let filename = tp.find_filename("filename", "");
    let ext = has_extension(&filename, "tga") || has_extension(&filename, "png");
    let gamma = tp.find_bool("gamma", ext);

    let img = ImageTexture::<Float>::new(
        map, &filename, trilerp, max_aniso,
        mode, scale, gamma).into();

    Some(Arc::new(img))
}

pub fn create_image_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let map = get_mapping2d(t2w, tp);

    // Initialize ImageTexture parameters
    let max_aniso = tp.find_float("maxanisotropy", 8.0);
    let trilerp = tp.find_bool("trilinear", false);
    let wrap = tp.find_string("wrap", "repeat");

    let mode = match wrap.as_str() {
        "black" => ImageWrap::Black,
        "clamp" => ImageWrap::Clamp,
        _       => ImageWrap::Repeat
    };
    let scale = tp.find_float("scale", 1.0);
    let filename = tp.find_filename("filename", "");
    let ext = has_extension(&filename, "tga") || has_extension(&filename, "png");
    let gamma = tp.find_bool("gamma", ext);

    let img: TextureSpec = ImageTexture::<RGBSpectrum>::new(
        map, &filename, trilerp,
        max_aniso, mode, scale, gamma).into();

    Some(Arc::new(img))
}

#[test]
fn test_image() {
    crate::core::stats::init_stats();
    crate::core::mipmap::init_stats();
    let uv: TextureMapping2Ds = UVMapping2D::new(1.0, 0.5, 1.0, 0.5).into();
    let tex = ImageTexture::<RGBSpectrum>::new(
        uv, "C:\\Users\\alexm\\Documents\\code\\Rust\\pbrt-rust\\src\\scenes\\textures\\lines.png", true,
        8.0, ImageWrap::Repeat, 1.0, true
    );


    let size = tex.mipmap.pyramid.len();
    println!("{:?}", tex.mipmap.pyramid[4]);
}
