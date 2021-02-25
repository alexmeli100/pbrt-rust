use enum_dispatch::enum_dispatch;
use log::error;
use crate::textures::dots::DotsTexture;
use crate::textures::constant::ConstantTexture;
use crate::textures::scaled::ScaleTexture;
use crate::core::interaction::SurfaceInteraction;
use crate::textures::imagemap::{ ImageTextureFloat, ImageTextureRGB};
use crate::textures::mix::MixTexture;
use crate::textures::biler::BilerTexture;
use crate::textures::uv::UVTexture;
use crate::textures::marble::MarbleTexture;
use crate::textures::wrinkled::WrinkledTexture;
use crate::textures::fbm::FBmTexture;
use crate::textures::windy::WindyTexture;
use crate::textures::checkerboard::{Checkerboard3DTexture, Checkerboard2DTexture};
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::pbrt::{Float, INV_PI, INV2_PI, PI, lerp, clamp};
use crate::core::transform::Transform;
use crate::core::geometry::geometry::{spherical_theta, spherical_phi};
use crate::core::spectrum::{Spectrum, RGBSpectrum, SampledSpectrum};
use std::ops::{Mul, Add, AddAssign, Div};
use crate::core::mipmap::Clampable;
use crate::core::paramset::TextureParams;

const NOISE_PERM_SIZE: usize = 256;
const NOISE_PERM: [usize; 2 * NOISE_PERM_SIZE] = [
    151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194, 233, 7, 225, 140,
    36, 103, 30, 69, 142,
    // Remainder of the noise permutation table
    8, 99, 37, 240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
    94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56, 87, 174,
    20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139, 48, 27, 166, 77,
    146, 158, 231, 83, 111, 229, 122, 60, 211, 133, 230, 220, 105, 92, 41, 55,
    46, 245, 40, 244, 102, 143, 54, 65, 25, 63, 161, 1, 216, 80, 73, 209, 76,
    132, 187, 208, 89, 18, 169, 200, 196, 135, 130, 116, 188, 159, 86, 164, 100,
    109, 198, 173, 186, 3, 64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147,
    118, 126, 255, 82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28,
    42, 223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153, 101,
    155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79, 113, 224, 232,
    178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242, 193, 238, 210, 144, 12,
    191, 179, 162, 241, 81, 51, 145, 235, 249, 14, 239, 107, 49, 192, 214, 31,
    181, 199, 106, 157, 184, 84, 204, 176, 115, 121, 50, 45, 127, 4, 150, 254,
    138, 236, 205, 93, 222, 114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66,
    215, 61, 156, 180, 151, 160, 137, 91, 90, 15, 131, 13, 201, 95, 96, 53, 194,
    233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37, 240, 21, 10, 23, 190, 6,
    148, 247, 120, 234, 75, 0, 26, 197, 62, 94, 252, 219, 203, 117, 35, 11, 32,
    57, 177, 33, 88, 237, 149, 56, 87, 174, 20, 125, 136, 171, 168, 68, 175, 74,
    165, 71, 134, 139, 48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60,
    211, 133, 230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
    63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200, 196, 135,
    130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3, 64, 52, 217, 226,
    250, 124, 123, 5, 202, 38, 147, 118, 126, 255, 82, 85, 212, 207, 206, 59,
    227, 47, 16, 58, 17, 182, 189, 28, 42, 223, 183, 170, 213, 119, 248, 152, 2,
    44, 154, 163, 70, 221, 153, 101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19,
    98, 108, 110, 79, 113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251,
    34, 242, 193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
    14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204, 176, 115,
    121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222, 114, 67, 29, 24, 72,
    243, 141, 128, 195, 78, 66, 215, 61, 156, 180
];

pub type TextureFloat = Textures<Float, Float>;
pub type TextureSpec = Textures<Spectrum, Spectrum>;

#[enum_dispatch]
pub trait Texture<T2> {
    fn evaluate(&self, s: &SurfaceInteraction) -> T2;
}

// All Texture generic types must implement these traits
pub trait SpectrumT<T>:
    Copy +
    Send +
    Sync +
    num::Zero +
    Clampable +
    AddAssign +
    From<Float> +
    From<SampledSpectrum> +
    From<RGBSpectrum> +
    Mul<T, Output = T> +
    Mul<Float, Output = T> +
    Div<Float, Output = T> +
    Add<T, Output = T>{}

// Implementations for valid Texture generic types
impl SpectrumT<Float> for Float{}
impl SpectrumT<RGBSpectrum> for RGBSpectrum{}
impl SpectrumT<SampledSpectrum> for SampledSpectrum{}

#[enum_dispatch(Texture<T2>)]
pub enum Textures<T1, T2>
where T1: SpectrumT<T1> + Mul<T2, Output = T2>,
      T2: SpectrumT<T2> + From<T1>
{
    MarbleTexture,
    UVTexture,
    FBmTexture,
    WrinkledTexture,
    WindyTexture,
    MixTexture(MixTexture<T2>),
    BilerTexture(BilerTexture<T2>),
    ScaleTexture(ScaleTexture<T1, T2>),
    DotsTexture(DotsTexture<T2>),
    ImageTextureFloat(ImageTextureFloat),
    ImageTextureRGB(ImageTextureRGB),
    ConstantTexture(ConstantTexture<T2>),
    Checkerboard2DTexture(Checkerboard2DTexture<T2>),
    Checkerboard3DTexture(Checkerboard3DTexture<T2>)
}

#[enum_dispatch]
pub trait TextureMapping2D {
    fn map(&self, si: &SurfaceInteraction,
           dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f;
}

#[enum_dispatch(TextureMapping2D)]
pub enum TextureMapping2Ds {
    UVMapping2D,
    PlannarMapping2D,
    SphericalMapping2D,
    CylindricalMapping2D
}

pub struct UVMapping2D {
    su: Float,
    sv: Float,
    du: Float,
    dv: Float,
}

impl UVMapping2D {
    pub fn new(su: Float, sv: Float, du: Float, dv: Float) -> Self {
        Self { su, sv, du, dv }
    }
}

impl TextureMapping2D for UVMapping2D {
    fn map(&self, si: &SurfaceInteraction,
           dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        // Compute texture differentials for sphere (u, v) mapping
        *dstdx = Vector2f::new(self.su * si.dudx.get() , self.sv * si.dvdx.get());
        *dstdy = Vector2f::new(self.su * si.dudy.get(), self.sv * si.dvdy.get());

        Point2f::new(self.su * si.uv[0] + self.du, self.sv * si.uv[1] + self.dv)
    }
}

impl Default for UVMapping2D {
    fn default() -> Self {
        Self {
            su: 1.0,
            sv: 1.0,
            du: 0.0,
            dv: 0.0
        }
    }
}

pub struct SphericalMapping2D {
    world_to_texture: Transform
}

impl SphericalMapping2D {
    pub fn new(wtt: &Transform) -> Self {
        Self { world_to_texture: *wtt }
    }

    fn sphere(&self, p: &Point3f) -> Point2f {
        let vec = (
            self.world_to_texture.transform_point(p) -
            Point3f::new(0.0, 0.0, 0.0))
            .normalize();
        let theta = spherical_theta(&vec);
        let phi = spherical_phi(&vec);

        Point2f::new(theta * INV_PI, phi * INV2_PI)
    }
}

impl TextureMapping2D for SphericalMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f,
           dstdy: &mut Vector2f) -> Point2f {
        let st = self.sphere(&si.p);

        // Compute texture coordinate differentials for sphere (u, v) mapping
        let delta = 0.1;
        let st_deltax = self.sphere(&(si.p + si.dpdx.get() * delta));
        *dstdx = (st_deltax - st) / delta;
        let st_deltay = self.sphere(&(si.p + si.dpdy.get() * delta));
        *dstdy = (st_deltay - st) / delta;

        // Handle sphere mapping discontinuity for coordinate differentials
        if dstdx[1] > 0.5 { dstdx[1] = 1.0 - dstdx[1]; }
        else if (*dstdx)[1] < -0.5 { (*dstdx)[1] = -((*dstdx)[1] + 1.0); }
        if dstdy[1] > 0.5 { dstdy[1] = 1.0 - dstdy[1]; }
        else if dstdy[1] < -0.5 { dstdy[1] = -(dstdy[1] + 1.0); }

        st
    }
}

pub struct CylindricalMapping2D {
    world_to_texture: Transform
}

impl CylindricalMapping2D {
    pub fn new(wtt: &Transform) -> Self {
        Self { world_to_texture: *wtt }
    }

    fn cylinder(&self, p: &Point3f) -> Point2f {
        let vec = (
            self.world_to_texture.transform_point(p) -
            Point3f::new(0.0, 0.0, 0.0))
            .normalize();

        Point2f::new(PI + vec.y.atan2(vec.x) * INV2_PI, vec.z)
    }
}

impl TextureMapping2D for CylindricalMapping2D {
    fn map(&self, si: &SurfaceInteraction,
           dstdx: &mut Vector2f, dstdy: &mut Vector2f) -> Point2f {
        let st = self.cylinder(&si.p);

        // Compute texture coordinate differentials for cylinder (u, v) mapping
        let delta = 0.1;
        let st_deltax = self.cylinder(&(si.p + si.dpdx.get() * delta));
        *dstdx = (st_deltax - st) / delta;
        let st_deltay = self.cylinder(&(si.p + si.dpdy.get() * delta));
        *dstdy = (st_deltay - st) / delta;

        // Handle sphere mapping discontinuity for coordinate differentials
        if dstdx[1] > 0.5 { dstdx[1] = 1.0 - dstdx[1]; }
        else if (*dstdx)[1] < -0.5 { (*dstdx)[1] = -((*dstdx)[1] + 1.0); }
        if dstdy[1] > 0.5 { dstdy[1] = 1.0 - dstdy[1]; }
        else if dstdy[1] < -0.5 { dstdy[1] = -(dstdy[1] + 1.0); }

        st
    }
}

pub struct PlannarMapping2D {
    vs: Vector3f,
    vt: Vector3f,
    ds: Float,
    dt: Float
}

impl PlannarMapping2D {
    pub fn new(vs: &Vector3f, vt: &Vector3f,
               ds: Float, dt: Float) -> Self {
        Self {
            ds,
            dt,
            vs: *vs,
            vt: *vt
        }
    }
}

impl TextureMapping2D for PlannarMapping2D {
    fn map(&self, si: &SurfaceInteraction, dstdx: &mut Vector2f,
           dstdy: &mut Vector2f) -> Point2f {
        let vec = Vector3f::from(si.p);
        *dstdx = Vector2f::new(si.dpdx.get().dot(&self.vs), si.dpdx.get().dot(&self.vt));
        *dstdy = Vector2f::new(si.dpdy.get().dot(&self.vs), si.dpdy.get().dot(&self.vt));

        Point2f::new(self.ds + vec.dot(&self.vs), self.dt + vec.dot(&self.vt))
    }
}

#[enum_dispatch]
pub trait TextureMapping3D {
    fn map(&self, si: &SurfaceInteraction, dpdx: &mut Vector3f,
           dpdy: &mut Vector3f) -> Point3f;
}

#[enum_dispatch(TextureMapping3D)]
pub enum TextureMapping3Ds {
    IdentityMapping3D
}

pub struct IdentityMapping3D {
    world_to_texture: Transform
}

impl IdentityMapping3D {
    pub fn new(w2t: &Transform) -> Self {
        Self { world_to_texture: *w2t }
    }
}

impl TextureMapping3D for IdentityMapping3D {
    fn map(&self, si: &SurfaceInteraction, dpdx: &mut Vector3f,
           dpdy: &mut Vector3f) -> Point3f {
        *dpdx = self.world_to_texture.transform_vector(&si.dpdx.get());
        *dpdy = self.world_to_texture.transform_vector(&si.dpdy.get());

        self.world_to_texture.transform_point(&si.p)

    }
}

pub fn lanczos(mut x: Float, tau: Float) -> Float {
    x = x.abs();
    if x < 1.0e-5 { return 1.0; }
    if x > 1.0 { return 0.0; }
    x *= PI;
    let s = (x * tau).sin() / ( x * tau);
    let lanc = x.sin() / x;

    s * lanc
}

pub fn noise(x: Float, y: Float, z: Float) -> Float {
    let (mut ix, mut iy, mut iz) = (
        x.floor() as usize,
        y.floor() as usize,
        z.floor() as usize);

    let (dx, dy, dz) = (
        x - ix as Float,
        y - iy as Float,
        z - iz as Float);

    // Compute gradient weights
    ix &= NOISE_PERM_SIZE - 1;
    iy &= NOISE_PERM_SIZE - 1;
    iz &= NOISE_PERM_SIZE - 1;
    let w000 = grad(ix, iy, iz, dx, dy, dz);
    let w100 = grad(ix + 1, iy, iz, dx - 1.0, dy, dz);
    let w010 = grad(ix, iy + 1, iz, dx, dy - 1.0, dz);
    let w110 = grad(ix + 1, iy + 1, iz, dx - 1.0, dy - 1.0, dz);
    let w001 = grad(ix, iy, iz + 1, dx, dy, dz - 1.0);
    let w101 = grad(ix + 1, iy, iz + 1, dx - 1.0, dy, dz - 1.0);
    let w011 = grad(ix, iy + 1, iz + 1, dx, dy - 1.0, dz - 1.0);
    let w111 = grad(ix + 1, iy + 1, iz + 1, dx - 1.0, dy - 1.0, dz - 1.0);

    // Compute trilinear interpolation of weights
    let (wx, wy, wz) = (
        noise_weight(dx),
        noise_weight(dy),
        noise_weight(dz));

    let x00 = lerp(wx, w000, w100);
    let x10 = lerp(wx, w010, w110);
    let x01 = lerp(wx, w001, w101);
    let x11 = lerp(wx, w011, w111);
    let y0 = lerp(wy, x00, x10);
    let y1 = lerp(wy, x01, x11);

    lerp(wz, y0, y1)
}

pub fn noisep(p: Point3f) -> Float {
    noise(p.x, p.y, p.z)
}

fn grad(x: usize, y: usize, z: usize, dx: Float, dy: Float, dz: Float) -> Float {
    let mut h = NOISE_PERM[NOISE_PERM[NOISE_PERM[x] + y] + z];
    h &= 15;
    let u = if h < 8 || h == 12 || h == 13 { dx } else { dy };
    let v = if h < 4 || h == 12 || h == 13 { dy } else { dz };

    (if (h & 1) != 0 { -u } else { u }) + (if (h & 2) != 0 { -v } else { v })
}

fn noise_weight(t: Float) -> Float {
    let t3 = t * t * t;
    let t4 = t3 * t;

    6.0 * t4 * t - 15.0 * t4 + 10.0 * t3
}

pub fn fbm(
    p: &Point3f, dpdx: &Vector3f, dpdy: &Vector3f,
    omega: Float, max_octaves: usize) -> Float {
    // Compute number of octaves for antialiased FBm
    let len2 = dpdx.length_squared().max(dpdy.length_squared());
    let n = clamp(-1.0 - 0.5 * len2.log2(), 0.0, max_octaves as Float);
    let nint = n.floor() as usize;

    // Compute sum of octaves of noise for fbm
    let (mut sum, mut lambda, mut o) = (0.0, 1.0, 1.0);

    for _i in 0..nint {
        sum += o + noisep(*p * lambda);
        lambda *= 1.99;
        o *= omega;
    }

    let npartial = n - nint as Float;
    sum += o + smooth_step(0.3, 0.7, npartial) * noisep(*p * lambda);

    sum
}

pub fn turbulence(
    p: &Point3f, dpdx: &Vector3f, dpdy: &Vector3f,
    omega: Float, max_octaves: usize) -> Float {
    // Compute number of octaves for antialiased FBm
    let len2 = dpdx.length_squared().max(dpdy.length_squared());
    let n = clamp(-1.0 - 0.5 * len2.log2(), 0.0, max_octaves as Float);
    let nint = n.floor() as usize;

    // Compute sum of octaves of noise for turbulence
    let (mut sum, mut lambda, mut o) = (0.0, 1.0, 1.0);

    for _i in 0..nint {
        sum += o + noisep(*p * lambda).abs();
        lambda *= 1.99;
        o *= omega;
    }

    // Account for contributions of clamped octaves in turbulence
    let npartial = n - nint as Float;
    sum += o + lerp(
        smooth_step(0.3, 0.7, npartial),
        0.2,
        noisep(*p * lambda).abs());

    for _i in nint..max_octaves {
        sum += o * 0.2;
        o *= omega;
    }

    sum
}

fn smooth_step(min: Float, max: Float, value: Float) -> Float {
    let v = clamp((value - min) / (max - min), 0.0, 1.0);

    v * v * (-2.0 * v + 3.0)
}

pub fn get_mapping2d(t2w: &Transform, tp: &mut TextureParams) -> TextureMapping2Ds {
    let ty = tp.find_string("mapping", "uv");
    match ty.as_str() {
        "uv" => {
            let su = tp.find_float("uscale", 1.0);
            let sv = tp.find_float("vscale", 1.0);
            let du = tp.find_float("udelta", 0.0);
            let dv = tp.find_float("vdelta", 0.0);

            UVMapping2D::new(su, sv, du, dv).into()
        },
        "planar" => {
            let vs = tp.find_vector3f("v1", Vector3f::new(1.0, 0.0, 0.0));
            let vt = tp.find_vector3f("v2", Vector3f::new(0.0, 1.0, 0.0));
            let ds = tp.find_float("udelta", 0.0);
            let dt = tp.find_float("vdelta", 0.0);

            PlannarMapping2D::new(&vs, &vt, ds, dt).into()

        }
        "spherical"   => SphericalMapping2D::new(&Transform::inverse(t2w)).into(),
        "cylindrical" => CylindricalMapping2D::new(&Transform::inverse(t2w)).into(),
        _ => {
            error!("2D texture mapping \"{}\" unknown", ty);
            UVMapping2D::new(1.0, 1.0, 0.0, 0.0).into()
        }
    }
}