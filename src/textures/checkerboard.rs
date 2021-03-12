use crate::core::texture::{TextureMapping2Ds, Textures, SpectrumT, Texture, TextureMapping2D, TextureMapping3Ds, TextureMapping3D, TextureFloat, get_mapping2d, IdentityMapping3D, TextureSpec};
use std::sync::Arc;
use log::{error, warn};
use crate::core::interaction::SurfaceInteraction;
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::pbrt::Float;
use crate::core::paramset::TextureParams;
use crate::core::transform::Transform;
use crate::core::spectrum::Spectrum;

pub enum AAMethod { None, ClosedForm }

pub struct Checkerboard2DTexture<T: SpectrumT<T>> {
    mapping : TextureMapping2Ds,
    tex1    : Arc<Textures<T, T>>,
    tex2    : Arc<Textures<T, T>>,
    method  : AAMethod
}

impl<T: SpectrumT<T>> Checkerboard2DTexture<T> {
    pub fn new(mapping: TextureMapping2Ds, tex1: Arc<Textures<T, T>>,
               tex2: Arc<Textures<T, T>>, method: AAMethod) -> Self {
        Self { mapping, tex1, tex2, method }
    }
}

impl<T: SpectrumT<T>> Texture<T> for Checkerboard2DTexture<T> {
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dstdx = Vector2f::default();
        let mut dstdy = Vector2f::default();
        let st = self.mapping.map(s, &mut dstdx, &mut dstdy);

        match self.method {
            AAMethod::None => {
                if (st.x.floor() as isize + st.y.floor() as isize) % 2 == 0 {
                    self.tex1.evaluate(s)
                } else {
                    self.tex2.evaluate(s)
                }
            },
            _ => {
                // Compute closed-form box-filtered Checkerboard2DTexture value
                let ds = dstdx.x.abs().max(dstdy.x.abs());
                let dt = dstdx.y.abs().max(dstdy.y.abs());
                let (s0, s1) = (st[0] - ds, st[0] + ds);
                let (t0, t1) = (st[1] - dt, st[1] + dt);

                if s0.floor() == s1.floor() &&
                   t0.floor() == t1.floor() {
                    // Point sample Checkerboard2Dtexture
                    return if (st.x.floor() as isize + st.y.floor() as isize) % 2 == 0 {
                        self.tex1.evaluate(s)
                    } else {
                        self.tex2.evaluate(s)
                    }
                }

                // Apply box filter to checkerboard region
                let bump = |x: Float| -> Float {
                    (x / 2.0).floor() + 2.0 * (x / 2.0 - (x / 2.0).floor() - 0.5).max(0.0)
                };

                let sint = (bump(s1) - bump(s0)) / (2.0 * ds);
                let tint = (bump(t1) - bump(t0)) / (2.0 * dt);
                let mut area2 = sint * tint - 2.0 * sint * tint;

                if ds > 1.0 || dt > 1.0 { area2 = 0.5 }

                self.tex1.evaluate(s) * (1.0 - area2) + self.tex2.evaluate(s) * area2

            }
        }
    }
}

pub struct Checkerboard3DTexture<T: SpectrumT<T>> {
    mapping: TextureMapping3Ds,
    tex1: Arc<Textures<T, T>>,
    tex2: Arc<Textures<T, T>>,
}

impl<T: SpectrumT<T>> Checkerboard3DTexture<T> {
    pub fn new(mapping: TextureMapping3Ds, tex1: Arc<Textures<T, T>>,
               tex2: Arc<Textures<T, T>>, ) -> Self {
        Self { mapping, tex1, tex2 }
    }
}

impl<T: SpectrumT<T>> Texture<T> for Checkerboard3DTexture<T> {
    fn evaluate(&self, s: &SurfaceInteraction) -> T {
        let mut dstdx = Vector3f::default();
        let mut dstdy = Vector3f::default();
        let p = self.mapping.map(s, &mut dstdx, &mut dstdy);

        if (p.x.floor() as isize + p.y.floor() as isize + p.z.floor() as isize) % 2 == 0 {
            self.tex1.evaluate(s)
        } else {
            self.tex2.evaluate(s)
        }
    }
}

pub fn create_checkerboard_float(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let dim = tp.find_int("mapping", 2);

    if dim != 2 && dim != 3 {
        error!("{} dimensional checkerboard texture not supported", dim);
        return None;
    }

    let tex1 = tp.get_floattexture("tex1", 1.0);
    let tex2 = tp.get_floattexture("tex2", 0.0);

    match dim {
        2 => {
            let map = get_mapping2d(t2w, tp);
            let aa = tp.find_string("aamode", "none");

            let aam = match aa.as_str() {
                "none"       => AAMethod::None,
                "closedform" => AAMethod::ClosedForm,
                _ => {
                    warn!(
                        "Antialiasing mode \"{}\" not understood by \
                        Checkerboard2DTexture; using \"closedform\"",
                        aa
                    );
                    AAMethod::ClosedForm
                }
            };

            Some(Arc::new(Checkerboard2DTexture::new(map, tex1, tex2, aam).into()))
        },
        _ => {
            let map: TextureMapping3Ds= IdentityMapping3D::new(t2w).into();

            Some(Arc::new(Checkerboard3DTexture::new(map, tex1, tex2).into()))
        }
    }
}

pub fn create_checkerboard_spectrum(t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let dim = tp.find_int("mapping", 2);

    if dim != 2 && dim != 3 {
        error!("{} dimensional checkerboard texture not supported", dim);
        return None;
    }

    let tex1 = tp.get_spectrumtexture("tex1", Spectrum::new(1.0));
    let tex2 = tp.get_spectrumtexture("tex2", Spectrum::new(0.0));

    match dim {
        2 => {
            let map = get_mapping2d(t2w, tp);
            let aa = tp.find_string("aamode", "none");

            let aam = match aa.as_str() {
                "none"       => AAMethod::None,
                "closedform" => AAMethod::ClosedForm,
                _ => {
                    warn!(
                        "Antialiasing mode \"{}\" not understood by \
                        Checkerboard2DTexture; using \"closedform\"",
                        aa
                    );
                    AAMethod::ClosedForm
                }
            };

            Some(Arc::new(Checkerboard2DTexture::new(map, tex1, tex2, aam).into()))
        },
        _ => {
            let map: TextureMapping3Ds= IdentityMapping3D::new(t2w).into();

            Some(Arc::new(Checkerboard3DTexture::new(map, tex1, tex2).into()))
        }
    }
}

