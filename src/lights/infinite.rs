use crate::core::mipmap::MIPMap;
use crate::core::spectrum::{RGBSpectrum, Spectrum};
use crate::core::geometry::point::{Point2i, Point3f, Point2f};
use crate::core::pbrt::{Float, PI};
use crate::core::sampling::Distribution2D;
use crate::core::medium::MediumInterface;
use crate::core::transform::Transform;
use crate::core::light::LightFlags;
use crate::init_light_data;
use crate::mipmap;
use rayon::prelude::*;
use crate::core::imageio::read_image;

pub struct InfiniteAreaLight {
    map              : MIPMap<RGBSpectrum>,
    world_centre     : Point3f,
    world_radius     : Float,
    distribution     : Distribution2D,
    // Light Data
    flags            : u8,
    nsamples         : usize,
    medium_interface : MediumInterface,
    light_to_world   : Transform,
    world_to_light   : Transform
}

impl InfiniteAreaLight {
    pub fn new(l2w: &Transform, power: &Spectrum, nsamples: usize, map: &str) -> Self {
        macro_rules! no_texels {
            () => {
                (vec![RGBSpectrum::from(*power); 1], Point2i::new(1, 1))
            }
        }

        // Read data from texmap and initialize map
        let rgb = RGBSpectrum::from(*power);
        let (s, res) = if !map.is_empty() {
            match read_image(map) {
                Ok((mut s, r)) => {
                    for i in 0..r.x * r.y {
                        s[i as usize] *= rgb;
                    }

                    (s, r)
                },
                _ => no_texels!()
            }
        } else {
            no_texels!()
        };



        let map = mipmap!(&res, s);

        // TODO: Initialize distribution
        let width = 2 * map.width();
        let height = 2 * map.height();
        let mut img = vec![0.0; width * height];
        let fwidth = 0.5 / std::cmp::min(width, height) as Float;

        img
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, val)| {
                let v = i / width;
                let u = i % width;
                let vp = (v as Float + 0.5) / height as Float;
                let sin_theta = (PI * (v as Float + 0.5) / height as Float).sin();
                let up = (u as Float + 0.5) / width as Float;
                *val = map.lookup(&Point2f::new(up, vp), fwidth).y();
                *val *= sin_theta;
            });

        let distribution = Distribution2D::new(&img, width, height);

        let il = InfiniteAreaLight{
            map,
            nsamples,
            distribution,
            world_radius    : 0.0,
            world_centre    : Default::default(),
            flags           : Default::default(),
            medium_interface: Default::default(),
            light_to_world  : Default::default(),
            world_to_light  : Default::default()
        };

        // Initalize Light data
        let flags = LightFlags::DeltaPosition as u8;
        init_light_data!(il, flags, nsamples, MediumInterface::default(), l2w);

        il
    }
}