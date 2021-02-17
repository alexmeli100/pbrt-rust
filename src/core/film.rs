use crate::core::geometry::point::{Point2i, Point2f};
use crate::core::spectrum::{Spectrum, xyz_to_rgb};
use crate::core::pbrt::{Float, Options, clamp, INFINITY};
use crate::core::filter::{Filters, Filter};
use crate::core::geometry::bounds::{Bounds2i, Bounds2f};
use crate::core::parallel::AtomicFloat;
use std::sync::RwLock;
use log::{info, error, warn};
use anyhow::Result;
use typed_arena::Arena;
use crate::core::geometry::vector::Vector2f;
use crate::core::paramset::ParamSet;
use crate::core::imageio::write_image;
use std::path::{PathBuf};

const FILTER_TABLE_WIDTH: usize = 16;

#[derive(Default)]
struct FilmTilePixel {
    contrib_sum         : Spectrum,
    filter_weight_sum   : Float
}

struct Pixel {
    xyz                 : [Float; 3],
    filter_weight_sum   : Float,
    splat_xyz           : [AtomicFloat; 3],
    _pad                : Float
}

pub struct Film {
    pub full_resolution     : Point2i,
    pub diagonal            : Float,
    pub filter              : Filters,
    pub filename            : PathBuf,
    pub cropped_pixel_bounds: Bounds2i,
    pixels                  : RwLock<Vec<Pixel>>,
    filter_table            : [Float; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH],
    scale                   : Float,
    max_sample_luminance    : Float
}

impl Film {
    pub fn new(resolution: &Point2i, crop_window: &Bounds2f, filt: Filters, diagonal: Float,
               filename: PathBuf, scale: Float, max_sample_luminance: Float) -> Self {
        let crop_pixel_bounds = Bounds2i::from_points(
            &Point2i::new(
                (resolution.x as Float * crop_window.p_min.x).ceil() as isize,
                (resolution.y as Float * crop_window.p_min.y).ceil() as isize),
            &Point2i::new(
                (resolution.x as Float * crop_window.p_max.x).ceil() as isize,
                (resolution.y as Float * crop_window.p_max.y).ceil() as isize
            )
        );

        info!("Created film with full resolution {}\
            . Crop window of {} -> croppedPixelBounds {}",
            resolution, crop_window, crop_pixel_bounds);

        // Allocate film image storage
        let pixels = Vec::with_capacity(crop_pixel_bounds.area() as usize);
        // TODO: filmPixelMemory

        // Precompute filter weight table
        let mut offset = 0;
        let mut filter_table = [0.0; FILTER_TABLE_WIDTH * FILTER_TABLE_WIDTH];

        for y in 0..FILTER_TABLE_WIDTH {
            for x in 0..FILTER_TABLE_WIDTH {
                let p = Point2f::new(
                    (x as Float + 0.5) * filt.radius().x / FILTER_TABLE_WIDTH as Float,
                    (y as Float + 0.5) * filt.radius().y / FILTER_TABLE_WIDTH as Float
                );
                filter_table[offset] = filt.evaluate(&p);
                offset += 1;
            }
        }

        Self {
            full_resolution: *resolution,
            diagonal: diagonal * 0.001,
            filter: filt,
            filename: filename.to_owned(),
            scale,
            max_sample_luminance,
            cropped_pixel_bounds: crop_pixel_bounds,
            pixels: RwLock::new(pixels),
            filter_table
        }
    }

    pub fn get_sample_bounds(&self) -> Bounds2i {
        let p1 = (Point2f::from(self.cropped_pixel_bounds.p_min) +
                  Vector2f::new(0.5, 0.5) - self.filter.radius()).floor();
        let p2 = (Point2f::from(self.cropped_pixel_bounds.p_max) -
                  Vector2f::new(0.5, 0.5) + self.filter.radius()).ceil();

        Bounds2i::from_points(&Point2i::from(p1), &Point2i::from(p2))

    }

    pub fn get_physical_extent(&self) -> Bounds2f {
        let aspect = self.full_resolution.y as Float / self.full_resolution.x as Float;
        let x = (self.diagonal * self.diagonal / (1.0 + aspect * aspect)).sqrt();
        let y = aspect * x;

        Bounds2f::new(
            &Point2f::new(-x / 2.0, -y / 2.0),
            &Point2f::new(x / 2.0, y / 2.0)
        )
    }

    pub fn get_film_tile(&self, sample_bounds: &Bounds2i) -> FilmTile {
        // Bound image pixels that samples in sampleBounds contribute to
        let half_pixel = Vector2f::new(0.5, 0.5);
        let float_bounds = Bounds2f {
            p_min: Point2f::from(sample_bounds.p_min),
            p_max: Point2f::from(sample_bounds.p_max)
        };
        let p0f = (float_bounds.p_min - half_pixel - self.filter.radius()).ceil();
        let p1f = (float_bounds.p_max - half_pixel + self.filter.radius()).floor();
        let p0 = Point2i::from(p0f);
        let p1 = Point2i::from(p1f) + Point2i::new(1, 1);
        let tile_bounds = Bounds2i::from_points(&p0, &p1).intersect(&self.cropped_pixel_bounds);

        FilmTile::new(&tile_bounds, &self.filter.radius(),
                      &self.filter_table, FILTER_TABLE_WIDTH, self.max_sample_luminance)
    }

    pub fn merge_film_tile(&self, tile: &mut FilmTile) {
        // TODO: ProfilePhase
        let mut pixels = self.pixels.write().unwrap();
        info!("Merging film tile {}", tile.pixel_bounds);

        for p in &tile.get_pixel_bounds() {
            // Merge pixel into Film::pixels
            let tile_pixel = tile.get_pixel(&p);
            let offset = self.get_pixel(&p);
            let merge_pixel = &mut pixels[offset];

            let xyz = tile_pixel.contrib_sum.to_xyz();

            for i in 0..3 {
                merge_pixel.xyz[i] += xyz[i];
            }

            merge_pixel.filter_weight_sum += tile_pixel.filter_weight_sum;
        }
    }

    fn get_pixel(&self, p: &Point2i) -> usize {
        assert!(self.cropped_pixel_bounds.inside_exclusive(p));

        let width = self.cropped_pixel_bounds.p_max.x - self.cropped_pixel_bounds.p_min.x;
        let offset = (p.x - self.cropped_pixel_bounds.p_min.x) + (p.y - self.cropped_pixel_bounds.p_min.y) * width;

        offset as usize
    }

    pub fn set_image(&self, img: &[Spectrum]) {
        let npixels = self.cropped_pixel_bounds.area() as usize;
        let mut pixels = self.pixels.write().unwrap();

        for i in 0..npixels {
            let p = &mut pixels[i];
            p.xyz = img[i].to_xyz();
            p.filter_weight_sum = 1.0;
            p.splat_xyz[0] = AtomicFloat::new(0.0);
            p.splat_xyz[1] = AtomicFloat::new(0.0);
            p.splat_xyz[2] = AtomicFloat::new(0.0);
        }
    }

    pub fn add_splat(&self, p: &Point2f, mut v: Spectrum) {
        // TODO: ProfilePhase
        if v.has_nans() {
            error!("Ignoring splatted spectrum with NaN values at ({}, {})", p.x, p.y);
            return;
        } else if v.y() < 0.0 {
            error!("Ignoring splatted spectrum with negative luminance {} at ({}, {})", v.y(), p.x, p.y);
            return
        } else if v.y().is_infinite() {
            error!("Ignoring slatted spectrum with infinite luminance at ({}, {})", p.x, p.y);
            return;
        }

        let pi = Point2i::from(p.floor());

        if !self.cropped_pixel_bounds.inside_exclusive(&pi) { return; }
        if v.y() > self.max_sample_luminance {
            v *= self.max_sample_luminance / v.y();
        }

        let mut pixels = self.pixels.write().unwrap();

        let xyz = v.to_xyz();
        let offset = self.get_pixel(&pi);
        let pixel = &mut pixels[offset];

        for i in 0..3 {
            pixel.splat_xyz[i].add(xyz[i]);
        }
    }

    pub fn write_image(&self, splat_scale: Float) -> Result<()> {
        // Convert image to RGB and compute final pixel values
        info!("Converting image to RGB and computing final weighted pixel values");
        let mut rgb = vec![0.0; (3 * self.cropped_pixel_bounds.area()) as usize];
        let mut offset = 0;

        for p in &self.cropped_pixel_bounds {
            // Convert pixel XYZ color to RGB
            let poffset = self.get_pixel(&p);
            let pixel = &self.pixels.read().unwrap()[poffset];
            let xyz = xyz_to_rgb(pixel.xyz);
            rgb[3 * offset] = xyz[0];
            rgb[3 * offset + 1] = xyz[1];
            rgb[3 * offset + 2] = xyz[2];

            // Normalize pixel with weight sum
            let filter_weight_sum = pixel.filter_weight_sum;

            if filter_weight_sum != 0.0 {
                let invwt = 1.0 / filter_weight_sum;
                rgb[3 * offset] = 0.0_f32.max(rgb[3 * offset] * invwt);
                rgb[3 * offset + 1] = 0.0_f32.max(rgb[3 * offset + 1] * invwt);
                rgb[3 * offset + 2] = 0.0_f32.max(rgb[3 * offset + 2] * invwt);
            }

            // splate value at pixel
            let splat_xyz: [Float; 3] = [
                pixel.splat_xyz[0].clone().into(),
                pixel.splat_xyz[1].clone().into(),
                pixel.splat_xyz[2].clone().into()
            ];
            let splat_rgb = xyz_to_rgb(splat_xyz);
            rgb[3 * offset] += splat_scale * splat_rgb[0];
            rgb[3 * offset + 1] += splat_scale * splat_rgb[1];
            rgb[3 * offset + 2] += splat_scale * splat_rgb[2];

            // Scale pixel value by scale
            rgb[3 * offset] *= self.scale;
            rgb[3 * offset + 1] *= self.scale;
            rgb[3 * offset + 2] *= self.scale;
            offset += 1;
        }

        info!("Writing image {} with bounds {}", self.filename.display(), self.cropped_pixel_bounds);

        // TODO: WriteImage
        write_image(&self.filename, &rgb, &self.cropped_pixel_bounds, &self.full_resolution)
    }
}

pub struct FilmTile<'a> {
    pixel_bounds        : Bounds2i,
    filter_radius       : Vector2f,
    inv_filter_radius   : Vector2f,
    filter_table        : &'a[Float],
    filter_table_size   : usize,
    pixels              : Vec<FilmTilePixel>,
    max_sample_luminance: Float
}

impl<'a> FilmTile<'a> {
    pub fn new(pixel_bounds: &Bounds2i, filter_radius: &Vector2f, filter_table: &'a[Float],
               filter_table_size: usize, max_sample_luminance: Float) -> Self {
        Self {
            filter_table,
            filter_table_size,
            max_sample_luminance,
            pixel_bounds: *pixel_bounds,
            filter_radius: *filter_radius,
            inv_filter_radius: Vector2f::new(1.0 / filter_radius.x, 1.0 / filter_radius.y),
            pixels: Vec::with_capacity(std::cmp::max(0, pixel_bounds.area() as usize))
        }
    }

    pub fn add_sample(&mut self, pfilm: &Point2f, mut L: Spectrum, sample_weight: Float) {
        // TODO: ProfilePhase
        if L.y() > self.max_sample_luminance { L *= self.max_sample_luminance / L.y(); }

        // Compute sample's raster bounds;
        let pfilm_discrete = *pfilm - Vector2f::new(0.5, 0.5);
        let p0f = (pfilm_discrete - self.filter_radius).ceil();
        let p1f = (pfilm_discrete + self.filter_radius).floor();
        let mut p0 = Point2i::new(p0f.x as isize, p0f.y as isize);
        let mut p1 = Point2i::new(p1f.x as isize, p1f.y as isize) + Point2i::new(1, 1);
        p0 = p0.max(&self.pixel_bounds.p_min);
        p1 = p1.min(&self.pixel_bounds.p_max);

        // Loop over filter support and add sample to pixel arrays;
        let arena = Arena::new();
        let ifx = arena.alloc(vec![0; (p1.x - p0.x) as usize]);
        let ify = arena.alloc(vec![0, (p1.y - p0.y) as usize]);

        for x in p0.x..p1.x {
            let fx = ((x as Float - pfilm_discrete.x) * self.inv_filter_radius.x * self.filter_table_size as Float).abs();
            ifx[(x - p0.x) as usize] = std::cmp::min(fx.floor() as usize, self.filter_table_size - 1);
        }

        for y in p0.y..p1.y {
            let fy = ((y as Float - pfilm_discrete.y) * self.inv_filter_radius.y * self.filter_table_size as Float).abs();
            ify[(y - p0.y) as usize] = std::cmp::min(fy.floor() as usize, self.filter_table_size - 1);
        }

        for y in p0.y..p1.y {
            for x in p0.x..p1.x {
                // Evaluate filter value at (x, y) pixel
                let offset = ify[(y - p0.y) as usize] * self.filter_table_size + ifx[(x - p0.x) as usize];
                let filter_weight = self.filter_table[offset];

                // Update pixel values with filtered sample contribution
                let pixel = self.get_pixel(&Point2i::new(x, y));
                pixel.contrib_sum += L * sample_weight * filter_weight;
                pixel.filter_weight_sum += filter_weight;
            }
        }
    }

    fn get_pixel(&mut self, p: &Point2i) -> &mut FilmTilePixel {
        assert!(self.pixel_bounds.inside_exclusive(p));

        let width = self.pixel_bounds.p_max.x - self.pixel_bounds.p_min.x;
        let offset = (p.x - self.pixel_bounds.p_min.x) + (p.y - self.pixel_bounds.p_min.y) * width;

        &mut self.pixels[offset as usize]
    }

    fn get_pixel_bounds(&self) -> Bounds2i {
        self.pixel_bounds
    }
}

pub fn create_film(params: &ParamSet, filter: Filters, opts: &Options) -> Film {
    let filename = if !opts.image_file.as_os_str().is_empty() {
        let params_filename = params.find_one_string("filename", "".to_owned());

        if !params_filename.is_empty() {
            warn!("Output filename supplied on command line. \"{}\" is overriding\
                  filename provided in scene description file, \"{}\"",
                  opts.image_file.display(), params_filename);
        }

        opts.image_file.clone()
    } else {
        let f = params.find_one_string("filename", "pbrt.exr".to_owned());

        PathBuf::from(f)
    };

    let mut xres = params.find_one_int("xresolution", 1280);
    let mut yres = params.find_one_int("yresolution", 720);

    if opts.quick_render {
        xres = std::cmp::max(1, xres / 4);
        yres = std::cmp::max(1, yres / 4);
    }

    let mut crop = Bounds2f::default();
    let mut cwi = 0;
    let cr_some = params.find_float("cropwindow", &mut cwi);

    if cr_some.is_some() && cwi == 4 {
        let cr = cr_some.unwrap();
        crop.p_min.x = clamp(cr[0].min(cr[1]), 0.0, 1.0);
        crop.p_max.x = clamp(cr[0].max(cr[1]), 0.0, 1.0);
        crop.p_min.y = clamp(cr[2].min(cr[3]), 0.0, 1.0);
        crop.p_max.y = clamp(cr[2].max(cr[3]), 0.0, 1.0);
    } else if cr_some.is_some() {
        error!("{} values supplised fir \"cropwindow\". Expected 4.", cwi);
    } else {
        crop = Bounds2f::new(
            &Point2f::new(
                    clamp(opts.crop_window[0][0], 0.0, 1.0),
                    clamp(opts.crop_window[1][0], 0.0, 1.0)),
            &Point2f::new(
                    clamp(opts.crop_window[0][1], 0.0, 1.0),
                    clamp(opts.crop_window[1][1], 0.0, 0.0)
            )
        );
    }

    let scale = params.find_one_float("scale", 1.0);
    let diagonal = params.find_one_float("diagonal", 35.0);
    let max_sample_luminance = params.find_one_float("maxsampleluminance", INFINITY);

    Film::new(
        &Point2i::new(xres, yres), &crop, filter,
        diagonal, filename, scale, max_sample_luminance)
}
