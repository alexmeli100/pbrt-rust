use anyhow::{Result, Context, anyhow};
use crate::core::geometry::point::Point2i;
use crate::core::spectrum::{RGBSpectrum, SpectrumType};
use byteorder::{LittleEndian, ReadBytesExt, BigEndian, ByteOrder, WriteBytesExt};
use std::path::Path;
use image::GenericImageView;
use rayon::prelude::*;
use crate::core::pbrt::{Float, clamp, gamma_correct};
use log::info;
use exr::prelude::*;
use std::io::{BufRead, BufReader, ErrorKind};
use std::fs::File;
use crate::core::geometry::bounds::Bounds2i;
use std::io::Write;
use crate::core::geometry::vector::Vector2i;

pub fn read_image<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    let ext = name.as_ref()
        .extension()
        .with_context(|| format!("Failed to get filename extension \"{}\"", name.as_ref().display()))?;

    if ext == "png" || ext == "PNG" {
        read_image_png(name)
    } else if ext == "tga" || ext == "TGA" {
        read_image_tga(name)
    } else if ext == "exr" || ext == "EXR" {
        read_image_exr(name)
    } else if ext == "pfm" {
        read_image_pfm(name)
    } else {
        let err = anyhow!(
        "Unable to load image stored in format \"{:?}\" for filename \"{}\"",
        ext, name.as_ref().display());
        Err(err)
    }

}

pub fn write_image<P: AsRef<Path>>(
    name: P, rgb: &[f32], bounds: &Bounds2i,
    total_res: &Point2i) -> Result<()> {
    let ext = name.as_ref()
        .extension()
        .with_context(|| format!("Failed to get filename extension \"{}\"", name.as_ref().display()))?;

    if ext == "png" || ext == "tga" {
        write_image_png_tga(name, rgb, bounds, total_res)
    } else if ext == "exr" {
        write_image_exr(name, rgb, bounds, total_res)
    } else if ext == "pfm" {
        write_image_pfm(name, rgb, bounds.diagonal())
    }
    else {
        let err = anyhow!("Unsupported file format \"{:?}\"", ext);
        Err(err)
    }
}

struct ExrImage {
    pixels: Vec<(Float, Float, Float)>,
    width : usize,
    height: usize
}

fn read_image_exr<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    let image = read_first_rgba_layer_from_file(
        name,
        |info| {
            let width = info.resolution.width();
            let height = info.resolution.height();
            let pixels = vec![(0.0, 0.0, 0.0); width * height];

            ExrImage {pixels, width, height}
        },
        |pixs, pos, pixel| {
            let r = pixel.red.to_f32();
            let g = pixel.green.to_f32();
            let b = pixel.blue.to_f32();

            pixs.pixels[pos.y() * pixs.width + pos.x()] = (r, g, b);
        }
    )?;

    let storage = image.layer_data.channel_data.storage;
    let pixels = storage.pixels
        .par_iter()
        .map(|(r, g, b)| RGBSpectrum::from_rgb([*r, *g, *b], SpectrumType::Reflectance))
        .collect::<Vec<RGBSpectrum>>();
    let res = Point2i::new(storage.width as isize, storage.height as isize);

    Ok((pixels, res))
}

type Pixel = (f32, f32, f32);

pub fn write_image_exr<P: AsRef<Path>>(
    name: P, rgb: &[Float], bounds: &Bounds2i,
    _total_res: &Point2i) -> Result<()> {
    let res = bounds.diagonal();
    let data = rgb.chunks(3).map(|p| (p[0], p[1], p[2])).collect::<Vec<Pixel>>();

    write_rgb_f32_file(
        name.as_ref(),
        (res.x as usize, res.y as usize),
        |x, y| {
            data[y * res.x as usize + x]
        }
    )
        .with_context(||
            format!("Error writing exr image \"{}\"", name.as_ref().display()))
}

fn is_whitespace(c: char) -> bool {
    c == ' ' || c == '\n' || c == '\t'
}

fn read_word<R: BufRead>(f: &mut R) -> Result<String> {
    let mut s = String::new();

    let mut buf = [0_u8; 1];

    loop {
        match f.read_exact(&mut buf) {
            Ok(_) => {
                let c = buf[0] as char;
                if is_whitespace(c) { break; }
                s.push(c);
            }
            Err(e) if e.kind() == ErrorKind::UnexpectedEof =>
                break,
            _ =>  return Err(anyhow!("error reading word"))
        }

    }

    Ok(s)
}

fn read_image_pfm<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    macro_rules! fail {
        () => {
            let err = anyhow!(
            "Error reading PFM file \"{}\"",
            name.as_ref().display());

            return Err(err);
        }
    }

    let f = File::open(name.as_ref())?;
    let mut r = BufReader::new(f);

    // read either "Pf" or "PF"
    let mut word = if let Ok(w) = read_word(&mut r) {
        w
    } else {
        fail!();
    };

    let nchannels: usize;

    if word == "Pf" {
        nchannels = 1;
    } else if word == "PF" {
        nchannels = 3;
    } else {
        fail!();
    }

    // read the rest if the header
    // read width
    word = if let Ok(w) = read_word(&mut r) {
        w
    } else {
        fail!();
    };

    let width = if let Ok(w) = word.parse::<usize>() {
        w
    } else {
        fail!();
    };

    // read height
    word = if let Ok(w) = read_word(&mut r) {
        w
    } else {
        fail!();
    };

    let height = if let Ok(h) = word.parse::<usize>() {
        h
    } else {
        fail!();
    };

    // read scale
    word = if let Ok(w) = read_word(&mut r) {
        w
    } else {
        fail!();
    };

    let scale = if let Ok(s) = word.parse::<Float>() {
        s
    } else {
        fail!();
    };

    let file_little_endian = scale < 0.0;
    let host_little_endian = true;

    // Read the data
    let n = nchannels * width * height;
    let mut data = vec![0.0; n];

    for y in (0..height).rev() {
        let start = y * nchannels * width;
        let end = start + nchannels * width;

        let res = if file_little_endian {
            r.read_f32_into::<LittleEndian>(&mut data[start..end])
        } else {
            r.read_f32_into::<BigEndian>(&mut data[start..end])
        };

        if res.is_err() {
            fail!();
        }
    }

    // apply endian conversion and scale if appropriate
    if host_little_endian ^ file_little_endian {
        for f in data.iter_mut() {
            *f = LittleEndian::read_f32(&f.to_le_bytes());
        }
    }

    if scale.abs() != 1.0 {
        for f in data.iter_mut() {
            *f *= scale.abs();
        }
    }

    let pixels = if nchannels == 1 {
        data.iter().map(|n| RGBSpectrum::new(*n)).collect()
    } else {
        data.chunks(3)
            .map(|rgb|
                RGBSpectrum::from_rgb([rgb[0], rgb[1], rgb[2]], SpectrumType::Reflectance))
            .collect()
    };
    let res = Point2i::new(width as isize, height as isize);

    Ok((pixels, res))

}

fn write_image_pfm<P: AsRef<Path>>(name: P, rgb: &[f32], res: Vector2i) -> Result<()> {
    let width = res.x as usize;
    let height = res.y as usize;
    let mut f = File::create(name.as_ref())
        .with_context(|| format!("Unable to open output PFM file \"{}\"", name.as_ref().display()))?;

    let mut buf = Vec::new();

    // only write 3 channel PFMs here
    buf.extend_from_slice("PF".as_bytes());
    buf.push(b'\n');

    // write the width and height which must be positive
    buf.extend_from_slice(format!("{} {}\n", width, height).as_bytes());

    let host_little_endian = true;

    // write the scale, which encodes endianness
    let scale = if host_little_endian { -1.0 } else { 1.0 };
    buf.extend_from_slice(format!("{}\n", scale).as_bytes());

    let mut scanline = vec![0.0_f32; 3 * width];
    // write the data from bottom left to upper right as specified by
    // http://netpbm.sourceforge.net/doc/pfm.html
    // The raster is a sequence of pixels, packed one after another, with no
    // delimiters of any kind. They are grouped by row, with the pixels in each
    // row ordered left to right and the rows ordered bottom to top.

    for y in (0..height).rev() {
        for x in 0..(3 * width) {
            scanline[x] = rgb[y * width * 3 + x];
        }

        for f in &scanline {
            buf.write_f32::<LittleEndian>(*f)?;
        }
    }

    f.write_all(&buf).with_context(||
        format!("Error writing PFM file \"{}\"", name.as_ref().display()))
}

fn read_image_png<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    read_image_png_tga(name, "PNG")
}

fn read_image_tga<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    read_image_png_tga(name, "TGA")
}

fn read_image_png_tga<P: AsRef<Path>>(name: P, ext: &str) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    let buf = image::open(name.as_ref())?;
    let (width, height) = buf.dimensions();
    let rgb = buf.to_rgb8().into_raw();
    let res = Point2i::new(width as isize, height as isize);

    let pixels = rgb
        .par_chunks(3)
        .map(|p| {
            let r = p[0] as Float / 255.0;
            let g = p[1] as Float / 255.0;
            let b = p[2] as Float / 255.0;

            RGBSpectrum::from_rgb([r, g, b], SpectrumType::Reflectance)
        }).collect::<Vec<RGBSpectrum>>();

    info!("Read {} image {} ({} X {})", ext, name.as_ref().display(), width, height);

    Ok((pixels, res))
}

fn write_image_png_tga<P: AsRef<Path>>(
    name: P, rgb: &[f32], bounds: &Bounds2i,
    _total_res: &Point2i) -> Result<()> {


    let res = bounds.diagonal();
    let buf = rgb.iter().map(|v| clamp(
        255.0 * gamma_correct(*v) + 0.5, 0.0, 255.0) as u8
    ).collect::<Vec<_>>();

    image::save_buffer(
        name.as_ref(),
        &buf,
        res.x as u32,
        res.y as u32,
        image::ColorType::Rgb8
    )
        .with_context(|| format!("Error writing image \"{}\"", name.as_ref().display()))
}

