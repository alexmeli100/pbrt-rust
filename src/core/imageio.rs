use anyhow::Result;
use crate::core::geometry::point::Point2i;
use crate::core::spectrum::RGBSpectrum;
use combine::lib::path::Path;

pub fn read_image<P: AsRef<Path>>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    unimplemented!()
}

pub fn read_image_png<P>(name: P) -> Result<(Vec<RGBSpectrum>, Point2i)> {
    unimplemented!()
}