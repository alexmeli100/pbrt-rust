use std::cell::Cell;
use log::{warn, error};
use std::sync::Mutex;
use lazy_static::lazy_static;
use crate::core::pbrt::Float;
use std::collections::HashMap;
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::geometry::vector::{Vector2f, Vector3f};
use crate::core::geometry::normal::Normal3f;
use crate::core::spectrum::{Spectrum, SpectrumType, black_body_normalized};
use crate::core::cie::{N_CIE_SAMPLES, CIE_LAMBDA};
use crate::core::fileutil::{absolute_path, resolve_filename};
use crate::core::floatfile::read_float_file;
use std::sync::Arc;
use crate::core::texture::{TextureFloat, TextureSpec};
use crate::textures::constant::ConstantTexture;
use std::fmt::{Display, Formatter};
use std::fmt;

lazy_static! {
    static ref CACHED_SPECTRA: Mutex<HashMap<String, Spectrum>> = Mutex::new(HashMap::new());
}

macro_rules! find {
    ($x:ident, $a:ident, $t:ty) => {
        pub fn $x(&self, name: &str, nvalues: &mut usize) -> Option<Vec<$t>> {
            let found = self.$a.iter().find(|ref f| f.name == name);

            if let Some(ref e) = found {
                *nvalues = e.nvalues;
                e.looked_up.set(true);
            }

            found.map(|e| e.values.clone())
        }
    }
}

macro_rules! find_one {
    ($x:ident, $a:ident, $t:ty) => {
        pub fn $x(&self, name: &str, d: $t) -> $t {
            let found = self.$a.iter().find(|ref f| f.name == name && f.nvalues == 1);

            if let Some(ref e) = found {
                e.looked_up.set(true);
            }

            found.map(|e| e.values[0].clone()).unwrap_or(d)
        }
    }
}

macro_rules! erase {
    ($x:ident, $a:ident) => {
        pub fn $x(&mut self, n: &str) -> bool {
            let pos = self.$a.iter().position(|e| e.name == n);

            pos.map(|p| self.$a.remove(p)).is_some()
        }
    }
}

macro_rules! add {
    ($x:ident, $y:ident, $a:ident, $t:ty) => {
        pub fn $x(&mut self, name: &str, values: Vec<$t>, nvalues: usize) {
            self.$y(name);
            let p = ParamSetItem::new(name, values, nvalues);

            self.$a.push(p);
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ParamSetItem<T> {
    pub name        : String,
    pub values      : Vec<T>,
    pub nvalues     : usize,
    pub looked_up   : Cell<bool>
}

impl<T> ParamSetItem<T> {
    fn new(name: &str, v: Vec<T>, nvalues: usize)  -> Self {
        Self {
            name: name.to_string(),
            values: v,
            nvalues,
            looked_up: Cell::new(false)
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct ParamSet {
    pub bools       : Vec<ParamSetItem<bool>>,
    pub ints        : Vec<ParamSetItem<isize>>,
    pub floats      : Vec<ParamSetItem<Float>>,
    pub point2fs    : Vec<ParamSetItem<Point2f>>,
    pub point3fs    : Vec<ParamSetItem<Point3f>>,
    pub vector2fs   : Vec<ParamSetItem<Vector2f>>,
    pub vector3fs   : Vec<ParamSetItem<Vector3f>>,
    pub normals     : Vec<ParamSetItem<Normal3f>>,
    pub spectra     : Vec<ParamSetItem<Spectrum>>,
    pub strings     : Vec<ParamSetItem<String>>,
    pub textures    : Vec<ParamSetItem<String>>
}

impl ParamSet {
    erase!(erase_int, ints);
    erase!(erase_float, floats);
    erase!(erase_bool, bools);
    erase!(erase_string, strings);
    erase!(erase_point2f, point2fs);
    erase!(erase_point3f, point3fs);
    erase!(erase_vector2f, vector2fs);
    erase!(erase_vector3f, vector3fs);
    erase!(erase_normal3f, normals);
    erase!(erase_spectrum, spectra);
    erase!(erase_texture, textures);

    add!(add_int, erase_int, ints, isize);
    add!(add_float, erase_float, floats, Float);
    add!(add_bool, erase_bool, bools, bool);
    add!(add_string, erase_string, strings, String);
    add!(add_point2f, erase_point2f, point2fs, Point2f);
    add!(add_point3f, erase_point3f, point3fs, Point3f);
    add!(add_vector2f, erase_vector2f, vector2fs, Vector2f);
    add!(add_vector3f, erase_vector3f, vector3fs, Vector3f);
    add!(add_normal3f, erase_normal3f, normals, Normal3f);

    pub fn add_rgb_spectrum(&mut self, name: &str, values: Vec<Float>, nvalues: usize) {
        self.erase_spectrum(name);

        assert_eq!(nvalues % 3, 0);
        let n = nvalues / 3;
        let mut s = Vec::with_capacity(n);

        for i in 0..n {
            let o = i * 3;
            s.push(Spectrum::from_rgb([values[o], values[o + 1], values[o + 2]], SpectrumType::Reflectance))
        }

        let psi = ParamSetItem::new(name, s, n);
        self.spectra.push(psi);
    }

    pub fn add_xyz_spectrum(&mut self, name: &str, values: Vec<Float>, nvalues: usize) {
        self.erase_spectrum(name);

        assert_eq!(nvalues % 3, 0);
        let n = nvalues / 3;
        let mut s = Vec::with_capacity(n);

        for i in 0..n {
            let o = i * 3;
            s.push(Spectrum::from_xyz([values[o], values[o + 1], values[o + 2]], SpectrumType::Reflectance))
        }

        let psi = ParamSetItem::new(name, s, n);
        self.spectra.push(psi);
    }

    pub fn add_blackbody_spectrum(&mut self, name: &str, values: &[Float], nvalues: usize) {
        self.erase_spectrum(name);
        assert_eq!(nvalues % 2, 0);

        let n = nvalues / 2;
        let mut s = Vec::with_capacity(n);
        let mut v = vec![0.0; N_CIE_SAMPLES];

        for i in 0..n {
            black_body_normalized(&CIE_LAMBDA, N_CIE_SAMPLES, values[2 * i], &mut v);
            let res =  Spectrum::from_sampled(&CIE_LAMBDA, &v, N_CIE_SAMPLES) * values[2 * i + 1];
            s.push(res);
        }

        let item = ParamSetItem::new(name, s, n);
        self.spectra.push(item);
    }

    pub fn add_sampled_spectrum(&mut self, name: &str, values: &[Float], nvalues: usize) {
        self.erase_spectrum(name);
        assert_eq!(nvalues % 2, 0);

        let n = nvalues / 2;
        let mut wl = Vec::with_capacity(n);
        let mut v = Vec::with_capacity(n);

        for i in 0..n {
            wl.push(values[2 * i]);
            v.push(values[2 * i + 1]);
        }

        let s = vec![Spectrum::from_sampled(&wl[0..], &v[0..], n)];
        let item = ParamSetItem::new(name, s, 1);
        self.spectra.push(item)
    }

    pub fn add_sampled_spectrum_files(&mut self, name: &str, names: &[String], nvalues: usize) {
        self.erase_spectrum(name);
        let mut cached = CACHED_SPECTRA.lock().unwrap();

        let mut s = Vec::with_capacity(nvalues);

        for n in names.iter() {
            let fname = absolute_path(&resolve_filename(n));

            let entry = cached.get(&fname);

            if let Some(spec) = entry {
                s.push(*spec);
                continue;
            }

            let mut vals = Vec::new();
            let res = read_float_file(&fname, &mut vals);

            if res.is_err() {
                warn!(
                    "{}: Unable to read SPD file {}. Using black distribution",
                    res.err().unwrap(),
                    fname);
                s.push(Spectrum::new(0.0));
                continue;
            }

            if vals.len() % 2 != 0 {
                warn!("Extra value found in spectrum file \"{}\". Ignoring it.", fname)
            }

            let mut wls = Vec::with_capacity(vals.len() / 2);
            let mut v = Vec::with_capacity(vals.len() / 2);

            for j in 0..vals.len() / 2 {
                wls.push(vals[2 * j]);
                v.push(vals[ 2 * j + 1]);
            }

            let spectrum = Spectrum::from_sampled(&wls[0..], &v[0..], wls.len());
            s.push(spectrum);
            cached.insert(fname, spectrum);
        }

        let item = ParamSetItem::new(name, s, nvalues);
        self.spectra.push(item);
    }

    pub fn add_texture(&mut self, name: &str, value: &str) {
        self.erase_texture(name);

        let s = vec![value.to_owned()];
        let item = ParamSetItem::new(name, s, 1);
        self.textures.push(item);
    }

    find!(find_int, ints, isize);
    find!(find_bool, bools, bool);
    find!(find_float, floats, Float);
    find!(find_string, strings, String);
    find!(find_point2f, point2fs, Point2f);
    find!(find_point3f, point3fs, Point3f);
    find!(find_spectrum, spectra, Spectrum);
    find!(find_normal3f, normals, Normal3f);
    find!(find_vector2f, vector2fs, Vector2f);
    find!(find_vector3f, vector3fs, Vector3f);
    find_one!(find_one_int, ints, isize);
    find_one!(find_one_bool, bools, bool);
    find_one!(find_one_float, floats, Float);
    find_one!(find_texture, textures, String);
    find_one!(find_one_string, strings, String);
    find_one!(find_one_point2f, point2fs, Point2f);
    find_one!(find_one_point3f, point3fs, Point3f);
    find_one!(find_one_spectrum, spectra, Spectrum);
    find_one!(find_one_normal3f, normals, Normal3f);
    find_one!(find_one_vector2f, vector2fs, Vector2f);
    find_one!(find_one_vector3f, vector3fs, Vector3f);

    pub fn find_one_filename(&self, name: &str, d: &str) -> String {
        let filename = self.find_one_string(name, d.to_owned());

        if filename.is_empty() { return d.to_owned() }

        absolute_path(&resolve_filename(&filename))
    }

    pub fn report_unused(&self) {
        macro_rules! check_unused {
            ($a:ident) => {
                self.$a
                .iter()
                .filter(|n| !n.looked_up.get())
                .for_each(|x| warn!("Parameter \"{}\" not used", x.name));
            }
        }

        check_unused!(ints);
        check_unused!(bools);
        check_unused!(floats);
        check_unused!(point2fs);
        check_unused!(vector2fs);
        check_unused!(point2fs);
        check_unused!(vector3fs);
        check_unused!(normals);
        check_unused!(spectra);
        check_unused!(strings);
        check_unused!(textures);
    }

    pub fn clear(&mut self) {
        self.ints.clear();
        self.floats.clear();
        self.bools.clear();
        self.point2fs.clear();
        self.point3fs.clear();
        self.vector2fs.clear();
        self.vector3fs.clear();
        self.normals.clear();
        self.spectra.clear();
        self.strings.clear();
        self.textures.clear();
    }

    pub fn print(&self, indent: usize) {
        print_items("integer", indent, &self.ints);
        print_items("bool", indent, &self.bools);
        print_items("float", indent, &self.floats);
        print_items("point2", indent, &self.point2fs);
        print_items("vector2", indent, &self.vector2fs);
        print_items("point", indent, &self.point3fs);
        print_items("vector", indent, &self.vector3fs);
        print_items("normal", indent, &self.normals);
        print_items("string", indent, &self.strings);
        print_items("texture", indent, &self.textures);
        print_items("rgb", indent, &self.spectra);

    }
}

macro_rules! display_param_values {
    ($values:expr) => {{
        $values.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(" ")
    }}
}

impl Display for ParamSet {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        for item in self.ints.iter() {
            write!(
                f, "integer \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.bools.iter() {
            write!(
                f, "bool \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.floats.iter() {
            write!(
                f, "float \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.point2fs.iter() {
            write!(
                f, "point2 \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.vector2fs.iter() {
            write!(
                f, "vector2 \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.point3fs.iter() {
            write!(
                f, "point3 \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.vector3fs.iter() {
            write!(
                f, "vector3 \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.normals.iter() {
            write!(
                f, "normals \"{}\" [ {} ] ", item.name,
                display_param_values!(&item.values))?;
        }
        for item in self.strings.iter() {
            write!(
                f, "integer \"{}\" [ {} ] ", item.name,
                item.values.iter()
                    .map(|s| format!("\"{}\" ", s))
                    .collect::<Vec<_>>()
                    .join(""))?;
        }
        for item in self.textures.iter() {
            write!(
                f, "integer \"{}\" [ {} ] ", item.name,
                item.values.iter()
                    .map(|s| format!("\"{}\" ", s))
                    .collect::<Vec<_>>()
                    .join(""))?;
        }
        for item in self.spectra.iter() {
            write!(
                f, "integer \"{}\" [ {} ] ", item.name,
                item.values.iter()
                    .map(|s| {
                        let rgb = s.to_rgb();
                        format!("[ {} {} {} ] ", rgb[0], rgb[1], rgb[2])
                    })
                    .collect::<Vec<_>>()
                    .join(""))?;
        }

        Ok(())
    }
}

pub fn print_items<T: Display>(ty: &str, indent: usize, items: &[ParamSetItem<T>]) {
    for item in items {
        let mut np = 0;
        let mut s = format!("\n{:indent$}\"{} {}\" [ ", "", ty, item.name, indent=indent+8);
        np += s.len();
        print!("{}", s);

        for (i, val) in item.values.iter().enumerate() {
            s = format!("{}", val);
            np += s.len();
            print!("{}", s);

            if np > 80 && i < item.nvalues - 1 {
                s = format!("\n{:indent$}", "", indent=indent+8);
                np = s.len();
                print!("{}", s)
            }

            print!("] ")
        }
    }
}

// TextureParams Declarations
pub struct TextureParams<'a> {
    float_textures      : &'a HashMap<String, Arc<TextureFloat>>,
    spectrum_textures   : &'a HashMap<String, Arc<TextureSpec>>,
    geo_params          : &'a ParamSet,
    material_params     : &'a ParamSet
}

impl<'a> TextureParams<'a> {
    pub fn new(
        geo_params: &'a ParamSet, material_params: &'a ParamSet,
        float_textures: &'a HashMap<String, Arc<TextureFloat>>,
        spectrum_textures: &'a HashMap<String, Arc<TextureSpec>>) -> Self
    {
        Self {
            geo_params,
            material_params,
            float_textures,
            spectrum_textures
        }
    }

    pub fn find_float(&mut self, n: &str, d: Float) -> Float {
        self.geo_params.find_one_float(n, self.material_params.find_one_float(n, d))
    }

    pub fn find_string(&mut self, n: &str, d: &str) -> String {
        self.geo_params.find_one_string(n, self.material_params.find_one_string(n, d.to_owned()))
    }

    pub fn find_filename(&mut self, n: &str, d: &str) -> String {
        self.geo_params.find_one_filename(n, &self.material_params.find_one_filename(n, d))
    }

    pub fn find_int(&mut self, n: &str, d: isize) -> isize {
        self.geo_params.find_one_int(n, self.material_params.find_one_int(n, d))
    }

    pub fn find_bool(&mut self, n: &str, d: bool) -> bool {
        self.geo_params.find_one_bool(n, self.material_params.find_one_bool(n, d))
    }

    pub fn find_point3f(&mut self, n: &str, d: Point3f) -> Point3f {
        self.geo_params.find_one_point3f(n, self.material_params.find_one_point3f(n, d))
    }

    pub fn find_vector3f(&mut self, n: &str, d: Vector3f) -> Vector3f {
        self.geo_params.find_one_vector3f(n, self.material_params.find_one_vector3f(n, d))
    }

    pub fn find_normal3f(&mut self, n: &str, d: Normal3f) -> Normal3f {
        self.geo_params.find_one_normal3f(n, self.material_params.find_one_normal3f(n, d))
    }

    pub fn find_spectrum(&mut self, n: &str, d: Spectrum) -> Spectrum {
        self.geo_params.find_one_spectrum(n, self.material_params.find_one_spectrum(n, d))
    }

    pub fn get_spectrumtexture_ornull(&mut self, n: &str) -> Option<Arc<TextureSpec>> {
        let mut name = self.geo_params.find_texture(n, "".to_owned());

        if name.is_empty() {
            let mut count = 0;
            let s = self.geo_params.find_spectrum(n, &mut count);

            if let Some(ref specs) = s {
                if count > 1 {
                    warn!("Ignoring excess values provided with parameter \"{}\"", n);
                }

                let t = ConstantTexture::new(specs[0]);
                return Some(Arc::new(t.into()));
            }

            name = self.material_params.find_texture(n, "".to_owned());

            if name.is_empty() {
                let mut count = 0;
                let s = self.material_params.find_spectrum(n, &mut count);

                if let Some(ref specs) = s {
                    if count > 1 {
                        warn!("Ignoring excess values provided with parameter \"{}\"", n);
                    }

                    let t = ConstantTexture::new(specs[0]);
                    return Some(Arc::new(t.into()));
                }
            }

            if name.is_empty() { return None; }
        }

        if self.spectrum_textures.contains_key(&name) {
            return Some(self.spectrum_textures[&name].clone());
        }

        error!("Couldn't find spectrum texture named \"{}\" for parameter \"{}\"", name, n);
        None
    }

    pub fn get_spectrumtexture(&mut self, n: &str, def: Spectrum) -> Arc<TextureSpec> {
        self.get_spectrumtexture_ornull(n)
            .unwrap_or_else(|| {
                let t = ConstantTexture::new(def);
                Arc::new(t.into())
            })
    }

    pub fn get_floattexture(&mut self, n: &str, def: Float) -> Arc<TextureFloat> {
        self.get_floattexture_ornull(n)
            .unwrap_or_else(|| {
                let t = ConstantTexture::new(def);
                Arc::new(t.into())
            })
    }

    pub fn get_floattexture_ornull(&mut self, n: &str) -> Option<Arc<TextureFloat>> {
        // Check the shape parameters first
        let mut name = self.geo_params.find_texture(n, "".to_owned());

        if name.is_empty() {
            let mut count = 0;
            let s = self.geo_params.find_float(n, &mut count);

            if let Some(ref specs) = s {
                if count > 1 {
                    warn!("Ignoring excess values provided with parameter \"{}\"", n);
                }

                let t = ConstantTexture::new(specs[0]);
                return Some(Arc::new(t.into()));
            }

            name = self.material_params.find_texture(n, "".to_owned());

            if name.is_empty() {
                let mut count = 0;
                let s = self.material_params.find_float(n, &mut count);

                if let Some(ref specs) = s {
                    if count > 1 {
                        warn!("Ignoring excess values provided with parameter \"{}\"", n);
                    }

                    let t = ConstantTexture::new(specs[0]);
                    return Some(Arc::new(t.into()));
                }
            }

            if name.is_empty() { return None; }
        }

        // We have a texture name, from either the shape or material's parameters
        if self.float_textures.contains_key(&name) {
            return Some(self.float_textures[&name].clone());
        }

        error!("Couldn't find float texture named \"{}\" for parameter \"{}\"", name, n);
        None
    }

    fn report_unused_material_params<T>(
        mtl: &[ParamSetItem<T>],
        geom: &[ParamSetItem<T>]) {
        for param in mtl.iter() {
            if param.looked_up.get() { continue; }

            // Don't complain about any unused material parameters if their
            // values were provided by a shape parameter
            if geom.iter().find(|gp| gp.name == param.name).is_none() {
                warn!("Paramter \"{}\" not used", param.name)
            }
        }
    }

    pub fn report_unused(&self) {
        self.geo_params.report_unused();
        TextureParams::report_unused_material_params(&self.material_params.ints, &self.geo_params.ints);
        TextureParams::report_unused_material_params(&self.material_params.bools, &self.geo_params.bools);
        TextureParams::report_unused_material_params(&self.material_params.floats, &self.geo_params.floats);
        TextureParams::report_unused_material_params(&self.material_params.point2fs, &self.geo_params.point2fs);
        TextureParams::report_unused_material_params(&self.material_params.vector2fs, &self.geo_params.vector2fs);
        TextureParams::report_unused_material_params(&self.material_params.point3fs, &self.geo_params.point3fs);
        TextureParams::report_unused_material_params(&self.material_params.vector3fs, &self.geo_params.vector3fs);
        TextureParams::report_unused_material_params(&self.material_params.normals, &self.geo_params.normals);
        TextureParams::report_unused_material_params(&self.material_params.spectra, &self.geo_params.spectra);
        TextureParams::report_unused_material_params(&self.material_params.strings, &self.geo_params.strings);
        TextureParams::report_unused_material_params(&self.material_params.textures, &self.geo_params.textures);
    }

}