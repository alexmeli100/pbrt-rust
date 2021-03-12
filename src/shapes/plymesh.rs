use crate::core::transform::Transform;
use crate::core::paramset::ParamSet;
use bumpalo::core_alloc::sync::Arc;
use std::collections::HashMap;
use crate::core::texture::TextureFloat;
use crate::core::shape::Shapes;
use std::fs::File;
use std::io::BufReader;
use ply_rs::{ parser, ply };
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::geometry::normal::Normal3f;
use ply_rs::ply::Property;
use log::{error, warn, info};
use crate::textures::constant::ConstantTexture;
use crate::shapes::triangle::create_trianglemesh;

pub fn create_plymesh(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool, params: &ParamSet,
    textures: &HashMap<String, Arc<TextureFloat>>) -> Vec<Arc<Shapes>> {
    let filename = params.find_one_filename("filename", "");
    let f = File::open(&filename).unwrap();
    let mut reader = BufReader::new(f);

    let vertex_parser = parser::Parser::<Vertex>::new();
    let face_parser = parser::Parser::<Face>::new();

    let header = vertex_parser.read_header(&mut reader).unwrap();
    let mut vcount = 0;
    let mut fcount = 0;
    let mut has_normals = false;
    let mut has_texture = false;

    for (k, e) in &header.elements {
        if k == "vertex" {
            vcount = e.count;
            if !e.properties.contains_key("x") ||
               !e.properties.contains_key("y") ||
               !e.properties.contains_key("z") {
                error!(
                    "PLY file \"{}\": Vertex coordinate property not found",
                    filename);
                return Vec::new();
            }

            if e.properties.contains_key("nx") &&
               e.properties.contains_key("nx") &&
               e.properties.contains_key("nx") {
                has_normals = true;
            }

            if (e.properties.contains_key("u") && e.properties.contains_key("v")) ||
               (e.properties.contains_key("s") && e.properties.contains_key("t")) ||
               (e.properties.contains_key("texture_u") && e.properties.contains_key("texture_v")) ||
               (e.properties.contains_key("texture_s") && e.properties.contains_key("texture_t")) {
                has_texture = true;
            }
        } else if k == "face" {
            fcount = e.count
        }
    }

    if vcount == 0 || fcount == 0 {
        error!(
            "PLY file \"{}\" is invalid! No face/vertex elements found",
            filename);
        return Vec::new();
    }

    info!(
        "Loading PLY file with {} vertices and {} faces",
        vcount, fcount
    );

    let mut vertices = Vec::new();
    let mut faces = Vec::new();

    for (_, elem) in &header.elements {
        match elem.name.as_str() {
            "vertex" => {
                vertices = vertex_parser
                    .read_payload_for_element(&mut reader, elem, &header)
                    .unwrap();
            }
            "face"  => {
                faces = face_parser
                    .read_payload_for_element(&mut reader, elem, &header)
                    .unwrap();
            }
            _ => unreachable!()
        }
    }

    let indices: Vec<usize> = faces
        .into_iter()
        .flat_map(|f| {
            let l = f.indices.len();
            if l != 3 && l != 4 {
                warn!(
                    "plymesh: Ignoring face with {} vertices \
                    (only triangles and quads are supported!)", l);
                Vec::new()
            } else {
                let mut v = Vec::with_capacity(6);
                v.push(f.indices[0]);
                v.push(f.indices[1]);
                v.push(f.indices[2]);

                if l == 4 {
                    v.push(f.indices[3]);
                    v.push(f.indices[0]);
                    v.push(f.indices[2]);
                }

                v
            }
        }).collect();

    let mut p = Vec::with_capacity(vcount);
    let mut n = Vec::with_capacity(vcount);
    let mut uv = Vec::with_capacity(vcount);

    for v in vertices {
        p.push(v.p);

        if has_normals {
            n.push(v.n);
        }

        if has_texture {
            uv.push(v.uv);
        }
    }

    // Look up an alpha texture if applicable
    let mut alpha_tex: Option<Arc<TextureFloat>> = None;
    let tex_name = params.find_texture("alpha", "".to_owned());

    if !tex_name.is_empty() {
        alpha_tex = match textures.get(&tex_name) {
            Some(t) =>  Some(t.clone()),
            _       => {
                error!(
                    "Couldn't find float texture \"{}\" for \"alpha\" parameter",
                    tex_name
                );

                None
            }
        }
    } else if params.find_one_float("alpha", 1.0) == 0.0 {
        alpha_tex = Some(Arc::new(ConstantTexture::new(0.0).into()));
    }

    let mut shadow_alphatex: Option<Arc<TextureFloat>> = None;
    let tex_name = params.find_texture("shadowalpha", "".to_owned());

    if !tex_name.is_empty() {
        shadow_alphatex = match textures.get(&tex_name) {
            Some(t) =>  Some(t.clone()),
            _       => {
                error!(
                    "Couldn't find float texture \"{}\" for \"shadowalpha\" parameter",
                    tex_name
                );

                None
            }
        }
    } else if params.find_one_float("shadowalpha", 1.0) == 0.0 {
        shadow_alphatex = Some(Arc::new(ConstantTexture::new(0.0).into()));
    }

    create_trianglemesh(
        o2w, w2o, reverse_orientation,
        indices.len() / 3 , indices, p.len(), p,
        Vec::new(), n, uv, alpha_tex, shadow_alphatex)
}

#[derive(Default)]
struct Vertex {
    p   : Point3f,
    n   : Normal3f,
    uv  : Point2f
}

impl ply::PropertyAccess for Vertex {
    fn new() -> Self {
        Vertex::default()
    }

    fn set_property(&mut self, name: String, property: Property) {
        match (name.as_str(), property) {
            ("x", ply::Property::Float(v)) => self.p.x = v,
            ("y", ply::Property::Float(v)) => self.p.y = v,
            ("z", ply::Property::Float(v)) => self.p.z = v,
            ("nx", ply::Property::Float(v)) => self.n.x = v,
            ("ny", ply::Property::Float(v)) => self.n.y = v,
            ("nz", ply::Property::Float(v)) => self.n.z = v,
            ("u", ply::Property::Float(v)) |
            ("texture_u", ply::Property::Float(v)) |
            ("s", ply::Property::Float(v)) |
            ("texture_s", ply::Property::Float(v)) => self.uv.x = v,
            ("v", ply::Property::Float(v)) |
            ("texture_v", ply::Property::Float(v)) |
            ("t", ply::Property::Float(v)) |
            ("texture_t", ply::Property::Float(v)) => self.uv.y = v,

            _ => error!("Unknow property \"{}\" found for vectext element", name)
        }
    }
}

#[derive(Default)]
struct Face {
    indices: Vec<usize>
}

impl ply::PropertyAccess for Face {
    fn new() -> Self {
        Face::default()
    }

    fn set_property(&mut self, name: String, property: Property) {
        match (name.as_str(), property) {
            ("vertex_indices", ply::Property::ListInt(v))  =>
                self.indices = v.iter().map(|x| *x as usize).collect(),
            ("vertex_indices", ply::Property::ListUInt(v)) =>
                self.indices = v.iter().map(|x| *x as usize).collect(),
            (k, p) =>
                error!("Face: Invalid combination key/value for key {} / prop {:?}", k, p)
        }
    }
}