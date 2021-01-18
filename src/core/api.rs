use crate::core::transform::{Transform, AnimatedTransform};
use std::ops::{IndexMut, Index, Deref};
use std::sync::Arc;
use std::sync::Mutex;
use lazy_static::lazy_static;
use static_assertions::{const_assert_eq};
use crate::core::material::Materials;
use crate::core::paramset::{ParamSet, TextureParams};
use std::collections::HashMap;
use crate::core::texture::{Textures, TextureFloat, TextureSpec};
use crate::core::pbrt::{Float, Options};
use crate::core::spectrum::Spectrum;
use log::{info, error, warn};
use crate::core::medium::{Mediums, MediumInterface};
use crate::core::light::{Lights, AreaLights};
use crate::core::primitive::{Primitives, GeometricPrimitive, TransformedPrimitive};
use typed_arena::Arena;
use crate::{stat_counter, stat_memory_counter, stat_percent, stat_int_distribution};
use crate::core::stats::*;
use nalgebra::Matrix4;
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::point::Point3f;
use crate::core::shape::Shapes;
use crate::core::film::Film;
use crate::core::camera::Cameras;
use crate::core::sampler::Samplers;
use crate::accelerators::bvh::{create_bvh_accelerator, SplitMethod};
use crate::accelerators::kdtreeaccel::create_kdtree_accelerator;
use crate::core::filter::Filters;
use bumpalo::Bump;
use crate::accelerators::bvh::BVHAccel;
use crate::core::integrator::Integrators;
use crate::core::scene::Scene;
use crate::materials::matte::create_matte_material;
use crate::materials::plastic::create_plastic_material;
use crate::materials::fourier::create_fourier_material;
use crate::materials::mix::create_mix_material;
use crate::textures::constant::{create_constant_float, create_constant_spectrum};
use crate::textures::scaled::{ create_scale_float, create_scale_spectrum};
use crate::textures::mix::{create_mix_float, create_mix_spectrum};
use crate::textures::biler::{create_biler_float, create_biler_spectrum};
use crate::textures::imagemap::{create_image_float, create_image_spectrum};
use crate::textures::uv::{create_uv_float, create_uv_spectrum};
use crate::textures::checkerboard::{create_checkerboard_float, create_checkerboard_spectrum};
use crate::textures::dots::{create_dots_float, create_dots_spectrum};
use crate::textures::fbm::{create_fbm_float, create_fbm_spectrum};
use crate::textures::wrinkled::{create_wrinkled_float, create_wrinkled_spectrum};
use crate::textures::marble::{create_marble_float, create_marble_spectrum};
use crate::textures::windy::{ create_windy_float, create_windy_spectrum};
use crate::filters::boxfilter::create_box_filter;
use crate::filters::gaussian::create_gaussian_filter;
use crate::filters::mitchell::create_mitchell_filter;
use crate::filters::sinc::create_sinc_filter;
use crate::filters::triangle::create_triangle_filter;

const MAX_TRANSFORMS: usize = 2;
const START_TRANSFORM_BITS: usize = 1 << 0;
const END_TRANSFORM_BITS: usize = 1 << 1;
const ALL_TRANSFORM_BITS: usize = (1 << MAX_TRANSFORMS) - 1;

// Declare stat counters
stat_counter!("Scene/Materials created", nmaterials_created);
stat_counter!("Scene/Object instances used", nobject_instances_used);
stat_counter!("Scene/Object instances created", nobject_instances_created);
stat_memory_counter!("Memory/TransformCache", transform_cache_bytes);
stat_percent!("Scene/TransformCache hits", ntransform_cache_hitsper);
stat_int_distribution!("Scene/Probes per TransformCache lookup", transform_cache_probes);

// Initialize stat counters
pub fn init_stats() {
    nmaterials_created::init();
    nobject_instances_used::init();
    nobject_instances_created::init();
    transform_cache_bytes::init();
    ntransform_cache_hitsper::init();
    transform_cache_probes::init();
}

#[derive(Debug, Default, Copy, Clone)]
struct TransformSet {
    t: [Transform; MAX_TRANSFORMS]
}

impl TransformSet {
    fn is_animated(&self) -> bool {
        for i in 0..MAX_TRANSFORMS - 1 {
            if self.t[i] != self.t[i + 1] { return true; }
        }

        false
    }

    fn inverse(ts: &Self) -> Self {
        let mut tinv = TransformSet::default();

        ts.t
            .iter()
            .map(|t| Transform::inverse(t))
            .enumerate()
            .for_each(|(i, t)| tinv.t[i] = t)
        ;

        tinv
    }
}

impl Index<usize> for TransformSet {
    type Output = Transform;

    fn index(&self, i: usize) -> &Self::Output {
        assert!(i > 0);
        assert!(i < MAX_TRANSFORMS);

        &self.t[i]
    }
}

impl IndexMut<usize> for TransformSet {
    fn index_mut(&mut self, i: usize) -> &mut Self::Output {
        assert!(i > 0);
        assert!(i < MAX_TRANSFORMS);

        &mut self.t[i]
    }
}

struct MaterialInstance {
    name      : String,
    material  : Option<Arc<Materials>>,
    params    : ParamSet
}

// MaterialInstance represents both an instance of a material as well as
// the information required to create another instance of it (possibly with
// different parameters from the shape).
impl MaterialInstance {
    fn new(name: &str, material: Option<Arc<Materials>>, params: ParamSet) -> Self {
        Self {
            name: name.to_owned(),
            material,
            params
        }
    }
}

struct RenderOptions {
    transform_start_time    : Float,
    transform_end_time      : Float,
    filter_name             : String,
    filter_params           : ParamSet,
    film_name               : String,
    film_params             : ParamSet,
    sampler_name            : String,
    sampler_params          : ParamSet,
    accelerator_name        : String,
    accelerator_params      : ParamSet,
    integrator_name         : String,
    integrator_params       : ParamSet,
    camera_name             : String,
    camera_params           : ParamSet,
    camera_to_world         : TransformSet,
    named_media             : HashMap<String, Arc<Mediums>>,
    lights                  : Vec<Arc<Lights>>,
    primitives              : Vec<Arc<Primitives>>,
    instances               : HashMap<String, Vec<Arc<Primitives>>>,
    current_instance        : Option<Vec<Arc<Primitives>>>,
    have_scattering_media   : bool
}

impl Default for RenderOptions {
    fn default() -> Self {
        Self {
            transform_start_time: 0.0,
            transform_end_time: 1.0,
            filter_name: "box".to_owned(),
            film_name: "image".to_owned(),
            sampler_name: "halton".to_owned(),
            accelerator_name: "bvh".to_owned(),
            integrator_name: "path".to_owned(),
            camera_name: "perspective".to_owned(),
            ..Default::default()
        }
    }
}

impl RenderOptions {
    fn make_scene(&mut self) -> Option<Scene> {
        unimplemented!()
    }

    fn make_integrator(&self) -> Option<Integrators> {
        unimplemented!()
    }

    fn make_camera(&self) -> Option<Cameras> {
        unimplemented!()
    }
}

type FloatTextureMap = HashMap<String, Arc<TextureFloat>>;
type SpectrumTextureMap = HashMap<String, Arc<TextureSpec>>;
type NamedMaterialMap = HashMap<String, Arc<MaterialInstance>>;

#[derive(Default, Clone)]
struct GraphicsState {
    opts                     : Options,
    float_textures           : Arc<FloatTextureMap>,
    float_textures_shared    : bool,
    spectrum_textures        : Arc<SpectrumTextureMap>,
    spectrum_textures_shared : bool,
    named_materials          : Arc<NamedMaterialMap>,
    named_materials_shared   : bool,
    current_material         : Option<Arc<MaterialInstance>>,
    area_light_params        : ParamSet,
    area_light               : String,
    reverse_orientation      : bool,
    current_inside_medium    : String,
    current_outside_medium   : String,
}

impl GraphicsState {
    pub fn new() -> Self {
        // TODO: GraphicState::new()
        unimplemented!()
    }

    pub fn get_materialfor_shape(&self, params: &ParamSet) -> Option<Arc<Materials>> {
        assert!(self.current_material.is_some());

        if shape_may_set_materialparameters(params) {
            let mut mp = TextureParams::new(
                &params,
                &self.current_material.as_ref().unwrap().params,
                &*self.float_textures,
                &*self.spectrum_textures
            );

            return make_material(
                &self.current_material.as_ref().unwrap().name,
                &mut mp, self, &self.opts)
        }

        self.current_material.as_ref().unwrap().material.clone()
    }

    fn create_medium_interface(&self, opts: &RenderOptions) -> MediumInterface {
        let mut m = MediumInterface::default();
        let inside = &self.current_inside_medium;
        let outside = &self.current_outside_medium;

        if !inside.is_empty() {
            match opts.named_media.get(inside) {
                Some(media) => m.inside = Some(media.clone()),
                _ => error!("Named medium \"{}\" undefined", inside)
            }
        }


        if !outside.is_empty() {
            match opts.named_media.get(outside) {
                Some(media) => m.inside = Some(media.clone()),
                _ => error!("Named medium \"{}\" undefined", outside)
            }
        }

        m
    }
}

lazy_static! {
    static ref CACHE_ARENA: Mutex<Arena<Transform>> = Mutex::new(Arena::new());
}

struct TransformCache {
    table: Vec<Option<Arc<Transform>>>,
    table_occupancy: usize,
    arena: Bump
}

impl Default for TransformCache {
    fn default() -> Self {
        Self::new()
    }
}

impl TransformCache {
    fn new() -> Self {
        Self {
            table: Vec::with_capacity(512),
            table_occupancy: 0,
            arena: Bump::new()
        }
    }

    fn hash(t: &Transform) -> u64 {
        let mut size = std::mem::size_of::<Matrix4<Float>>();
        let mut hash = 14695981039346656037_u64;

        unsafe {
            let mut ptr = &t.m as *const Matrix4<Float> as *const char;

            while size > 0 {
                hash ^= *ptr as u64;
                hash *= 1099511628211_u64;
                ptr = ptr.add(1);
                size -= 1;
            }
        }

        hash
    }

    fn lookup(&mut self, t: &Transform) -> Arc<Transform> {
        ntransform_cache_hitsper::inc_den();
        let mut offset = TransformCache::hash(t) & (self.table.len() as u64 - 1);
        let mut step = 1;

        loop {
            // Keep looking until we find the Transform or determine that it's not present
            let val = self.table[offset as usize].as_ref().unwrap().deref();
            if self.table[offset as usize].is_none() || val == t {
                break;
            }

            // Advance using quadratic probing
            offset = (offset + step * step) & (self.table.len() as u64 - 1);
            step += 1;
        }

        transform_cache_probes::report_value(step);
        let tcached = &self.table[offset as usize];

        if tcached.is_none() {

            let tr = self.arena.alloc(Arc::new(*t));
            TransformCache::insert(&mut self.table, &mut self.table_occupancy, tr);
            return tr.clone();
        }

        ntransform_cache_hitsper::inc_num();
        tcached.as_ref().unwrap().clone()
    }

    fn clear(&mut self) {
        let bytes = self.arena.allocated_bytes() + self.table.len() * std::mem::size_of::<Option<Arc<Transform>>>();
        transform_cache_bytes::add(bytes as u64);
        self.table_occupancy = 0;
        self.table.clear();
        self.table.resize(512, Default::default());
        self.arena.reset();

    }

    fn insert(table: &mut Vec<Option<Arc<Transform>>>, occupancy: &mut usize, tnew: &Arc<Transform>) {
        if *occupancy == table.len() / 2 {
            *occupancy += 1;
            TransformCache::grow(table);
        }

        let base_offset= TransformCache::hash(tnew) & (table.len() as u64 - 1);
        let mut nprobes = 0;

        loop {
            // Quadratic probing
            let offset = (base_offset + nprobes / 2 + nprobes * nprobes / 2) & (table.len() as u64 - 1);

            if table[offset as usize].is_none() {
                table[offset as usize] = Some(tnew.clone());
                return;
            }

            nprobes += 1;
        }
    }

    fn grow(table: &mut Vec<Option<Arc<Transform>>>) {
        let mut new_table: Vec<Option<Arc<Transform>>> = Vec::with_capacity(2 * table.len());
        info!("Growing transform cache hash table to {}", new_table.capacity());
        let size = table.len();

        // Insert current elements in new_table
        for entry in table.iter_mut() {
            if entry.is_none() {
                continue;
            }

            let base_offset = TransformCache::hash(&*entry.as_ref().unwrap()) & (size as u64 - 1);
            let mut nprobes = 0;

            loop {
                // Quadratic probing
                let offset = (base_offset + nprobes / 2 + nprobes * nprobes / 2) & (size as u64 - 1);

                if new_table[offset as usize].is_none() {
                    new_table[offset as usize] = entry.take();
                    break;
                }

                nprobes += 1;
            }
        }

        std::mem::swap(table, &mut new_table)
    }
}

#[derive(Eq, PartialEq)]
enum APIState {
    Uninitialized,
    OptionsBlock,
    WorldBlock
}

fn make_shapes(name: &str, obj_2_world: &Transform, world_2_obj: &Transform,
               rorientation: bool, params: &ParamSet) -> Vec<Arc<Shapes>> {
    unimplemented!()
}

fn make_material(name: &str, mp: &mut TextureParams, state: &GraphicsState, opts: &Options) -> Option<Arc<Materials>> {
    // TODO: Add more materials
    if name.is_empty() || name == "none" {
        return None;
    }

    let material = match name {
        "matte"     => Some(Arc::new(create_matte_material(mp))),
        "plastic"   => Some(Arc::new(create_plastic_material(mp))),
        "fourier"   => Some(Arc::new(create_fourier_material(mp))),
        "mix"       => {
            let m1 = mp.find_string("namedmaterial1", "");
            let m2 = mp.find_string("namedmaterial2", "");

            let mat1 = if !state.named_materials.contains_key(&m1) {
                error!("Named material \"{}\" undefined. Using \"matte\"", m1);
                make_material("matte", mp, state, opts)
            } else {
                state.named_materials.get(&m1).unwrap().material.clone()
            };

            let mat2 = if !state.named_materials.contains_key(&m2) {
                error!("Named material \"{}\" undefined. Using \"matte\"", m2);
                make_material("matte", mp, state, opts)
            } else {
                state.named_materials.get(&m2).unwrap().material.clone()
            };

            Some(Arc::new(create_mix_material(mp, mat1.unwrap(), mat2.unwrap())))
        }
        _ => {
            warn!("Material \"{}\" unknown. Using \"matte\".", name);
            Some(Arc::new(create_matte_material(mp)))
        }
    };

    if (name == "subsurface" || name == "kdsubsurface") ||
        (opts.integrator_name != "path" &&
         (opts.integrator_name != "volpath")) {
        warn!(
            "Subsurface scattering material \"{}\" used, but \"{}\"\
             integrator doesn't support subsurface scattering. \
             Use \"path\" or \"volpath\"", name, opts.integrator_name
        );
    }

    mp.report_unused();

    // TODO ++nMaterialsCreated
    material
}

fn make_float_texture(name: &str, t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureFloat>> {
    let tex = match name {
        "constant"      => create_constant_float(t2w, tp),
        "scale"         => create_scale_float(t2w, tp),
        "mix"           => create_mix_float(t2w, tp),
        "bilerp"        => create_biler_float(t2w, tp),
        "imagemap"      => create_image_float(t2w, tp),
        "uv"            => create_uv_float(t2w, tp),
        "checkerboard"  => create_checkerboard_float(t2w, tp),
        "dots"          => create_dots_float(t2w, tp),
        "fbm"           => create_fbm_float(t2w, tp),
        "wrinkled"      => create_wrinkled_float(t2w, tp),
        "marble"        => create_marble_float(t2w, tp),
        "windy"         => create_windy_float(t2w, tp),
        _               => {
            warn!("Float texture \"{}\" unknown.", name);
            None
        }
    };

    tp.report_unused();

    tex
}

fn make_spectrum_texture(name: &str, t2w: &Transform, tp: &mut TextureParams) -> Option<Arc<TextureSpec>> {
    let tex = match name {
        "constant"      => create_constant_spectrum(t2w, tp),
        "scale"         => create_scale_spectrum(t2w, tp),
        "mix"           => create_mix_spectrum(t2w, tp),
        "bilerp"        => create_biler_spectrum(t2w, tp),
        "imagemap"      => create_image_spectrum(t2w, tp),
        "uv"            => create_uv_spectrum(t2w, tp),
        "checkerboard"  => create_checkerboard_spectrum(t2w, tp),
        "dots"          => create_dots_spectrum(t2w, tp),
        "fbm"           => create_fbm_spectrum(t2w, tp),
        "wrinkled"      => create_wrinkled_spectrum(t2w, tp),
        "marble"        => create_marble_spectrum(t2w, tp),
        "windy"         => create_windy_spectrum(t2w, tp),
        _               => {
            warn!("Spectrum texture \"{}\" unknown.", name);
            None
        }
    };

    tp.report_unused();

    tex
}

fn make_medium(name: &str, params: &ParamSet, medium_2_world: &Transform) -> Option<Arc<Mediums>> {
    unimplemented!()
}

fn make_light(name: &str, params: &ParamSet, light_2_world: &Transform, minterface: &MediumInterface) -> Option<Arc<Lights>> {
    unimplemented!()
}

fn make_area_light(name: &str, light_2_world: &Transform, minterface: &MediumInterface,
                   params: &ParamSet, shape: &Arc<Shapes>) -> Option<Arc<AreaLights>> {
    unimplemented!()
}

fn make_accelerator(name: &str, prims: Vec<Arc<Primitives>>, params: &ParamSet) -> Arc<Primitives> {
    let acc = match name {
        "bvh"    => create_bvh_accelerator(prims, params),
        "kdtree" => create_kdtree_accelerator(prims, params),
        _ => {
            warn!("Accelerator \"{}\" unknown. Using BVH.", name);
            create_bvh_accelerator(prims, params)
        }
    };

    params.report_unused();
    acc
}

fn make_camera(name: &str, params: &ParamSet, cam_2_worldset: &TransformSet,
               transform_start: Float, transform_end: Float, film: Option<Arc<Film>>) -> Option<Cameras> {
    unimplemented!()
}

fn make_sampler(name: &str, params: &ParamSet, film: Option<Arc<Film>>) -> Arc<Samplers>  {
    unimplemented!()
}

fn make_filter(name: &str, params: &ParamSet) -> Filters {
    let f = match name {
        "box"       => create_box_filter(params),
        "gaussian"  => create_gaussian_filter(params),
        "mitchell"  => create_mitchell_filter(params),
        "sinc"      => create_sinc_filter(params),
        "triangle"  => create_triangle_filter(params),
        _           => panic!(format!("Filter \"{}\" unknown", name))
    };

    params.report_unused();

    f
}

fn make_film(name: &str, params: &ParamSet, filter: Filters) -> Option<Arc<Film>> {
    unimplemented!()
}

pub struct API {
    current_state: APIState,
    curr_transform: TransformSet,
    active_transform_bits: u32,
    named_coordinate_system: HashMap<String, TransformSet>,
    render_options: RenderOptions,
    graphics_state: GraphicsState,
    pushed_graphics_states: Vec<GraphicsState>,
    pushed_transforms: Vec<TransformSet>,
    pushed_active_transformbits: Vec<u32>,
    indent_count: usize,
    opts: Options,
    transform_cache: TransformCache
}

macro_rules! verify_initialized {
    ($self:ident, $func:expr) => {{
        if (!$self.opts.cat || $self.opts.to_ply) && $self.current_state == APIState::Uninitialized {
            error!("init() must be before calling \"{}\".\nIgnoring", $func);
            return;
        }}
    }
}

macro_rules! verify_options {
    ($self:ident, $func:expr) => {
        if (!$self.opts.cat || $self.opts.to_ply) && $self.current_state == APIState::WorldBlock {
            error!("Options cannot be set inside world block;\n\"{}\" not allowed. Ignoring", $func);
            return;
        }
    }
}

macro_rules! verify_world {
    ($self:ident, $func:expr) => {
        if (!$self.opts.cat || $self.opts.to_ply) && $self.current_state == APIState::WorldBlock {
            error!("Scene description must be inside world block;\n\"{}\" not allowed. Ignoring", $func);
            return;
        }
    }
}

macro_rules! for_active_transform {
    ($self:ident, $tr:expr) => {
        for t in $self.curr_transform.t[0..MAX_TRANSFORMS].iter_mut() {
            *t = $tr
        }
    };

    ($self:ident, *$s:ident, $tr:expr) => {
        for t in $self.curr_transform.t[0..MAX_TRANSFORMS].iter_mut() {
            *t = *t * $tr
        }
    }
}

macro_rules! warn_if_animated_transform {
    ($self:ident, $func:expr) => {
        if $self.curr_transform.is_animated() {
            warn!("Animated transformations set; ignoring for \"{}\" and using the start transform only", $func);
        }
    }
}

impl API {
    pub fn init(&mut self, opts: Options) {
        // TODO: API::init
        self.opts = opts;

        Spectrum::init();
    }

    pub fn identity(&mut self) {
        let tr = Transform::new();
        verify_initialized!(self, "Identity");
        for_active_transform!(self, tr);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} Identity", indent=self.indent_count);
        }
    }

    pub fn translate(&mut self, dx: Float, dy: Float, dz: Float) {
        let tr = Transform::translate(&Vector3f::new(dx, dy, dz));
        verify_initialized!(self, "Translate");
        for_active_transform!(self, *t, tr);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} Translate {} {} {}", "", dx, dy, dz, indent=self.indent_count);
        }
    }

    pub fn transform(&mut self, tr: Vec<Float>) {
        assert_eq!(tr.len(), 16);
        let t = Transform::from_matrix(&Matrix4::from_row_slice(&tr));
        verify_initialized!(self, "Transform");
        for_active_transform!(self, t);

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Transform [ ", indent=self.indent_count);
            tr.iter().for_each(|i| print!("{} ", i));
            println!("]");
        }
    }

    pub fn concat_transform(&mut self, tr: Vec<Float>) {
        assert_eq!(tr.len(), 16);
        let t = Transform::from_matrix(&Matrix4::from_row_slice(&tr));
        verify_initialized!(self, "ConcatTransform");
        for_active_transform!(self, t);

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} ConcatTransform [ ", "", indent=self.indent_count);
            tr.iter().for_each(|i| print!("{} ", i));
            println!("]");
        }
    }

    pub fn rotate(&mut self, angle: Float, dx: Float, dy: Float, dz: Float) {
        let t = Transform::rotate(angle, &Vector3f::new(dx, dy, dz));
        verify_initialized!(self, "Rotate");
        for_active_transform!(self, *t, t);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} Rotate {} {} {} {}", "", angle, dx, dy, dz, indent=self.indent_count);
        }
    }

    pub fn scale(&mut self, sx: Float, sy: Float, sz: Float) {
        let t = Transform::scale(sx, sy, sz);
        verify_initialized!(self, "Rotate");
        for_active_transform!(self, *t, t);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} Scale {} {} {}", "", sx, sy, sz , indent=self.indent_count);
        }
    }

    pub fn lookat(&mut self, ex: Float, ey: Float, ez: Float, lx: Float,
                  ly: Float, lz: Float, ux: Float, uy: Float, uz: Float) {
        let pos = Point3f::new(ex, ey, ez);
        let look = Point3f::new(lx, ly, lz);
        let up = Vector3f::new(ux, uy, uz);
        let tr = Transform::look_at(&pos, &look, &up);
        verify_initialized!(self, "LookAt");
        for_active_transform!(self, *t, tr);

        if self.opts.cat || self.opts.to_ply {
            println!(
                "{:indent$} LookAt {} {} {}\n{:indent8$}{}{}{}\n{:indent8$}{}{}{}",
                "", ex, ey, ez, "", lx, ly, lz, "", ux, uy, uz,
                indent=self.indent_count, indent8=self.indent_count
            )
        }
    }

    pub fn coordinate_system(&mut self, name: &str) {
        verify_initialized!(self, "CoordinateSystem");
        self.named_coordinate_system.insert(name.to_owned(), self.curr_transform);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} CoordinateSystem \"{}\"", "", name, indent=self.indent_count);
        }
    }

    pub fn coord_sys_transform(&mut self, name: &str) {
        verify_initialized!(self, "CoordsysTransform");

        match self.named_coordinate_system.get(name) {
            Some(t) => self.curr_transform = *t,
            _ => warn!("Couldn't find named coordinate system \"{}\"", name)
        }

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} CoordSysTransform \"{}\"", "", name, indent=self.indent_count);
        }
    }

    pub fn active_transform_all(&mut self) {
        self.active_transform_bits = ALL_TRANSFORM_BITS as u32;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ActiveTransform All", "", indent=self.indent_count);
        }
    }

    pub fn active_transform_endtime(&mut self) {
        self.active_transform_bits = END_TRANSFORM_BITS as u32;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ActiveTransform EndTime", "", indent=self.indent_count);
        }
    }

    pub fn active_transform_starttime(&mut self) {
        self.active_transform_bits = START_TRANSFORM_BITS as u32;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ActiveTransform StartTime", "", indent=self.indent_count);
        }
    }

    pub fn transform_times(&mut self, start: Float, end: Float) {
        verify_options!(self, "TransformTimes");

        self.render_options.transform_start_time = start;
        self.render_options.transform_end_time = end;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} TransformTimes {} {}", "", start, end, indent=self.indent_count);
        }
    }

    pub fn pixel_filter(&mut self, name: &str, params: ParamSet) {
        verify_options!(self, "PixelFilter");

        self.render_options.film_name = name.to_owned();
        self.render_options.film_params = params.clone();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} PixelFilter \"{}\"", "", name, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn film(&mut self, ty: &str, params: ParamSet) {
        verify_options!(self, "Film");

        self.render_options.film_params = params.clone();
        self.render_options.film_name = ty.to_owned();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Film \"{}\"", "", ty, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn sampler(&mut self, name: &str, params: ParamSet) {
        verify_options!(self, "Film");

        self.render_options.sampler_params = params.clone();
        self.render_options.sampler_name = name.to_owned();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Sampler \"{}\"", "", name, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn accelerator(&mut self, name: &str, params: ParamSet) {
        verify_options!(self, "Accelerator");

        self.render_options.accelerator_params = params.clone();
        self.render_options.accelerator_name = name.to_owned();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Accelerator \"{}\"", "", name, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn integrator(&mut self, name: &str, params: ParamSet) {
        verify_options!(self, "Integrator");

        self.render_options.integrator_params = params.clone();
        self.render_options.integrator_name = name.to_owned();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Integrator \"{}\"", "", name, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn include(&mut self, name: &str) -> anyhow::Result<()> {
        unimplemented!()
    }

    pub fn camera(&mut self, name: &str, params: ParamSet) {
        verify_options!(self, "camera");

        self.render_options.camera_params = params.clone();
        self.render_options.camera_name = name.to_owned();
        self.render_options.camera_to_world = TransformSet::inverse(&self.curr_transform);
        self.named_coordinate_system.insert("name".to_owned(), self.render_options.camera_to_world);

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Accelerator \"{}\"", "", name, indent=self.indent_count);
            print!("{}", params);
            println!();
        }
    }

    pub fn make_named_medium(&mut self, name: &str, params: ParamSet) {
        verify_initialized!(self, "MakeNamedMedium");
        warn_if_animated_transform!(self, "MakeNamedMedium");

        let ty = params.find_one_string("type", "".to_owned());

        if ty.is_empty() {
            error!("No parameter string \"type\" found in MakeNamedMedium")
        } else {
            let medium = make_medium(&ty, &params, &self.curr_transform[0]);

            if let Some(m) = medium {
                self.render_options.named_media.insert(name.to_owned(), m);
            }
        }



        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} MakeNamedMedium \"{}\" ", "", name, indent=self.indent_count);
            params.print(self.indent_count);
            println!()
        }
    }

    pub fn medium_interface(&mut self, inside: &str, outside: &str) {
        verify_initialized!(self, "MediumInterface");
        self.graphics_state.current_inside_medium = inside.to_owned();
        self.graphics_state.current_outside_medium = outside.to_owned();
        self.render_options.have_scattering_media = true;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} MediumInterface \"{}\" \"{}\"\n", "", inside, outside, indent=self.indent_count);
        }
    }

    pub fn world_begin(&mut self) {
        verify_options!(self, "WorldBegin");
        let t = Transform::new();
        self.current_state = APIState::WorldBlock;
        for_active_transform!(self, t);
        self.active_transform_bits = ALL_TRANSFORM_BITS as u32;
        self.named_coordinate_system.insert("world".to_owned(), self.curr_transform);

        if self.opts.cat || self.opts.to_ply {
            print!("\n\nWorldBegin\n\n")
        }
    }

    pub fn attribute_begin(&mut self) {
        verify_world!(self, "AttributeBegin");
        self.pushed_graphics_states.push(self.graphics_state.clone());
        self.graphics_state.float_textures_shared = true;
        self.graphics_state.spectrum_textures_shared = true;
        self.graphics_state.named_materials_shared = true;
        self.pushed_transforms.push(self.curr_transform);
        self.pushed_active_transformbits.push(self.active_transform_bits);

        if self.opts.cat || self.opts.to_ply {
            println!("\n{:indent$} AttributeBegin", "", indent=self.indent_count);
            self.indent_count += 4;
        }
    }

    pub fn attribute_end(&mut self) {
        verify_world!(self, "AttributeEnd");

        if self.pushed_graphics_states.is_empty() {
            error!("Unmatched attribute_end() encountered. Ignoring it.");
            return;
        }

        self.graphics_state = self.pushed_graphics_states.pop().unwrap();
        self.curr_transform = self.pushed_transforms.pop().unwrap();
        self.active_transform_bits = self.pushed_active_transformbits.pop().unwrap();

        if self.opts.cat || self.opts.to_ply {
            self.indent_count -= 4;
            println!("{:indent$} AttributeEnd", "", indent=self.indent_count);
        }
    }

    pub fn transform_begin(&mut self) {
        verify_world!(self, "TransformBegin");
        self.pushed_transforms.push(self.curr_transform);
        self.pushed_active_transformbits.push(self.active_transform_bits);

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} TransformBegin", "", indent=self.indent_count);
            self.indent_count += 4;
        }
    }

    pub fn transform_end(&mut self) {
        verify_world!(self, "TransformEnd");

        if self.pushed_transforms.is_empty() {
            error!("Unmatched transform_end() encountered. Ignoring it.");
            return;
        }

        self.curr_transform = self.pushed_transforms.pop().unwrap();
        self.active_transform_bits = self.pushed_active_transformbits.pop().unwrap();

        if self.opts.cat || self.opts.to_ply {
            self.indent_count -= 4;
            println!("{:indent$} TransformEnd", "", indent=self.indent_count);
        }
    }

    pub fn texture(&mut self, name: &str, ty: &str, texname: &str, params: &ParamSet) {
        verify_world!(self, "Texture");

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} \"{}\" \"{}\" \"{}\" ", "", name, ty, texname, indent=self.indent_count);
            params.print(self.indent_count);
            println!();
            return;
        }

        let mut tp = TextureParams::new(
            params, params,
            &self.graphics_state.float_textures,
            &self.graphics_state.spectrum_textures);

        match ty {
            "float" => {
                // Create float texture and store in float_textures
                if self.graphics_state.float_textures.contains_key(name) {
                    warn!("Texture \"{}\" being redefined", name);
                }

                warn_if_animated_transform!(self, "Texture");
                let ft = make_float_texture(texname, &self.curr_transform[0], &mut tp);

                if let Some(texture) = ft {
                    if self.graphics_state.float_textures_shared {
                        self.graphics_state.float_textures = self.graphics_state.float_textures.to_owned();
                        self.graphics_state.float_textures_shared = false;
                    }


                    let textures = Arc::make_mut(&mut self.graphics_state.float_textures);

                    textures.insert(name.to_owned(), texture);
                }
            },
            "color" | "spectrum" => {
                // Create color texture and store in spectrum_textures
                if self.graphics_state.spectrum_textures.contains_key(name) {
                    warn!("Texture \"{}\" being redefined", name);
                }

                warn_if_animated_transform!(self, "Texture");
                let st = make_spectrum_texture(texname, &self.curr_transform[0], &mut tp);

                if let Some(texture) = st {
                    if self.graphics_state.spectrum_textures_shared {
                        self.graphics_state.spectrum_textures = self.graphics_state.spectrum_textures.to_owned();
                        self.graphics_state.spectrum_textures_shared = false;
                    }


                    let textures = Arc::make_mut(&mut self.graphics_state.spectrum_textures);

                    textures.insert(name.to_owned(), texture);
                }
            },
            _ => error!("Texture type \"{}\" unknown.", ty)
        }
    }

    pub fn material(&mut self,name: &str, params: ParamSet) {
        verify_world!(self, "Material");
        let empty_params = ParamSet::default();
        let mut mp = TextureParams::new(
            &params,
            &empty_params,
            &self.graphics_state.float_textures,
            &self.graphics_state.spectrum_textures);

        let mtl = make_material(name, &mut mp, &self.graphics_state, &self.opts);
        self.graphics_state.current_material = Some(Arc::new(MaterialInstance::new(name, mtl, params)));

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} Material \"{}\" ", "", name, indent=self.indent_count);
        }
    }

    pub fn make_named_material(&mut self, name: &str, params: ParamSet) {
        verify_world!(self, "MakeNamedMaterial");

        let empty_params = ParamSet::default();
        let mut mp = TextureParams::new(
            &params,
            &empty_params,
            &self.graphics_state.float_textures,
            &self.graphics_state.spectrum_textures);

        let mat_name = mp.find_string("type", "");
        warn_if_animated_transform!(self, "MakedNamedMaterial");

        if mat_name.is_empty() {
            error!("No parameter string \"type\" found in MakeNamedMaterial");
        }

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} MakeNamedMaterial \"{}\" ", "", name, indent=self.indent_count);
            params.print(self.indent_count);
            println!();
            return;
        }

        let mtl = make_material(name, &mut mp, &self.graphics_state, &self.opts);

        if self.graphics_state.named_materials.contains_key(name) {
            warn!("Named Material \"{}\" redefined.", name);
        }

        if self.graphics_state.named_materials_shared {
            self.graphics_state.named_materials = self.graphics_state.named_materials.to_owned();
            self.graphics_state.named_materials_shared = false;
        }

        let mat = Arc::new(MaterialInstance::new(&mat_name, mtl, params));
        let materials = Arc::make_mut(&mut self.graphics_state.named_materials);
        materials.insert(name.to_owned(), mat);
    }

    pub fn named_material(&mut self, name: &str) {
        verify_world!(self, "NamedMaterial");
        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} NamedMaterial \"{}\"", "", name, indent=self.indent_count);
            return;
        }

        match self.graphics_state.named_materials.get(name) {
            Some(m) => self.graphics_state.current_material = Some(m.clone()),
            _ => error!("NamedMaterial \"{}\" unknown", name)

        }
    }

    pub fn light_source(&mut self, name: &str, params: &ParamSet) {
        verify_world!(self, "LightSource");
        warn_if_animated_transform!(self, "LightSource");
        let mi = self.graphics_state.create_medium_interface(&self.render_options);
        let lt = make_light(name, params, &self.curr_transform[0], &mi);

        if lt.is_none() {
            error!("LightSource: light type \"{}\" unknown.", name);
        } else {
            self.render_options.lights.push(lt.unwrap());
        }

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} LightSource \"{}\" ", "", name, indent=self.indent_count);
            params.print(self.indent_count);
            println!()
        }
    }

    pub fn area_lightsource(&mut self, name: &str, params: &ParamSet) {
        verify_world!(self, "AreaLightSource");
        self.graphics_state.area_light = name.to_owned();
        self.graphics_state.area_light_params = params.clone();

        if self.opts.cat || self.opts.to_ply {
            print!("{:indent$} AreaLightSource \"{}\" ", "", name, indent=self.indent_count);
            params.print(self.indent_count);
            println!();
        }
    }

    pub fn shape(&mut self, name: &str, params: &ParamSet) {
        verify_world!(self, "Shape");
        let mut prims: Vec<Arc<Primitives>> = Vec::new();
        let mut area_lights = Vec::new();
        //let mut cache = TRANSFORM_CACHE.lock().unwrap();

        if self.opts.cat || self.opts.to_ply && name != "trianglemesh" {
            print!("{:indent$} Shape \"{}\" ", "", name, indent=self.indent_count);
            params.print(self.indent_count);
            println!();
        }

        if !self.curr_transform.is_animated() {
            // Initialize prims and areaLights for static shape

            // Create shapes for shape name
            //let cache = &mut self.transform_cache;
            let curr = &self.curr_transform[0];
            let obj_to_world = self.transform_cache.lookup(curr);
            let inv = Transform::inverse(curr);
            let world_to_obj = self.transform_cache.lookup(&inv);


            let shapes = make_shapes(
                name,
                &obj_to_world,
                &world_to_obj,
                self.graphics_state.reverse_orientation,
                params);

            if shapes.is_empty() {
                return;
            }

            let mtl = self.graphics_state.get_materialfor_shape(params);
            params.report_unused();
            let mi = self.graphics_state.create_medium_interface(&self.render_options);
            prims = Vec::with_capacity(shapes.len());

            for s in shapes.iter() {
                // Possibly create area light for shape
                let mut area = None;

                if !self.graphics_state.area_light.is_empty() {
                    if let Some(a) = make_area_light(&self.graphics_state.area_light, &self.curr_transform[0],
                                                        &mi, &self.graphics_state.area_light_params, s) {
                        area = Some(a.clone());
                        area_lights.push(a)
                    }
                }

                let prim: Primitives = GeometricPrimitive::new(s.clone(), mtl.clone(), area, mi.clone()).into();
                prims.push(Arc::new(prim));
            }
        } else {
            // Initialize prims and areaLights for animated shape

            // Create initial shape or shapes for animated shape
            if self.graphics_state.area_light.is_empty() {
                warn!("Ignoring currently set area light when creating animated shape");
            }

            let tr = Transform::default();
            let identity = self.transform_cache.lookup(&tr);
            let orientation = self.graphics_state.reverse_orientation;
            let shapes = make_shapes(name, &identity, &identity, orientation, params);

            if shapes.is_empty() {
                return;
            }

            // Create GeometricPrimitives for animated shape
            let mtl = self.graphics_state.get_materialfor_shape(params);
            params.report_unused();
            let mi = self.graphics_state.create_medium_interface(&self.render_options);
            prims = Vec::with_capacity(shapes.len());

            shapes.iter().for_each(|s| {
                let prim = GeometricPrimitive::new(s.clone(), mtl.clone(), None, mi.clone());
                prims.push(Arc::new(prim.into()));

            });

            // Create single TransformPrimitive for prims

            // Get animatedObjectToWorld transform for shape
            const_assert_eq!(MAX_TRANSFORMS, 2);
            let obj1 = self.transform_cache.lookup(&self.curr_transform[0]);
            let obj2 = self.transform_cache.lookup(&self.curr_transform[1]);
            let animated_obt_toworld = AnimatedTransform::new(
                obj1,
                obj2,
                self.render_options.transform_start_time,
                self.render_options.transform_end_time);

            if prims.len() > 1 {
                let bvh = BVHAccel::new(prims, 1, SplitMethod::SAH);
                let mut new_prims: Vec<Arc<Primitives>> = Vec::new();
                new_prims.push(Arc::new(bvh.into()));
                prims = new_prims;
            }

            let tprim = TransformedPrimitive::new(prims[0].clone(), animated_obt_toworld);
            prims[0] = Arc::new(tprim.into());
        }

        // Add prims and areaLights to scene or current instance
        let curr_instance = &mut self.render_options.current_instance;
        let primitives = &mut self.render_options.primitives;
        let lights = &mut self.render_options.lights;

        if curr_instance.is_some() {
            if !area_lights.is_empty() {
                warn!("Area lights not supported with object instancing");
            }

            //let c = curr_instance.as_mut().unwrap();

            curr_instance.as_mut().unwrap().append(&mut prims);
        } else {
            primitives.append(&mut prims);

            if !area_lights.is_empty() {
                let mut ls: Vec<Arc<Lights>> = area_lights.iter().map(|l| {
                    let c = (**l).clone();
                    let light: Lights = c.into();

                    Arc::new(light)
                }).collect();

                lights.append(&mut ls)
            }
        }
    }

    pub fn reverse_orientation(&mut self) {
        verify_world!(self, "ReverseOrientation");
        self.graphics_state.reverse_orientation = !self.graphics_state.reverse_orientation;

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ReverseOrientation", "", indent=self.indent_count);
        }
    }

    pub fn object_begin(&mut self, name: &str) {
        verify_world!(self, "ObjectBegin");
        self.attribute_begin();

        if self.render_options.current_instance.is_some() {
            error!("ObjectBegin called inside of instance definition")
        }

        let prims: Vec<Arc<Primitives>> = Vec::new();
        self.render_options.instances.insert(name.to_owned(), prims);
        self.render_options.current_instance = self.render_options.instances.get_mut(name).cloned();

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ObjectBegin \"{}\"", "", name, indent=self.indent_count);
        }
    }

    pub fn object_end(&mut self) {
        verify_world!(self, "ObjectEnd");

        if self.render_options.current_instance.is_none() {
            error!("ObjectEnd called outside of instance definition");
        }

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ObjectEnd", "", indent=self.indent_count);
        }

        self.render_options.current_instance = None;
        self.attribute_end();
        // TODO nObjectInstancesCreated
    }

    pub fn object_instance(&mut self, name: &str) {
        verify_world!(self, "ObjectInstance");

        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} ObjectInstance \"{}\"", "", name, indent=self.indent_count);
            return;
        }

        // Perform object instance error checking
        if self.render_options.current_instance.is_some() {
            error!("ObjectInstance can't be called inside instance definition");
            return;
        }

        if !self.render_options.instances.contains_key(name) {
            error!("Unable to find instance named \"{}\"", name);
            return;
        }


        let instance = self.render_options.instances.get_mut(name).unwrap();

        if instance.is_empty() {
            return;
        }


        // TODO nObjectInstancesCreated
        if instance.len() > 1 {
            // Create aggregate for instance Primitives
            // TODO: check if it's ok to move the instance
            let accel = make_accelerator(name, instance.clone(), &self.render_options.accelerator_params);
            instance.clear();
            instance.push(accel)
        }

        const_assert_eq!(MAX_TRANSFORMS, 2);

        // Create AnimatedInstanceToWorld transform for instance
        let obj1 = self.transform_cache.lookup(&self.curr_transform[0]);
        let obj2 = self.transform_cache.lookup(&self.curr_transform[1]);

        let anim = AnimatedTransform::new(
            obj1,
            obj2,
            self.render_options.transform_start_time,
            self.render_options.transform_end_time);
        let tprim = TransformedPrimitive::new(instance[0].clone(), anim);
        self.render_options.primitives.push(Arc::new(tprim.into()));
    }

    pub fn world_end(&mut self) {
        verify_world!(self, "WorldEnd");

        // Ensure there are no pushed graphics states
        let mut size = self.pushed_graphics_states.len();

        for _ in 0..size {
            warn!("Missing end to attribute_begin()");
            self.pushed_graphics_states.pop();
            self.pushed_transforms.pop();
        }

        size = self.pushed_transforms.len();

        for _ in 0..size {
            warn!("Missing end to transform_begin()");
            self.pushed_transforms.pop();
        }

        // Create scene and render
        if self.opts.cat || self.opts.to_ply {
            println!("{:indent$} WorldEnd", "", indent=self.indent_count);
            return
        }

        let integrator = self.render_options.make_integrator();
        let scene = self.render_options.make_scene();

        // TODO: ProfilerState
        // if scene.is_some() && integrator.is_some() {
        //     integrator.unwrap().render(&scene.unwrap())
        // }

        // Clean up after rendering
        self.graphics_state = GraphicsState::new();
        self.transform_cache.clear();
        self.current_state = APIState::OptionsBlock;
        //TODO: ImageTexture
        self.render_options = Default::default();

        if self.opts.cat && !self.opts.to_ply {
            // TODO: ThreadStats
            if !self.opts.quiet {
                print_stats(std::io::stdout())
            }
        }

        for i in 0..MAX_TRANSFORMS {
            self.curr_transform[i] = Default::default();
        }

        self.active_transform_bits = ALL_TRANSFORM_BITS as u32;
        self.named_coordinate_system.clear();

    }
}

// TODO: stat counters

// Attempt to determine if the ParamSet for a shape may provide a value for
// its material's parameters. Unfortunately, materials don't provide an
// explicit representation of their parameters that we can query and
// cross-reference with the parameter values available from the shape.
//
// Therefore, we'll apply some "heuristics".
fn shape_may_set_materialparameters(ps: &ParamSet) -> bool {
    // Any texture other than one for an alpha mask is almost certainly
    // for a Material (or is unused!).
    if ps.textures.iter().any(|p| p.name != "alpha" && p.name != "shadowalpha") {
        return true;
    }

    // Special case spheres, which are the most common non-mesh primitive.
    if ps.strings.iter().any(|s| s.nvalues == 1 && s.name != "filename" && s.name != "type" && s.name != "scheme") {
        return true;
    }

    // For all other parameter types, if there is a single value of the
    // parameter, assume it may be for the material. This should be valid
    // (if conservative), since no materials currently take array
    // parameters.

    macro_rules! check {
        ($a:ident) => {
            if ps.$a.iter().any(|x| x.nvalues == 1) {
                return true;
            }
        }
    }

    check!(bools);
    check!(ints);
    check!(point2fs);
    check!(vector2fs);
    check!(point3fs);
    check!(vector3fs);
    check!(normals);
    check!(spectra);

    false
}
