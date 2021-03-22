use crate::core::sampler::{Samplers, Sampler};
use crate::core::camera::{Cameras, CameraSample, Camera};
use std::sync::Arc;
use crossbeam::crossbeam_channel::bounded;
use crate::core::geometry::bounds::{Bounds2i, Bounds2f};
use crate::core::geometry::point::{Point3f, Point2f, Point2i};
use crate::core::pbrt::{Float, PI, INFINITY};
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::normal::Normal3f;
use crate::core::medium::MediumInterface;
use crate::core::light::{Lights, LightFlags, is_delta_light, VisibilityTester};
use crate::core::interaction::{InteractionData, Interaction, Interactions, SurfaceInteraction, MediumInteraction};
use crate::core::geometry::ray::Ray;
use crate::core::spectrum::Spectrum;
use crate::core::light::Light;
use crate::core::primitive::Primitive;
use crate::core::medium::{PhaseFunction, Medium};
use crate::core::material::TransportMode;
use crate::core::reflection::BxDFType;
use crate::core::scene::Scene;
use std::hash::{Hash, Hasher};
use std::collections::HashMap;
use log::{error, debug, info, warn};
use crate::{stat_percent, stat_int_distribution};
use crate::core::sampling::Distribution1D;
use bumpalo_herd::{Member, Herd};
use std::fmt::{Result, Formatter, Display};
use crate::core::integrator::{Integrator, Integrators};
use crate::core::lightdistrib::create_light_sample_distribution;
use crate::core::film::Film;
use crate::filters::boxfilter::create_box_filter;
use crate::core::paramset::ParamSet;
use crate::core::lightdistrib::LightDistribution;
use rayon::prelude::*;
use std::path::PathBuf;

stat_percent!("Integrator/Zero-radiance paths", zrp_per_totalpaths);
stat_int_distribution!("Integrator/Path length", path_length);

pub fn init_stats() {
    zrp_per_totalpaths::init();
    path_length::init();
}

fn correct_shading_normal(
    isect: &SurfaceInteraction, wo: &Vector3f,
    wi: &Vector3f, mode: TransportMode) -> Float {
    match mode {
        TransportMode::Importance => {
            let num = wo.abs_dot_norm(&isect.shading.n) * wi.abs_dot_norm(&isect.n);
            let denom = wo.abs_dot_norm(&isect.n) * wi.abs_dot_norm(&isect.shading.n);
            // wi is occasionally perpendicular to isect.shading.n; this is
            // fine, but we don't want to return an infinite or NaN value in
            // that case.
            if denom == 0.0 { return 0.0; }

            num / denom
        },
        _                         => 1.0
    }
}


pub type LightIndexMap<'a> = HashMap<LightKey<'a>, usize>;

pub struct LightKey<'a> {
    lightref: &'a Arc<Lights>
}

impl<'a> LightKey<'a> {
    pub fn new(light: &'a Arc<Lights>) -> Self {
        Self { lightref: light }
    }
}

impl<'a> Eq for LightKey<'a>{}

impl<'a> Hash for LightKey<'a> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        (Arc::as_ptr(self.lightref)).hash(state)
    }
}

impl<'a, 'b> PartialEq<LightKey<'b>> for LightKey<'a> {
    fn eq(&self, other: &LightKey<'b>) -> bool {
        Arc::ptr_eq(self.lightref, other.lightref)
    }
}

#[derive(Clone)]
pub enum CameraOrLight<'a> {
    Camera(&'a Arc<Cameras>),
    Light(&'a Arc<Lights>),
    None
}

impl<'a> Default for CameraOrLight<'a> {
    fn default() -> Self {
        CameraOrLight::None
    }
}

#[derive(Default, Clone)]
pub struct EndpointInteraction<'a> {
    cam_or_light            : CameraOrLight<'a>,
    // Interaction Data
    pub p                   : Point3f,
    pub time                : Float,
    pub p_error             : Vector3f,
    pub wo                  : Vector3f,
    pub n                   : Normal3f,
    pub medium_interface    : Option<MediumInterface>,
}

impl<'a> EndpointInteraction<'a> {
    pub fn from_it_camera(it: &InteractionData, camera: &'a Arc<Cameras>) -> Self {
        Self {
            p: it.p,
            time: it.time,
            p_error: it.p_error,
            wo: it.wo,
            n: it.n,
            medium_interface: it.medium_interface.clone(),
            cam_or_light: CameraOrLight::Camera(camera)
        }
    }

    pub fn from_ray_camera(camera: &'a Arc<Cameras>, ray: &Ray) -> Self {
        let medium_interface = Some(MediumInterface::new(ray.medium.clone()));

        Self {
            p: ray.o,
            time: ray.time,
            medium_interface,
            cam_or_light: CameraOrLight::Camera(camera),
            ..Default::default()
        }
    }

    pub fn from_ray_light(light: &'a Arc<Lights>, ray: &Ray, n: Normal3f) -> Self {
        let medium_interface = Some(MediumInterface::new(ray.medium.clone()));

        Self {
            n,
            p: ray.o,
            time: ray.time,
            medium_interface,
            cam_or_light: CameraOrLight::Light(light),
            ..Default::default()
        }
    }

    pub fn from_it_light(it: &InteractionData, light: &'a Arc<Lights>) -> Self {
        Self {
            p: it.p,
            time: it.time,
            p_error: it.p_error,
            wo: it.wo,
            n: it.n,
            medium_interface: it.medium_interface.clone(),
            cam_or_light: CameraOrLight::Light(light)
        }
    }

    pub fn from_ray(ray: &Ray) -> Self {
        let n = Normal3f::from(-ray.d);
        let p = ray.find_point(1.0);
        let medium_interface = Some(MediumInterface::new(ray.medium.clone()));

        Self {
            p, n,
            time: ray.time,
            medium_interface,
            ..Default::default()
        }
    }
}

impl<'a> Interaction for EndpointInteraction<'a> {
    fn p(&self) -> Point3f {
        self.p
    }

    fn time(&self) -> f32 {
        self.time
    }

    fn p_error(&self) -> Vector3f {
        self.p_error
    }

    fn wo(&self) -> Vector3f {
        self.wo
    }

    fn n(&self) -> Normal3f {
        self.n
    }

    fn medium_interface(&self) -> Option<MediumInterface> {
        self.medium_interface.clone()
    }
}

#[derive(PartialOrd, PartialEq, Eq, Ord, Copy, Clone)]
pub enum VertexType { Camera, Light, Surface, Medium }

pub struct Vertex<'a> {
    vtype   : VertexType,
    beta    : Spectrum,
    delta   : bool,
    pdffwd  : Float,
    pdfrev  : Float,
    intr    : Interactions<'a>
}

impl<'a> Display for Vertex<'a> {
    fn fmt(&self, f: &mut Formatter<'_>) -> Result {
        let mut s = String::from("[Vertex type: ");

        s += match self.vtype {
            VertexType::Camera  => "camera",
            VertexType::Light   => "light",
            VertexType::Surface => "surface",
            VertexType::Medium  => "medium"
        };

        s.push_str(" connectible: ");
        s.push_str(if self.is_connectible() { "true" } else { "false" });
        s.push_str(&format!("\n p: {} ng: {}", self.p(), self.ng()));
        s.push_str(&format!("\n pdfFwd: {} pdfRev: {} beta: {}", self.pdffwd, self.pdfrev, self.beta));

        write!(f, "{}", s)
    }
}

impl<'a> Default for Vertex<'a> {
    fn default() -> Self {
        let intr = MediumInteraction::default();

        Self {
            vtype: VertexType::Medium,
            beta: Spectrum::default(),
            delta: false,
            pdffwd: 0.0,
            pdfrev: 0.0,
            intr: intr.into()
        }
    }
}

impl<'a> Vertex<'a> {
    pub fn new(vtype: VertexType, intr: Interactions<'a>, beta: Spectrum) -> Self {
        Self {
            beta, intr, vtype,
            delta: false,
            pdffwd: 0.0,
            pdfrev: 0.0
        }
    }

    pub fn create_camera_ray(camera: &'a Arc<Cameras>, ray: &Ray, beta: Spectrum) -> Self {
        let intr = EndpointInteraction::from_ray_camera(camera, ray);

        Self::new(VertexType::Camera, intr.into(), beta)
    }

    pub fn create_camera_intr(camera: &'a Arc<Cameras>, it: &InteractionData, beta: Spectrum) -> Self {
        let intr = EndpointInteraction::from_it_camera(it, camera);

        Self::new(VertexType::Camera, intr.into(), beta)
    }

    pub fn create_light_ray(
        light: &'a Arc<Lights>, ray: &Ray,
        nl: Normal3f, le: Spectrum, pdf: Float) -> Self {
        let ei = EndpointInteraction::from_ray_light(light, ray, nl);
        let mut v = Vertex::new(VertexType::Light, ei.into(), le);
        v.pdffwd = pdf;

        v
    }

    pub fn create_light_intr(ei: EndpointInteraction<'a>, beta: Spectrum, pdf: Float) -> Self {
        let mut v = Vertex::new(VertexType::Light, ei.into(), beta);
        v.pdffwd = pdf;

        v
    }

    pub fn create_surface(si: SurfaceInteraction<'a>, beta: Spectrum, pdf: Float, prev: &Self) -> Self {
        let mut v = Vertex::new(VertexType::Surface, si.into(), beta);
        v.pdffwd = prev.convert_density(pdf, &v);

        v
    }

    pub fn create_medium(mi: MediumInteraction, beta: Spectrum, pdf: Float, prev: &Self) -> Self {
        let mut v = Vertex::new(VertexType::Medium, mi.into(), beta);
        v.pdffwd = prev.convert_density(pdf, &v);

        v
    }

    pub fn p(&self) -> Point3f {
        self.intr.p()
    }

    pub fn time(&self) -> Float {
        self.intr.time()
    }

    pub fn ng(&self) -> Normal3f {
        self.intr.n()
    }

    pub fn ns(&self) -> Normal3f {
        match self.intr {
            Interactions::SurfaceInteraction(ref s) => s.shading.n,
            _ => self.intr.n()
        }
    }

    fn is_connectible(&self) -> bool {
        match self.vtype {
            VertexType::Medium => true,
            VertexType::Camera => true,
            VertexType::Light  => {
                if let Interactions::EndpointInteraction(ref ei) = self.intr {
                    if let CameraOrLight::Light(ref l) = ei.cam_or_light {
                        (l.flags() * LightFlags::DeltaDirection as u8) == 0
                    } else {
                        false
                    }
                } else {
                    false
                }
            },
            VertexType::Surface => {
                if let Interactions::SurfaceInteraction(ref si) = self.intr {
                    if let Some(ref bsdf) = si.bsdf {
                        let flags =
                            BxDFType::Diffuse as u8 | BxDFType::Glossy as u8 |
                            BxDFType::Reflection as u8 | BxDFType::Transmission as u8;
                        bsdf.num_components(flags) > 0
                    } else {
                        false
                    }
                } else {
                    false
                }
            }
        }
    }

    fn f(&self, next: &Self, mode: TransportMode) -> Spectrum {
        let mut wi = next.p() - self.p();
        if wi.length_squared() == 0.0 { return Spectrum::new(0.0); }
        wi = wi.normalize();

        match self.vtype {
            VertexType::Surface => {
                if let Interactions::SurfaceInteraction(ref si) = self.intr {
                    let bsdf = si.bsdf.as_ref().unwrap();
                    let ty = BxDFType::All as u8;
                    bsdf.f(&si.wo, &wi, ty) * correct_shading_normal(si, &si.wo, &wi, mode)
                } else {
                    Spectrum::default()
                }
            },
            VertexType::Medium  => {
                if let Interactions::MediumInteraction(ref mi) = self.intr {
                    let phase = mi.phase.as_ref().unwrap();
                    Spectrum::new(phase.p(&mi.wo, &wi))
                } else {
                    Spectrum::default()
                }
            }
            _ => Spectrum::default()
        }
    }

    fn pdf(&self, scene: &Scene, prev: Option<&Self>, next: &Self) -> Float {
        if self.vtype == VertexType::Light {
            return self.pdf_light(scene, next);
        }
        // Compute directions to preceding and next vertex
        let mut wn = next.p() - self.p();
        if wn.length_squared() == 0.0 { return 0.0; }
        wn = wn.normalize();
        let mut wp = Vector3f::default();

        if let Some(pv) = prev {
            wp = pv.p() - self.p();
            if wp.length_squared() == 0.0 { return 0.0; }
            wp = wp.normalize();
        } else {
            assert!(self.vtype == VertexType::Camera);
        }

        // Compute directional density depending on vertex types
        let mut pdf = 0.0;
        let mut unused = 0.0;

        match self.vtype {
            VertexType::Camera => {
                if let Interactions::EndpointInteraction(ref ei) = self.intr {
                    if let CameraOrLight::Camera(cam) = ei.cam_or_light {
                        cam.pdf_we(&ei.spawn_ray(&wn), &mut unused, &mut pdf);
                    }
                }
            },
            VertexType::Surface => {
                if let Interactions::SurfaceInteraction(ref si) = self.intr {
                    let bsdf = si.bsdf.as_ref().unwrap();
                    pdf = bsdf.pdf(&wp, &wn, BxDFType::All as u8);
                }
            },
            VertexType::Medium => {
                if let Interactions::MediumInteraction(ref mi) = self.intr {
                    let phase = mi.phase.as_ref().unwrap();
                    pdf = phase.p(&wp, &wn);
                }
            },
            _ => error!("Vertex::Pdf(): Unimplemented")
        }

        self.convert_density(pdf, next)
    }

    fn get_light(&self) -> Option<Arc<Lights>> {
        match self.vtype {
            VertexType::Light => {
                if let Interactions::EndpointInteraction(ref ei) = self.intr {
                    if let CameraOrLight::Light(light) = ei.cam_or_light {
                        Some(light.clone())
                    } else {
                        None
                    }
                } else {
                    None
                }
            },
            _ => {
                if let Interactions::SurfaceInteraction(ref si) = self.intr {
                    if let Some(ref prim) = si.primitive {
                        prim.get_area_light()
                    } else {
                        None
                    }
                } else {
                    None
                }
            }
        }
    }

    pub fn pdf_light(&self, scene: &Scene, v: &Self) -> Float {
        let mut w = v.p() - self.p();
        let inv_dist2 = 1.0 / w.length_squared();
        w *= inv_dist2.sqrt();

        let mut pdf = if self.is_infinite_light() {
            // Compute planar sampling density for infinite light sources
            let (_, wradius) = scene.wb.bounding_sphere();

            1.0 / (PI * wradius * wradius)
        } else {
            // Get pointer light to the light source at the vertex
            assert!(self.is_light());
            let l = self.get_light();

            assert!(l.is_some());

            // Compute density fir non-infinite light sources
            let light = l.unwrap();
            let mut pdfpos = 0.0;
            let mut pdfdir = 0.0;
            let ray = Ray::new(&self.p(), &w, INFINITY, self.time(), None, None);
            light.pdf_le(&ray, &self.ng(), &mut pdfpos, &mut pdfdir);

            pdfdir * inv_dist2
        };

        if v.is_onsurface() { pdf *= v.ng().abs_dot_vec(&w) }

        pdf
    }

    fn pdf_light_origin(
        &self, scene: &Scene, v: &Self, light_distr: &Arc<Distribution1D>,
        ltdi: &LightIndexMap) -> Float {
        let mut w = v.p() - self.p();
        if w.length_squared() == 0.0 { return 0.0; }
        w = w.normalize();
        if self.is_infinite_light() {
            // Return solid angle density for infinite light sources
            return infinite_light_density(scene, light_distr, ltdi, &w);
        }

        // Return solid angle density for non-infinite light sources
        let mut pdfpos = 0.0;
        let mut pdfdir = 0.0;
        let pdfchoice: Float;

        // Get pointer light to the light source at the vertex
        assert!(self.is_light());
        let l = self.get_light();
        assert!(l.is_some());
        let light = l.unwrap();

        // Compute discrite probability of sampling light pdfchoice
        let index = ltdi.get(&LightKey::new(&light));
        assert!(index.is_some());
        let idx = index.unwrap();
        pdfchoice = light_distr.discrete_pdf(*idx);
        let ray = Ray::new(&self.p(), &w, INFINITY, self.time(), None, None);
        light.pdf_le(&ray, &self.ng(), &mut pdfpos, &mut pdfdir);

        pdfpos * pdfchoice
    }

    fn is_onsurface(&self) -> bool {
        self.ng() != Default::default()
    }

    fn is_light(&self) -> bool {
        if self.vtype == VertexType::Light { return true; }

        if self.vtype == VertexType::Surface {
            if let Interactions::SurfaceInteraction(ref si) = self.intr {
                return si.primitive.as_ref().unwrap().get_area_light().is_some()
            }
        }

        false
    }

    fn is_delta_light(&self) -> bool {
        if self.vtype != VertexType::Light { return false; }

        if let Interactions::EndpointInteraction(ref ei) = self.intr {
            if let CameraOrLight::Light(ref l) = ei.cam_or_light {
                let flags = l.flags();

                return is_delta_light(flags);
            }
        }

        false
    }


    fn is_infinite_light(&self) -> bool {
        if self.vtype != VertexType::Light { return false; }

        if let Interactions::EndpointInteraction(ref ei) = self.intr {
            if let CameraOrLight::Light(ref l) = ei.cam_or_light {
                let flags = l.flags();
                if (flags & LightFlags::Infinite as u8) != 0 ||
                   (flags & LightFlags::DeltaDirection as u8) != 0 {
                    return true;
                }
            } else {
                return true;
            }
        }

        false
    }

    fn le(&self, scene: &Scene, v: &Vertex) -> Spectrum {
        if !self.is_light() { return Spectrum::new(0.0); }
        let mut w = v.p() - self.p();
        if w.length_squared() == 0.0 { return Spectrum::new(0.0); }
        w = w.normalize();

        if self.is_infinite_light() {
            // Return emitted radiance for infinite light sources
            let mut Le = Spectrum::new(0.0);

            for light in scene.infinite_lights.iter() {
                let ray = Ray { o: self.p(), d: -w, ..Default::default() };
                Le += light.le(&ray);
            }

            Le
        } else if let Interactions::SurfaceInteraction(ref si) = self.intr {
            if let Some(ref prim) = si.primitive {
                let l = prim.get_area_light();
                assert!(l.is_some());
                let light = l.unwrap();
                light.l(si, &w)
            } else {
                Spectrum::default()
            }
        } else {
            Spectrum::default()
        }
    }

    fn convert_density(&self, mut pdf: Float, next: &Self) -> Float {
        // Return solid angle density if next is an infinite light
        if next.is_infinite_light() { return pdf; }

        let w = next.p() - self.p();
        if w.length_squared() == 0.0 { return 0.0; }
        let inv_dist2 = 1.0 / w.length_squared();
        if next.is_onsurface() {
            pdf *= next.ng().abs_dot_vec(&(w * inv_dist2.sqrt()));
        }

        pdf * inv_dist2
    }
}

fn infinite_light_density(
    scene: &Scene, light_distr: &Arc<Distribution1D>,
    ltdi: &LightIndexMap, w: &Vector3f) -> Float {
    let mut pdf = 0.0;

    for light in scene.lights.iter() {
        let index = ltdi.get(&LightKey::new(light));
        assert!(index.is_some());
        let idx = index.unwrap();
        pdf += light.pdf_li(&Default::default(), &(-*w)) * light_distr.func[*idx];
    }

    pdf / (light_distr.func_int * light_distr.count() as Float)
}

pub struct BDPTIntegrator {
    sampler                : Box<Samplers>,
    camera                 : Arc<Cameras>,
    max_depth              : isize,
    visualize_strategies   : bool,
    visualize_weights      : bool,
    pbounds                : Bounds2i,
    lsample_strategy       : String
}

impl BDPTIntegrator {
    pub fn new(
        sampler: Box<Samplers>, camera: Arc<Cameras>, max_depth: isize,
        visualize_strategies: bool, visualize_weights: bool,
        pbounds: Bounds2i, lsample_strategy: String) -> Self {
        Self {
            sampler, camera, max_depth, visualize_strategies,
            visualize_weights, pbounds, lsample_strategy
        }
    }
}

impl Integrator for BDPTIntegrator {
    fn render(&mut self, scene: &Scene) {
        let light_distrib =
            create_light_sample_distribution(&self.lsample_strategy, scene).unwrap();

        // Compute a reverse mapping from light pointers to offsets into the
        // scene lights vector (and, equivalently, offsets into
        // lightDistr). Added after book text was finalized; this is critical
        // to reasonable performance with 100s+ of light sources.
        let mut lti: LightIndexMap = HashMap::new();

        for (i, light) in scene.lights.iter().enumerate() {
            lti.insert(LightKey::new(light), i);
        }

        // Partition the image into tiles
        let camera = &self.camera;
        let sampler = &self.sampler;
        let film: Arc<Film> = camera.film();
        let sbounds = film.get_sample_bounds();
        let sextent = sbounds.diagonal();
        let tilesize = 16;
        let nxtiles = (sextent.x + tilesize - 1) / tilesize;
        let nytiles = (sextent.y + tilesize - 1) / tilesize;
        let mut tiles = Vec::with_capacity((nxtiles  * nytiles) as usize);

        for y in 0..nytiles {
            for x in 0..nxtiles {
                tiles.push(Point2i::new(x, y));
            }
        }

        // Allocate buffer for debug visualization
        let buff_count = ((1 + self.max_depth) * (6 + self.max_depth) / 2) as usize;
        let mut weight_films: Vec<Option<Film>> = Vec::with_capacity(buff_count);

        if self.visualize_strategies || self.visualize_weights {
            for _ in 0..buff_count {
                weight_films.push(None);
            }

            for depth in 0..=self.max_depth as usize {
                for s in 0..=(depth + 2) {
                    let t = depth + 2 - s;
                    if t == 0 || (s == 1 && t == 1) { continue; }

                    let filename = PathBuf::from(format!("bdpt_d{}_s{}_t{}.exr", depth, s, t));
                    let cwindow = Bounds2f::new(&Default::default(), &Point2f::new(1.0, 1.0));
                    let filter = create_box_filter(&ParamSet::default());
                    let f = Film::new(
                        &film.full_resolution, &cwindow, filter,
                        film.diagonal * 1000.0, filename, 1.0, INFINITY);
                    weight_films[buffer_index(s, t)] = Some(f);
                }
            }
        }

        // Render and write the output image to disk
        if !scene.lights.is_empty() {
            let (sendt, recvt) = bounded(tiles.len());

            tiles
                .par_iter()
                .for_each(| Point2i { x, y } | {
                    // Render a single tile using BDPT
                    let sendtx = sendt.clone();

                    let seed = y * nxtiles + x;
                    let mut tsampler = Sampler::clone(sampler.as_ref(), seed);
                    let x0 = sbounds.p_min.x + x * tilesize;
                    let x1 = std::cmp::min(x0 + tilesize, sbounds.p_max.x);
                    let y0 = sbounds.p_min.y + y * tilesize;
                    let y1 = std::cmp::min(y0 + tilesize, sbounds.p_max.y);
                    let p1 = Point2i::new(x0, y0);
                    let p2 = Point2i::new(x1, y1);
                    let tile_bounds = Bounds2i::from_points(&p1, &p2);
                    info!("Starting image tile {}", tile_bounds);
                    let mut film_tile = film.get_film_tile(&tile_bounds);

                    let mut herd = Herd::new();

                    for ppixel in &tile_bounds {
                        tsampler.start_pixel(&ppixel);
                        if !self.pbounds.inside_exclusive(&ppixel) { continue; }

                        loop {
                            {
                                let arena = herd.get();
                                // Generate single sample using BDPT
                                let pfilm = Point2f::from(ppixel) + tsampler.get_2d();

                                // Trace the camera subpath
                                let mut cvertices = Vec::with_capacity(self.max_depth as usize + 2);
                                let mut lvertices = Vec::with_capacity(self.max_depth as usize + 1);
                                let ncamera = generate_camera_subpath(
                                    scene, &mut tsampler, &arena, (self.max_depth + 2) as usize,
                                    camera, &pfilm, &mut cvertices);
                                // Get a distribution for sampling the light at the
                                // start of the light subpath. Because the light path
                                // follows multiple bounces, basing the sampling
                                // distribution on any of the vertices of the camera
                                // path is unlikely to be a good strategy. We use the
                                // PowerLightDistribution by default here, which
                                // doesn't use the point passed to it.
                                let light_distr = light_distrib.lookup(&cvertices[0].p());
                                // Now trace the light subpath
                                let nlight = generate_light_subpath(
                                    scene, &mut tsampler, &arena, (self.max_depth + 1) as usize,
                                    cvertices[0].time(), &light_distr, &lti, &mut lvertices);

                                // Execute all BDPT connection strategies
                                let mut L = Spectrum::new(0.0);

                                for t in 1..=(ncamera as isize) {
                                    for s in 0..=(nlight as isize) {
                                        let depth = t + s - 2;

                                        if (s == 1 && t == 1) || depth < 0 || depth > self.max_depth {
                                            continue;
                                        }

                                        // Execute the (s, t) connection strategy and update L
                                        let mut pfilm_new = pfilm;
                                        let mut mis_weight = 0.0;
                                        let lpath = connect_bdpt(
                                            scene, &mut lvertices, &mut cvertices, s as usize,
                                            t as usize, &light_distr, &lti, camera,
                                            &mut tsampler, &mut pfilm_new, Some(&mut mis_weight));
                                        debug!(
                                            "Connect pdpt s: {}, t: {}, Lpath: {}, misWeight: {}",
                                            s, t, lpath, mis_weight);

                                        if self.visualize_strategies || self.visualize_weights {
                                            let mut value = Spectrum::default();

                                            if self.visualize_strategies {
                                                value = if mis_weight == 0.0 { Spectrum::new(0.0) } else { lpath / mis_weight };
                                            }
                                            if self.visualize_weights { value = lpath; }
                                            if let Some(ref f) = &weight_films[buffer_index(s as usize, t as usize)] {
                                                f.add_splat(&pfilm_new, value)
                                            }
                                        }

                                        if t != 1 {
                                            L += lpath;
                                        } else if !lpath.is_black() {
                                            //println!("{:?}", lpath);
                                            film.add_splat(&pfilm_new, lpath)
                                        }
                                    }
                                }
                                debug!("Add film sample pFilm: {}, L: {}, (y: {} )", pfilm, L, L.y());
                                film_tile.add_sample(&pfilm, L, 1.0);

                                if !tsampler.start_next_sample() { break; }
                            }
                            herd.reset()
                        }
                    }

                    sendtx.send(film_tile).unwrap();
                    info!("Finished image tile {}", tile_bounds);
                });

            // Merge film tiles in main thread
            for _ in 0..tiles.len() {
                let mut tile = recvt.recv().unwrap();

                // Merge image tile into film
                film.merge_film_tile(&mut tile);
            }

            film.write_image(1.0 / sampler.samples_per_pixel() as Float).unwrap();

            // Write buffers for debug visualization
            if self.visualize_strategies || self.visualize_weights {
                let inv_sample_count = 1.0 / sampler.samples_per_pixel() as Float;

                for f in weight_films.iter().flatten() {
                    f.write_image(inv_sample_count).unwrap();
                }
            }

        }

    }
}

fn buffer_index(s: usize, t: usize) -> usize {
    let above = s + t - 2;

    s + above * (5 + above) / 2
}

fn g<S: Sampler>(scene: &Scene, sampler: &mut S, v0: &Vertex, v1: &Vertex) -> Spectrum {
    let mut d = v0.p() - v1.p();
    let mut g = 1.0 / d.length_squared();
    d *= g.sqrt();
    if v0.is_onsurface() { g *= v0.ns().abs_dot_vec(&d); }
    if v1.is_onsurface() { g *= v1.ns().abs_dot_vec(&d); }

    let vis = VisibilityTester::new(v0.intr.get_data(), v1.intr.get_data());

    vis.tr(scene, sampler) * g
}

pub fn generate_camera_subpath<'a, S: Sampler>(
    scene: &Scene, sampler: &mut S,
    arena: &Member<'a>, max_depth: usize, camera: &'a Arc<Cameras>,
    pfilm: &Point2f, path: &mut Vec<Vertex<'a>>) -> usize {
    if max_depth == 0 { return 0; }
    // TODO: ProfilePhase
    // Sample initial ray for camera subpath
    let time = sampler.get_1d();
    let plens = sampler.get_2d();
    let csample = CameraSample {
        plens, time,
        pfilm: *pfilm,
    };
    let mut ray = Ray::default();
    let val = camera.generate_ray_differential(&csample, &mut ray);
    let mut beta = Spectrum::new(val);
    ray.scale_differential(1.0 / (sampler.samples_per_pixel() as Float).sqrt());

    // Generate first vertex on camera subpath and start random walk
    let mut pdfpos = 0.0;
    let mut pdfdir = 0.0;
    let vertex = Vertex::create_camera_ray(camera, &ray, beta);
    path.push(vertex);
    camera.pdf_we(&ray, &mut pdfpos, &mut pdfdir);
    debug!(
        "Starting camera subpath. Ray: {}, beta: {}, pdfpos: \
        {}, pdfdir: {}", ray, beta, pdfpos, pdfdir);

    random_walk(
        scene, &ray, sampler, arena, &mut beta,
        pdfdir, max_depth - 1,
        TransportMode::Radiance, path)
        + 1
}

pub fn generate_light_subpath<'a, S: Sampler>(
    scene: &'a Scene, sampler: &mut S, arena: &Member<'a>,
    max_depth: usize, time: Float, light_distr: &Arc<Distribution1D>,
    lti: &LightIndexMap, path: &mut Vec<Vertex<'a>>) -> usize {
    if max_depth == 0 { return 0; }
    // TODO: ProfilePhase
    // Sample initial ray for light subpath
    let mut lightpdf = 0.0;
    let lightnum = light_distr.sample_discrete(sampler.get_1d(), Some(&mut lightpdf), None);
    let light = &scene.lights[lightnum];
    let mut ray = Ray::default();
    let mut nlight = Normal3f::default();
    let mut pdfpos = 0.0;
    let mut pdfdir = 0.0;
    let u2 = sampler.get_2d();
    let u1 = sampler.get_2d();
    let Le = light.sample_le(
        &u1, &u2, time, &mut ray,
        &mut nlight, &mut pdfpos, &mut pdfdir);
    if pdfpos == 0.0 || pdfdir == 0.0 || Le.is_black() { return 0; }

    // Generate first vertex on light subpath and start random walk
    let vertex = Vertex::create_light_ray(light, &ray, nlight, Le, pdfpos * lightpdf);
    path.push(vertex);
    let mut beta = Le * nlight.abs_dot_vec(&ray.d) / (lightpdf * pdfpos * pdfdir);
    debug!(
        "Starting light subpath. Ray: {}, Le {}, beta: {}, \
        pdfPos: {}, pdfDir: {}", ray, Le, beta, pdfpos, pdfdir);
    let nvertices = random_walk(
        scene, &ray, sampler, arena,
        &mut beta, pdfdir, max_depth - 1,
        TransportMode::Importance, path);

    // Correct subpath sampling densities for infinite area lights
    if path[0].is_infinite_light() {
        // Set spatial density of path[1] for infinite area light
        if nvertices > 0 {
            path[1].pdffwd = pdfpos;
            if path[1].is_onsurface() {
                path[1].pdffwd *= ray.d.abs_dot_norm(&path[1].ng());
            }
        }

        // Set spatial density of path[0] for infinite area light
        path[0].pdffwd = infinite_light_density(scene, light_distr, lti, &ray.d);
    }

    nvertices + 1
}


fn random_walk<'a, S: Sampler>(
    scene: &Scene, ray: &Ray, sampler: &mut S,
    arena: &Member<'a>, beta: &mut Spectrum, pdf: Float,
    max_depth: usize, mode: TransportMode, path: &mut Vec<Vertex<'a>>) -> usize {
    let mut ray = ray.clone();
    if max_depth == 0 { return  0; }
    let mut bounces = 0;
    // Declare variables for forward and reverse probability densities
    let mut pdffwd = pdf;
    let mut pdfrev = 0.0;

    loop {
        // Attempt to create the next subpath vertex in path
        let mut mi = MediumInteraction::default();

        debug!(
            "Random walk. Bounces {}, beta {}, pdfFwd {}, pdfRev {}",
            bounces, beta, pdffwd, pdfrev);
        // Trace a ray and sample the medium if any
        let mut isect = SurfaceInteraction::default();
        let fintersection = scene.intersect(&mut ray, &mut isect);
        if let Some(ref med) = ray.medium {
            *beta *= med.sample(&ray, sampler, &mut mi);
        }

        if beta.is_black() { break; }

        let prev = &path[path.len() - 1];
        let previdx = path.len() - 1;

        if let Some(ref phase) = mi.phase {
            // Record medium interaction in path and compute forward density
            let vertex = Vertex::create_medium(mi.clone(), *beta, pdffwd, prev);
            bounces += 1;
            if bounces >= max_depth {
                path.push(vertex);
                break;
            }

            // Sample direction and compute reverse density at preceding vertex
            let mut wi = Vector3f::default();
            let val = phase.sample_p(&(-ray.d), &mut wi, &sampler.get_2d());
            pdffwd = val;
            pdfrev = val;
            ray = mi.spawn_ray(&wi);

            // Compute reverse area density at preceding vertex
            path[previdx].pdfrev = vertex.convert_density(pdfrev, prev);
            path.push(vertex);
        } else {
            // Handle surface interaction for path generation
            if !fintersection {
                // Capture escaped rays when tracing from the camera
                if mode == TransportMode::Radiance {
                    let ei = EndpointInteraction::from_ray(&ray);
                    let vertex = Vertex::create_light_intr(ei, *beta, pdffwd);
                    path.push(vertex);
                    bounces += 1;
                }
                break;
            }

            // Compute scattering functions for mode and skip over medium
            // boundaries
            isect.compute_scattering_functions(&ray, arena, true, mode);
            if isect.bsdf.is_none() {
                ray = isect.spawn_ray(&ray.d);
                continue;
            }

            // Initialize vertex with surface intersection information
            let mut vertex = Vertex::create_surface(isect.clone(), *beta, pdffwd, prev);
            bounces += 1;

            if bounces >= max_depth {
                path.push(vertex);
                break
            } else {
                // Sample BSDF at current vertex and compute reverse probability
                let mut wi = Vector3f::default();
                let wo = isect.wo;
                let mut ty = 0;
                let flags = BxDFType::All as u8;
                let bsdf = isect.bsdf.as_ref().unwrap();
                let f = bsdf.sample_f(
                    &wo, &mut wi, &sampler.get_2d(),
                    &mut pdffwd, flags, &mut ty);
                debug!("Random walk sampled dir {} f: {}, pdfFwd: {}", wi, f, pdffwd);
                if f.is_black() || pdffwd == 0.0 {
                    path.push(vertex);
                    break;
                }

                *beta *= f * wi.abs_dot_norm(&isect.shading.n) / pdffwd;
                debug!("Random walk beta now {}", beta);
                pdfrev = bsdf.pdf(&wi, &wo, BxDFType::All as u8);
                if (ty & BxDFType::Specular as u8) != 0 {
                    vertex.delta = true;
                    pdfrev = 0.0;
                    pdffwd = 0.0
                }
                *beta *= correct_shading_normal(&isect, &wo, &wi, mode);
                debug!("Random walk beta after shading normal correction {}", beta);
                ray = isect.spawn_ray(&wi);
            }

            // Compute reverse area density at preceding vertex
            path[previdx].pdfrev = vertex.convert_density(pdfrev, prev);
            path.push(vertex);
        }

    }

    bounces
}

fn mis_weight<'a>(
    scene: &Scene, lvertices: &mut [Vertex<'a>],
    cvertices: &mut [Vertex<'a>], sampled: Vertex<'a>,
    s: usize, t: usize, lightpdf: &Arc<Distribution1D>,
    lti: &LightIndexMap) -> Float {
    if s + t == 2 { return 1.0; }
    let mut sumri = 0.0;
    //Define helper function remap0 that deals with Dirac dela functions
    let remap0 = |f: Float| -> Float { if f!= 0.0 { f } else { 1.0 } };

    // Temporarily update vertex properties for current strategy

    // Look up connection vertices and their predecessors
    //let(c, d) = cvertices.split_at_mut(t - 1);
    let mut qs: Option<Vertex>  = if s > 0 {
        let vert = &lvertices[s - 1];
        let intr: Interactions = match vert.intr {
            Interactions::EndpointInteraction(ref e) => e.clone().into(),
            Interactions::MediumInteraction(ref m) => m.clone().into(),
            Interactions::SurfaceInteraction(ref si) => si.clone().into(),
            _ => unreachable!()
        };
        let v = Vertex {
            vtype: vert.vtype,
            beta: vert.beta,
            delta: vert.delta,
            pdffwd: vert.pdffwd,
            pdfrev: vert.pdfrev,
            intr,
        };
        Some(v)

    } else {
        None
    };

    let mut pt: Option<Vertex>  = if t > 0 {
        let vert = &cvertices[t - 1];
        let intr: Interactions = match vert.intr {
            Interactions::EndpointInteraction(ref e) => e.clone().into(),
            Interactions::MediumInteraction(ref m) => m.clone().into(),
            Interactions::SurfaceInteraction(ref si) => si.clone().into(),
            _ => unreachable!()
        };

        let v = Vertex {
            vtype: vert.vtype,
            beta: vert.beta,
            delta: vert.delta,
            pdffwd: vert.pdffwd,
            pdfrev: vert.pdfrev,
            intr,
        };
        Some(v)

    } else {
        None
    };


    let mut qsminus = if s > 1 { Some(&mut lvertices[s - 2]) } else { None };
    let mut ptminus = if t > 1 { Some(&mut cvertices[t - 2]) } else { None };

    // Update sampled vertex for s=1 or t=1 strategy
    if s == 1 {
        qs = Some(sampled);
    } else if t == 1 {
        pt = Some(sampled);
    }

    // Mark Connection vertices as non-degenerate
    if let Some(ref mut v) = pt { v.delta = false; }
    if let Some(ref mut v) = qs { v.delta = false; }

    // Update reverse density of vertex pt t-1
    if let Some(ref mut v) = pt {
        v.pdfrev = if s > 0 {
            if let Some(ref qsminus_ref) = qsminus {
                qs.as_ref().unwrap().pdf(scene, Some(qsminus_ref), v)
            } else {
                qs.as_ref().unwrap().pdf(scene, None, v)
            }

        } else if let Some(ref ptminus_ref) = ptminus {
            v.pdf_light_origin(scene, ptminus_ref, lightpdf, lti)
        } else {
            v.pdfrev
        };
    }

    // update reverse density of pt t-2
    let mut ptminus_prev_pdfrev = 0.0;
    if let Some(ref mut v) = ptminus {
        ptminus_prev_pdfrev = v.pdfrev;
        v.pdfrev = if s > 0 {
            if let Some(ref qs_ref) = qs {
                pt.as_ref().unwrap().pdf(scene, Some(qs_ref), v)
            } else {
                pt.as_ref().unwrap().pdf(scene, None, v)
            }
        } else {
            pt.as_ref().unwrap().pdf_light(scene, v)
        };
    }

    // Update reverse density of vertices pq s-1 and pq s-2
    if let Some(ref mut v) = qs {
        if let Some(ptminis_ref) = ptminus {
            v.pdfrev = pt.as_ref().unwrap().pdf(scene, Some(ptminis_ref), v);
        } else {
            v.pdfrev = pt.as_ref().unwrap().pdf(scene, None, v);
        }
    }

    let mut qsminus_prev_pdfrev = 0.0;
    if let Some(ref mut v) = qsminus {
        qsminus_prev_pdfrev = v.pdfrev;

        v.pdfrev = if let Some(ref pt_ref) = pt {
            qs.as_ref().unwrap().pdf(scene, Some(pt_ref), v)
        } else {
            qs.as_ref().unwrap().pdf(scene, None, v)
        }
    }

    // Consider hypothetical connection strategies along the camera subpath
    let mut ri = 1.0;
    let mut i = t - 1;

    while i > 0 {
        let mut v1 = &cvertices[i];

        if i == t - 1 {
            if let Some(ref v) = pt {
                v1 = v
            }
        }

        ri *= remap0(v1.pdfrev) / remap0(v1.pdffwd);

        if !v1.delta && !cvertices[i - 1].delta {
            sumri += ri
        }

        i -= 1;
    }

    // Consider hypothetical connection strategies along the light subpath
    ri = 1.0;
    let mut i = s as isize - 1;

    while i >= 0 {
        let mut v1 = &lvertices[i as usize];

        if i == s as isize - 1 {
            if let Some(ref v) = qs {
                v1 = v;
            }
        }

        ri *= remap0(v1.pdfrev) / remap0(v1.pdffwd);

        let delta_light_vertex = if i > 0 {
            lvertices[(i - 1) as usize].delta
        } else {
            lvertices[0].is_delta_light()
        };

        if !v1.delta && !delta_light_vertex {
            sumri += ri
        }

        i -= 1;
    }

    // Reset original vertex data
    if s > 1 {
        lvertices[s - 2].pdfrev = qsminus_prev_pdfrev;
    }

    if t > 1 {
        cvertices[t - 2].pdfrev = ptminus_prev_pdfrev;
    }

    1.0 / (1.0 + sumri)
}

pub fn connect_bdpt<'a, S: Sampler>(
    scene: &'a Scene, lvertices: &mut [Vertex<'a>],
    cvertices: &mut [Vertex<'a>], s: usize, t: usize,
    light_distr: &Arc<Distribution1D>, lti: &LightIndexMap,
    camera: &'a Arc<Cameras>, sampler: &mut S,
    praster: &mut Point2f, mis_weight_ptr: Option<&mut Float>) -> Spectrum {
    // TODO: ProfilePhase
    let mut L = Spectrum::new(0.0);
    // Ignore invalid connections related to infinite area lights
    if t > 1 &&s != 0 && cvertices[t - 1].vtype == VertexType::Light {
        return Spectrum::new(0.0);
    }

    let mut sampled = Vertex::default();

    // Perform connection and write contribution to L
    if s == 0 {
        // Interpret the camera subpath as a complete path
        let pt = &cvertices[t - 1];
        if pt.is_light() { L = pt.le(scene, &cvertices[t - 2]) * pt.beta }
        assert!(!L.has_nans())
    } else if t == 1 {
        // Sample point on the camera and connect it to the light subpath
        let qs = &lvertices[s - 1];
        if qs.is_connectible() {
            let mut vis = VisibilityTester::default();
            let mut wi = Vector3f::default();
            let mut pdf = 0.0;
            let Wi = camera.sample_wi(
                &qs.intr.get_data(), &sampler.get_2d(), &mut wi,
                &mut pdf, praster, &mut vis);

           if pdf > 0.0 && !Wi.is_black() {
               // Initialize dynamically sampled vertex and L for t=1 case
               sampled = Vertex::create_camera_intr(camera, &vis.p1(), Wi / pdf);
               L = qs.beta * qs.f(&sampled, TransportMode::Importance) * sampled.beta;
               if qs.is_onsurface() { L *= wi.abs_dot_norm(&qs.ns()); }
               assert!(!L.has_nans());
               // Only check visibility after we know that the path would
               // make a non-zero contribution.
               if !L.is_black() { L *= vis.tr(scene, sampler); }
           }
        }
    } else if s == 1 {
        // Sample a point on a light and connect it to the camera subpath
        let pt = &cvertices[t - 1];
        if pt.is_connectible() {
            let mut lightpdf = 0.0;
            let mut vis = VisibilityTester::default();
            let mut wi = Vector3f::default();
            let mut pdf = 0.0;
            let lightnum = light_distr.sample_discrete(sampler.get_1d(), Some(&mut lightpdf), None);
            let light = &scene.lights[lightnum];
            let lweight = light.sample_li(&pt.intr.get_data(), &sampler.get_2d(), &mut wi, &mut pdf, &mut vis);

            if pdf > 0.0 && !lweight.is_black() {
                let ei = EndpointInteraction::from_it_light(&vis.p1(), light);
                sampled = Vertex::create_light_intr(ei, lweight / (pdf * lightpdf), 0.0);
                sampled.pdffwd = sampled.pdf_light_origin(scene, pt, light_distr, lti);
                L = pt.beta * pt.f(&sampled, TransportMode::Radiance) * sampled.beta;
                if pt.is_onsurface() { L *= wi.abs_dot_norm(&pt.ns()); }
                // Only check visibility if the path would carry radiance
                if !L.is_black() { L *= vis.tr(scene, sampler) }
            }
        }
    } else {
        // Handle all other bidirectional connection cases
        let qs = &lvertices[s - 1];
        let pt = &cvertices[t - 1];

        if qs.is_connectible() && pt.is_connectible() {
            let qsf = qs.f(pt, TransportMode::Importance);
            let ptf = pt.f(qs, TransportMode::Radiance);
            L = qs.beta * qsf * ptf * pt.beta;
            let g = g(scene, sampler, qs, pt);
            debug!(
                "General connect s: {}, t: {}, qs: {}, pt: {}, \
                qs.f(qt): {}, pt.f(qs): {}, G: {}, dist^2: {}",
                s, t, qs, pt, qsf, ptf, g, qs.p().distance_squared(&pt.p()));

            if !L.is_black() { L *= g; }
        }
    }

    zrp_per_totalpaths::inc_den();
    if L.is_black() { zrp_per_totalpaths::inc_num(); }
    path_length::report_value((s + t - 2) as u64);

    // Compute MIS weight for connection strategy
    let mis_weight = if L.is_black() {
        0.0
    } else {
        mis_weight(scene, lvertices, cvertices, sampled, s, t, &light_distr, lti)
    };

    debug!("MIS weight for (s, t) = ( {}, {} ) connection: {}", s, t, mis_weight);
    assert!(!mis_weight.is_nan());
    L *= mis_weight;

    if let Some(ptr) = mis_weight_ptr {
        *ptr = mis_weight
    }

    L
}

pub fn create_bdpt_intergrator(
    params: &ParamSet, sampler: Box<Samplers>,
    camera: Arc<Cameras>) -> Option<Integrators> {
    let mut maxdepth = params.find_one_int("maxdepth", 5);
    let vis_strat = params.find_one_bool("visualizestrategies", false);
    let vis_weights = params.find_one_bool("visualizeweights", false);

    if (vis_strat || vis_weights) && maxdepth > 5 {
        warn!(
            "visualizestrategies/visualizeweights was enabled, \
            limiting maxdepth to 5");
        maxdepth = 5;
    }

    let mut np = 0;
    let pb = params.find_int("pixelbounds", &mut np);
    let mut pbounds = camera.film().get_sample_bounds();

    if let Some(p) = pb {
        if p.len() != 4 {
            error!("Expected four values for \"pixelbounds\" parameter. Got {}.", np);
        } else {
            let b = Bounds2i::from_points(
                &Point2i::new(p[0], p[2]),
                &Point2i::new(p[1], p[3]));
            pbounds = b.intersect(&pbounds);

            if pbounds.area() == 0 {
                error!("Degenerate \"pixelbounds\" specified.");
            }
        }
    }

    let lstrategy = params.find_one_string("lightsamplestrategy", "power".to_owned());

    Some(
        BDPTIntegrator::new(
            sampler, camera, maxdepth,
            vis_strat, vis_weights,
            pbounds, lstrategy).into())
}