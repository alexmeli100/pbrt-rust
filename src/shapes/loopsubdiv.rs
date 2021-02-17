use crate::core::geometry::point::Point3f;
use crate::core::pbrt::{Float, PI};
use smallvec::{smallvec, SmallVec};
use std::hash::{Hash, Hasher};
use crate::core::transform::Transform;
use crate::core::shape::Shapes;
use std::sync::Arc;
use std::collections::{HashSet, HashMap};
use crate::core::geometry::vector::Vector3f;
use crate::core::geometry::normal::Normal3f;
use crate::shapes::triangle::create_trianglemesh;
use crate::core::paramset::ParamSet;
use log::error;

macro_rules! next {
    ($i:expr) => {{
        (($i + 1) % 3) as usize
    }}
}

macro_rules! prev {
    ($i:expr) => {{
        (($i + 2) % 3) as usize
    }}
}

struct SDVertex {
    p           : Point3f,
    start_face  : isize,
    child       : isize,
    regular     : bool,
    boundary    : bool
}

impl SDVertex {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self::default()
    }

    pub fn valence(&self, vi: isize, faces: &[SDFace]) -> isize {
        let mut fi = self.start_face;

        if !self.boundary {
            // Compute valence of interior vertex
            let mut nf = 1;

            loop {
                if fi == -1 { break; }
                fi = faces[fi as usize].next_face(vi);
                if fi != self.start_face { nf += 1; } else { break; }
            }

            nf
        } else {
            // Compute valence of boundary vertex
            let mut nf = 1;

            loop {
                if fi == -1 { break; }
                fi = faces[fi as usize].next_face(vi);
                if fi == -1 { break; } else { nf +=1; }
            }

            fi = self.start_face;

            loop {
                if fi == -1 { break; }
                fi = faces[fi as usize].prev_face(vi);
                if fi == -1 { break; } else { nf += 1; }
            }

            nf + 1
        }
    }

    pub fn one_ring(&self, p: &mut [Point3f], vi: isize, verts: &[SDVertex], faces: &[SDFace]) {
        if !self.boundary {
            // Get one-ring vertices for interior vertex
            let mut face = self.start_face;
            let mut pi = 0;

            loop {
                let idx = faces[face as usize].next_vert(vi);
                p[pi] = verts[idx as usize].p;
                pi += 1;
                face = faces[face as usize].next_face(vi);

                if face == self.start_face { break }
            }
        } else {
            // Get one-ring vertices for boundary vertex
            let mut face = self.start_face;
            let mut f2: isize;
            let mut pi = 0;

            loop {
                f2 = faces[face as usize].next_face(vi);
                if f2 == -1 { break } else { face = f2; }
            }

            let idx = faces[face as usize].next_vert(vi);
            p[pi] = verts[idx as usize].p;
            pi += 1;

            loop {
                let idx = faces[face as usize].prev_vert(vi);
                p[pi] = verts[idx as usize].p;
                pi += 1;
                face = faces[face as usize].prev_face(vi);

                if face == -1 { break; }
            }
        }
    }
}

impl Default for SDVertex {
    fn default() -> Self {
        Self {
            p           : Default::default(),
            start_face  : -1,
            child       : -1,
            regular     : false,
            boundary    : false
        }
    }
}

impl From<Point3f> for SDVertex {
    fn from(p: Point3f) -> Self {
        Self {
            p,
            start_face  : -1,
            child       : -1,
            regular     : false,
            boundary    : false
        }
    }
}

struct SDFace {
    v       : [isize; 3],
    f       : [isize; 3],
    children: [isize; 4]
}

impl SDFace {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self::default()
    }

    fn vnum(&self, vert: isize) -> isize {
        for i in 0..3 {
            if self.v[i] == vert { return i as isize; }
        }

        panic!("Basic logic error in SDFace::vum()");
    }

    pub fn next_face(&self, vert: isize) -> isize {
        self.f[self.vnum(vert) as usize]
    }

    pub fn prev_face(&self, vert: isize) -> isize {
        self.f[prev!(self.vnum(vert))]
    }

    pub fn next_vert(&self, vert: isize) -> isize {
        self.v[next!(self.vnum(vert))]
    }

    pub fn prev_vert(&self, vert: isize) -> isize {
        self.v[prev!(self.vnum(vert))]
    }

    pub fn other_vert(&self, v0: isize, v1: isize) -> isize {
        for i in 0..3 {
            if self.v[i] != v0 && self.v[i] != v1 { return self.v[i]; }
        }

        panic!("Basic logic error in SDVertex::other_vert()")
    }
}

impl Default for SDFace {
    fn default() -> Self {
        Self {
            v: [-1, -1, -1],
            f: [-1, -1, -1],
            children: [-1, -1, -1, -1]
        }
    }
}

struct SDEdge {
    v           : [isize; 2],
    f           : [isize; 2],
    f0_edgenum  : isize
}

impl SDEdge {
    pub fn new(v0: isize, v1: isize) -> Self {
        let x = std::cmp::min(v0, v1);
        let y = std::cmp::max(v0, v1);

        Self {
            v: [x, y],
            f: [-1, -1],
            f0_edgenum: -1
        }
    }
}

impl Eq for SDEdge{}

impl PartialEq for SDEdge {
    fn eq(&self, other: &Self) -> bool {
        if self.v[0] == other.v[0] {
            self.v[1] == other.v[1]
        } else {
            false
        }
    }
}

impl Hash for SDEdge {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.v[0].hash(state);
        self.v[1].hash(state);
    }
}

fn beta(valence: isize) -> Float {
    if valence == 3 {
        3.0 / 16.0
    } else {
        3.0 / (8.0 * valence as Float)
    }
}

fn loop_gamma(valence: isize) -> Float {
    1.0 / (valence as Float + 3.0 / (8.0 * beta(valence)))
}

pub fn loop_subdivide(
    o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool,
    nlevels: usize, vertex_indices: &[isize], p: &[Point3f]) -> Vec<Arc<Shapes>> {
    let mut verts = Vec::with_capacity(p.len());
    for pi in p {
        verts.push(SDVertex::from(*pi));
    }

    let nfaces = vertex_indices.len() / 3;
    let mut faces = Vec::with_capacity(nfaces);
    for _ in 0..nfaces {
        faces.push(SDFace::default());
    }

    // set face to vertex pointers
    for i in 0..nfaces {
        let fi = i as isize;
        for j in 0..3 {
            let vi = vertex_indices[i * 3 + j];
            faces[fi as usize].v[j] = vi;
            verts[vi as usize].start_face = fi
        }
    }

    // Set neighbour pointers in faces
    let mut edges = HashSet::new();

    for i in 0..nfaces {
        let fi = i as isize;
        for edgenum in 0..3 {
            // update neighbour pointer for edgenum
            let v0 = edgenum;
            let v1 = next!(edgenum as isize);
            let mut e = SDEdge::new(faces[i].v[v0], faces[i].v[v1]);

            if !edges.contains(&e) {
                // handle new edge
                e.f[0] = fi;
                e.f0_edgenum = edgenum as isize;
                edges.insert(e);
            } else {
                // Handle previously seen edge
                let eopt = edges.take(&e);

                if let Some(edge) = eopt {
                    let f = &mut faces[edge.f[0] as usize];
                    f.f[edge.f0_edgenum as usize] = fi;
                    faces[fi as usize].f[edgenum] = edge.f[0]
                }
            }
        }
    }

    // Finish vertex initialization
    for (i, vert) in verts.iter_mut().enumerate() {
        let start_face = vert.start_face;
        let mut fi = start_face;

        loop {
            fi = faces[fi as usize].next_face(i as isize);
            if fi == -1 || fi == start_face { break; }
        }

        vert.boundary = fi == -1;
        let valence = vert.valence(i as isize, &faces);
        vert.regular = if !vert.boundary && valence == 6 {
            true
        } else {
            vert.boundary && valence == 4
        };
    }

    // Refine LoopSubDiv intro triangles
    for i in 0..nlevels {
        // Update faces and verts for next subdivisopn
        let mut new_faces = Vec::new();
        let mut new_vertices = Vec::new();

        // Allocate next level of children in mesh tree
        for vert in verts.iter_mut() {
            let ci = new_vertices.len();
            vert.child = ci as isize;
            new_vertices.push(SDVertex::default());
            new_vertices[ci].regular = vert.regular;
            new_vertices[ci].boundary = vert.boundary
        }

        for face in faces.iter_mut() {
            for k in 0..4 {
                let ci = new_faces.len();
                new_faces.push(SDFace::default());
                face.children[k] = ci as isize;
            }
        }

        // Update vertex positions and create new edge vertices

        // Update vertex positions for even vertices
        for vi in 0..verts.len() {
            let ci = verts[vi].child as usize;
            let child = &mut new_vertices[ci];

            if !verts[vi].boundary {
                // Apply one-ring rule for even vertex
                if verts[vi].regular {
                    child.p = weight_one_ring(
                        &verts[vi], vi as isize, &verts, &faces, 1.0 / 16.0);
                } else {
                    child.p = weight_one_ring(
                        &verts[vi], vi as isize, &verts, &faces,
                        beta(verts[vi].valence(vi as isize, &faces)));

                }
            } else {
                // Apply boundary rule for even vertex
                child.p = weight_boundary(
                    &verts[i], vi as isize, &verts, &faces, 1.0 / 8.0);
            }
        }

        // Compute new odd edge vertices
        let mut edge_verts = HashMap::new();

        for fi in 0..faces.len() {
            for k in 0..3 {
                // Compute odd vertex on kth edge
                let edge = SDEdge::new(
                    faces[fi].v[k],
                    faces[fi].v[next!(k)]);
                let contains_edge = edge_verts.contains_key(&edge);

                if !contains_edge {
                    // Create and initialize new odd vertex
                    let nvi = new_vertices.len();
                    new_vertices.push(Default::default());
                    let vert = &mut new_vertices[nvi];
                    vert.regular = true;
                    vert.boundary = faces[fi].f[k] == -1;
                    vert.start_face = faces[fi].children[3];

                    // Apply edge rules to compute new vertex position
                    if vert.boundary {
                        vert.p = verts[edge.v[0] as usize].p * 0.5;
                        vert.p += verts[edge.v[1] as usize].p * 0.5;
                    } else {
                        vert.p = verts[edge.v[0] as usize].p * (3.0 / 8.0);
                        vert.p += verts[edge.v[1] as usize].p * (3.0 / 8.0);
                        let vi = faces[fi].other_vert(edge.v[0], edge.v[1]);
                        vert.p += verts[vi as usize].p * (1.0 / 8.0);
                        let vi = faces[faces[fi].f[k] as usize]
                            .other_vert(edge.v[0], edge.v[1]);
                        vert.p += verts[vi as usize].p * (1.0 / 8.0)
                    }

                    edge_verts.insert(edge, nvi as isize);
                }
            }
        }

        // Update mesh topology

        // Update even vertex face pointers
        for (vi, vert) in verts.iter_mut().enumerate() {
            let mut ci = -1;
            let mut face_child = -1;

            let start_face = vert.start_face as usize;
            let face = &mut faces[start_face];
            let vnum = face.vnum(vi as isize) as usize;
            ci = vert.child;
            face_child = face.children[vnum];

            if ci != -1 {
                new_vertices[ci as usize].start_face = face_child;
            }
        }

        // Update face neightbor pointers
        for i in 0..faces.len() {
            let face = &faces[i];
            for j in 0..3 {
                // Update children f pointer for siblings
                let ci = face.children[3] as usize;
                new_faces[ci].f[j] = face.children[next!(j as isize)];
                let ci = face.children[j] as usize;
                new_faces[ci].f[next!(j as isize)] = face.children[3];

                // Update children f pointers for neighbor children
                let mut fi2 = face.f[j];

                if fi2 != -1 {
                    let f2 = &faces[fi2 as usize];
                    let ci2 = f2.children[f2.vnum(face.v[j]) as usize];
                    let ci = face.children[j] as usize;
                    new_faces[ci].f[j] = ci2
                } else {
                    let ci = face.children[j] as usize;
                    new_faces[ci].f[j] = -1
                }

                fi2 = face.f[prev!(j as isize)];

                if fi2 != -1 {
                    let f2 = &faces[fi2 as usize];
                    let ci2 = f2.children[f2.vnum(face.v[j]) as usize];
                    let ci = face.children[j] as usize;
                    new_faces[ci].f[prev!(j as isize)] = ci2;
                } else {
                    let ci = face.children[j] as usize;
                    new_faces[ci].f[prev!(j as isize)] = -1;
                }
            }
        }

        // Update face vertex pointers
        for face in &faces {
            let mut nvi = -1;
            for j in 0..3 {
                // Update child vertex pointer to new even vertex
                let ci = face.children[j] as usize;
                let vi = face.v[j] as usize;
                let vertex = &verts[vi];
                new_faces[ci].v[j] = vertex.child;

                // Update child vertex pointer to odd vertex;
                let k = SDEdge::new(face.v[j], face.v[next!(j as isize)]);
                let vopt = edge_verts.get(&k);
                if let Some(ptr) = vopt {
                    nvi = *ptr;
                }

                new_faces[ci].v[next!(j as isize)] = nvi;
                let ci = face.children[next!(j as isize)] as usize;
                new_faces[ci].v[j] = nvi;
                let ci = face.children[3] as usize;
                new_faces[ci].v[j] = nvi;
            }
        }

        // prepare for next level of subdivision
        faces = new_faces.split_off(0);
        verts = new_vertices.split_off(0);
    }

    // Push vertices to limit surface
    let mut plimit = Vec::with_capacity(verts.len());

    for i in 0..verts.len() {
        let v = &verts[i];

        if v.boundary {
            plimit.push(weight_boundary(
                v, i as isize, &verts,
                &faces, 1.0/ 5.0));
        } else {
            plimit.push(weight_one_ring(
                v, i as isize, &verts, &faces,
                loop_gamma(v.valence(i as isize, &faces))));
        }
    }

    for (i, v) in verts.iter_mut().enumerate() {
        v.p = plimit[i];
    }

    // Compute vertex tangents on limit surface
    let mut ns = Vec::with_capacity(verts.len());
    let mut pring = vec![Point3f::default(); 16];

    for (vi, vertex) in verts.iter().enumerate() {
        let mut S = Vector3f::default();
        let mut T = Vector3f::default();
        let valence = vertex.valence(vi as isize, &faces);

        if valence > pring.len() as isize {
            pring.resize_with(valence as usize, Default::default);
        }

        vertex.one_ring(&mut pring, vi as isize, &verts, &faces);

        if !vertex.boundary {
            // Compute tangent of interior face
            for (i, p) in pring.iter().enumerate() {
                S += Vector3f::from(*p) * (2.0 * PI * i as Float / valence as Float).cos();
                T += Vector3f::from(*p) * (2.0 * PI * i as Float / valence as Float).sin();
            }
        } else {
            // Compute tangents of boundary face
            S = pring[(valence - 1) as usize] - pring[0];
            if valence == 2 {
                T = pring[0] + pring[1] - vertex.p * 2.0;
            } else if valence == 3 {
                T = pring[1] - vertex.p;
            } else if valence == 4 {
                T = Vector3f::from(
                    pring[0] * -1.0 +
                    pring[1] * 2.0 +
                    pring[2] * 2.0 +
                    pring[3] * -1.0 +
                    vertex.p * -2.0
                )
            } else {
                let theta = PI / (valence - 1) as Float;
                T = Vector3f::from((pring[0] + pring[(valence - 1) as usize]) * theta.sin());

                for (k, p) in pring
                    .iter()
                    .enumerate()
                    .take((valence - 1) as usize)
                    .skip(1){
                    let wt = (2.0 * theta.cos() - 2.0) * (k as Float * theta).sin();
                    T += Vector3f::from(*p * wt);
                }
                T = -T;
            }
        }

        ns.push(Normal3f::from(S.cross(&T)));
    }

    // Create triangle mesh from subdivision mesh
    let ntris = faces.len();
    let mut vindices = Vec::with_capacity(3 * ntris);
    let totverts = verts.len();

    for face in faces.iter() {
        for j in 0..3 {
            vindices.push(face.v[j] as usize)
        }
    }

    create_trianglemesh(
        o2w, w2o, reverse_orientation,
        ntris, vindices, totverts,
        plimit, Vec::new(), ns, Vec::new(), None, None)
}

fn weight_one_ring(
    vert: &SDVertex, vi: isize, verts: &[SDVertex],
    faces: &[SDFace], beta: Float) -> Point3f {
    // Put vert one-ring in pRing
    let valence = vert.valence(vi, faces) as usize;
    let mut pring: SmallVec<[Point3f; 256]> = smallvec![Default::default(); valence];
    vert.one_ring(&mut pring, vi, verts, faces);
    let mut p = vert.p * (1.0 - valence as Float * beta);

    for i in 0..valence {
        p += pring[i] * beta;
    }

    p
}

fn weight_boundary(
    vert: &SDVertex, vi: isize, verts: &[SDVertex],
    faces: &[SDFace], beta: Float) -> Point3f {
    // Put vert one-ring in pRing
    let valence = vert.valence(vi, faces) as usize;
    let mut pring: SmallVec<[Point3f; 256]> = smallvec![Default::default(); valence];
    vert.one_ring(&mut pring, vi, verts, faces);
    let mut p = vert.p * (1.0 - 2.0 * beta);
    p += pring[0] * beta;
    p += pring[valence - 1] * beta;

    p
}

pub fn create_loop_dubdiv(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool,
    params: &ParamSet) -> Vec<Arc<Shapes>> {
    let n = params.find_one_int("nlevels", 3);
    let nlevels = params.find_one_int("levels", n) as usize;
    let mut nps = 0usize;
    let mut nindices = 0;
    let vindices = params.find_int("indices", &mut nindices);
    let p = params.find_point3f("p", &mut nps);

    if vindices.is_none() {
        error!("Vertex indices \"indices\" not provided for LoopSubdiv shape.");
        return vec![];
    }
    if p.is_none() {
        error!("Vextex positions \"P\" not provided for LoopSubdiv shape.");
        return vec![];
    }

    let _scheme = params.find_one_string("scheme", "loop".to_owned());

    loop_subdivide(o2w, w2o, reverse_orientation, nlevels, &vindices.unwrap(), &p.unwrap())
}
