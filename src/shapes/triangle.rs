use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::geometry::normal::Normal3f;
use crate::core::geometry::vector::{Vector3f, vec3_coordinate_system};
use std::sync::Arc;
use crate::core::texture::{Texture, TextureFloat};
use crate::core::pbrt::{Float, clamp, PI, gamma};
use crate::core::transform::Transform;
use crate::core::shape::{Shapes, Shape};
use crate::core::interaction::{SurfaceInteraction, InteractionData};
use crate::core::geometry::ray::Ray;
use crate::core::geometry::bounds::Bounds3f;
use std::ops::Index;
use crate::core::sampling::uniform_sample_triangle;
use crate::core::paramset::ParamSet;
use std::collections::HashMap;
use log::{error, warn};
use crate::textures::constant::ConstantTexture;
use std::cmp::Ordering;

#[allow(dead_code)]
pub struct TriangleMesh {
    n_triangles         : usize,
    n_vertices          : usize,
    vertex_indices      : Vec<usize>,
    p                   : Vec<Point3f>,
    n                   : Vec<Normal3f>,
    s                   : Vec<Vector3f>,
    uv                  : Vec<Point2f>,
    alpha_mask          : Option<Arc<TextureFloat>>,
    shadow_alpha_mask   : Option<Arc<TextureFloat>>
}

impl TriangleMesh {
    pub fn new(
        object_to_world: &Transform, n_triangles: usize, vertex_indices: Vec<usize>,
        n_vertices: usize, pt: Vec<Point3f>, st: Vec<Vector3f>, nt: Vec<Normal3f>,
        uv: Vec<Point2f>, alpha_mask: Option<Arc<TextureFloat>>, shadow_alpha_mask: Option<Arc<TextureFloat>>
    ) -> Self {
        let p = pt.iter().map(|x| object_to_world.transform_point(x)).collect();
        let n = nt.iter().map(|x| object_to_world.transform_normal(x)).collect();
        let s = st.iter().map(|x| object_to_world.transform_vector(x)).collect();

        Self {
            n_triangles, n_vertices, vertex_indices,
            p, n, s, uv, alpha_mask, shadow_alpha_mask
        }
    }
}

pub fn create_trianglemesh(
    object_to_world: Arc<Transform>, world_to_object: Arc<Transform>,
    reverse_orientation: bool, n_triangles: usize, vertex_indices: Vec<usize>,
    n_vertices: usize, p: Vec<Point3f>, s: Vec<Vector3f>, n: Vec<Normal3f>, uv: Vec<Point2f>,
    alpha_mask: Option<Arc<TextureFloat>>, shadow_alpha_mask: Option<Arc<TextureFloat>>
) -> Vec<Arc<Shapes>> {
    let mesh = Arc::new(
        TriangleMesh::new(
            &object_to_world, n_triangles, vertex_indices, n_vertices,
            p, s, n, uv, alpha_mask, shadow_alpha_mask)
    );

    let mut tris = vec![];

    for i in 0..n_triangles {
        let tri = Triangle::new(
            object_to_world.clone(), world_to_object.clone(),
            reverse_orientation, i, mesh.clone());

        tris.push(Arc::new(tri.into()));
    }

    tris
}

pub struct Triangle {
    mesh                        : Arc<TriangleMesh>,
    v                           : usize,
    object_to_world             : Arc<Transform>,
    world_to_object             : Arc<Transform>,
    reverse_orientation         : bool,
    transform_swapshandedness   : bool

}

impl Triangle {
    pub fn new(object_to_world: Arc<Transform>, world_to_object: Arc<Transform>,
        reverse_orientation: bool, tri_number: usize, mesh: Arc<TriangleMesh>
    ) -> Self {
        let sh = object_to_world.swaps_handedness();

        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            v: tri_number * 3,
            mesh,
            transform_swapshandedness: sh,
        }
    }

    pub fn get_positions(&self) -> [Point3f; 3] {
        [
            self.mesh.p[self[0]],
            self.mesh.p[self[1]],
            self.mesh.p[self[2]]
        ]
    }

    fn get_uvs(&self) -> [Point2f; 3] {
        if !self.mesh.uv.is_empty() {
            [self.mesh.uv[self[0]], self.mesh.uv[self[1]], self.mesh.uv[self[2]]]
        } else {
            [Point2f::new(0.0, 0.0), Point2f::new(1.0, 0.0), Point2f::new(1.0, 1.0)]
        }
    }
}

impl Shape for Triangle {
    fn object_bound(&self) -> Bounds3f {
        // get triable vertices in p0, p1, and p2
        let [p0, p1, p2] = self.get_positions();
        let b = Bounds3f::from_points(
            self.world_to_object.transform_point(&p0),
            self.world_to_object.transform_point(&p1)
        );

        b.union_point(&self.world_to_object.transform_point(&p2))
    }

    fn world_bound(&self) -> Bounds3f {
        // get triable vertices in p0, p1, and p2
        let [p0, p1, p2] = self.get_positions();
        Bounds3f::from_points(p0, p1).union_point(&p2)
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut Float,
        isect: &mut SurfaceInteraction,
        test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {

        // get triable vertices in p0, p1, and p2
        let [p0, p1, p2] = self.get_positions();
        // Transform triangle vertices to ray coordinate space

        // Translate vertices based on ray origin
        let mut p0t = p0 - Vector3f::from(r.o);
        let mut p1t = p1 - Vector3f::from(r.o);
        let mut p2t = p2 - Vector3f::from(r.o);

        // permute components of triangle vertices and ray direction
        let kz = r.d.abs().max_dimension();
        let mut kx = kz + 1;
        if kx == 3 { kx = 0; }
        let mut ky = kx + 1;
        if ky == 3 { ky = 0; }
        let d = r.d.permute(kx, ky, kz);
        p0t = p0t.permute(kx, ky, kz);
        p1t = p1t.permute(kx, ky, kz);
        p2t = p2t.permute(kx, ky, kz);

        // Apply shear transformation to translated vertex positions
        let Sx = -d.x / d.z;
        let Sy = -d.y / d.z;
        let Sz = 1.0 / d.z;
        p0t.x += Sx * p0t.z;
        p0t.y += Sy * p0t.z;
        p1t.x += Sx * p1t.z;
        p1t.y += Sy * p1t.z;
        p2t.x += Sx * p2t.z;
        p2t.y += Sy * p2t.z;

        // compute edge function coefficients e0, e1 and e2
        let mut e0 = p1t.x * p2t.y - p1t.y * p2t.x;
        let mut e1 = p2t.x * p0t.y - p2t.y * p0t.x;
        let mut e2 = p0t.x * p1t.y - p0t.y * p1t.x;

        // fall back to double precision test at triangle edge
        if std::mem::size_of::<f32>() == std::mem::size_of::<Float>() && (e0 == 0.0 || e1 == 0.0 || e2 == 0.0) {
            let p2txp1ty = p2t.x as f64 * p1t.y as f64;
            let p2typ1tx = p2t.y as f64 * p1t.x as f64;
            e0 = (p2typ1tx - p2txp1ty) as f32;
            let p0txp2ty = p0t.x as f64 * p2t.y as f64;
            let p0typ2tx = p0t.y as f64 * p2t.x as f64;
            e1 = (p0typ2tx - p0txp2ty) as f32;
            let p1txp0ty = p1t.x as f64 * p0t.y as f64;
            let p1typ0tx = p1t.y as f64 * p0t.x as f64;
            e2 = (p1typ0tx - p1txp0ty) as f32;

        }

        // perform triangle edge and determinant tests
        if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) { return false; }

        let det = (e0 + e1 + e2) as Float;
        if det == 0.0 { return false;}

        // Compute scaled hit distance to triangle and test against ray t range
        p0t.z *= Sz;
        p1t.z *= Sz;
        p2t.z *= Sz;
        let tscaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
        if det < 0.0 && (tscaled >= 0.0 || tscaled < r.t_max * det) {
            return false;
        } else if det > 0.0 && (tscaled <= 0.0 || tscaled >= r.t_max * det) {
            return false;
        }

        // Compute barycentric coordinates and t value for triangle intersection
        let invdet = 1.0 / det;
        let b0 = e0 * invdet;
        let b1 = e1 * invdet;
        let b2 = e2 * invdet;
        let t = tscaled * invdet;

        // Ensure that computed triangle t is conservatively greater than zero

        // Compute deltaz term for triangle error bounds
        let maxzt = Vector3f::new(p0t.z, p1t.z, p2t.z).abs().max_component();
        let deltaz = gamma(3) * maxzt;

        // Compute deltax and deltay terms for triangle t error bounds
        let maxxt = Vector3f::new(p0t.x, p1t.x, p2t.x).abs().max_component();
        let maxyt = Vector3f::new(p0t.y, p1t.y, p2t.y).abs().max_component();
        let deltax = gamma(5) * (maxxt + maxzt);
        let deltay = gamma(5) * (maxyt + maxzt);

        // Compute deltae term for triangle t error bounds
        let deltae = 2.0 * (gamma(2) * maxxt * maxyt + deltay * maxxt + deltax * maxyt);

        // Compute deltat term for triangle t error bounds and check t
        let maxe = Vector3f::new(e0, e1, e2).abs().max_component();
        let deltat = 3.0 * (gamma(3) * maxe * maxzt + deltae * maxzt + deltaz * maxe) * invdet.abs();
        if t <= deltat { return false; }

        // Compute trianlge partial derivatives
        let mut dpdu = Vector3f::default();
        let mut dpdv = Vector3f::default();
        let uv = self.get_uvs();

        // Compute deltas for triangle partial derivatives
        let duv02 = uv[0] - uv[2];
        let duv12 = uv[1] - uv[2];
        let dp02 = p0 - p2;
        let dp12 = p1 - p2;
        let determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
        let degenerateuv = determinant.abs() < 1.0e-8;

        if !degenerateuv {
            let invdet = 1.0 / determinant;
            dpdu = (dp02 * duv12[1] - dp12 * duv02[1]) * invdet;
            dpdv = (dp02 * -duv12[0] + dp12 * duv02[0]) * invdet;
        }

        if degenerateuv || (dpdu.cross(&dpdv)).length_squared() == 0.0 {
            // Handle zero determinant for triangle partial derivative matric
            let ng = (p2 - p0).cross(&(p1 - p0));
            if ng.length_squared() == 0.0 {
                // The triangle is actually degenerate; the intersection is bogus
                return false
            }

            vec3_coordinate_system(&ng.normalize(), &mut dpdu, &mut dpdv);
        }

        // Compute error bounds for trianlge intersection
        let xabs_sum = (b0 * p0.x).abs() + (b1 * p1.x).abs() + (b2 * p2.x).abs();
        let yabs_sum = (b0 * p0.y).abs() + (b1 * p1.y).abs() + (b2 * p2.y).abs();
        let zabs_sum = (b0 * p0.z).abs() + (b1 * p1.z).abs() + (b2 * p2.z).abs();
        let perror = Vector3f::new(xabs_sum, yabs_sum, zabs_sum) * gamma(7);

        // Interpolate (u, v) parametric coordinates and hit point
        let phit = p0 * b0 + p1 * b1 + p2 * b2;
        let uvhit = uv[0] * b0 + uv[1] * b1 + uv[2] * b2;

        // Test intersection agains alpha texture if present
        if test_aphatexture {
            if let Some(ref mask) = self.mesh.alpha_mask {
                let isectl = SurfaceInteraction::new(
                    &phit, &Vector3f::default(), &uvhit, &(-r.d),
                    &dpdu, &dpdv, &Normal3f::default(), &Normal3f::default(),
                    r.time, s.clone());

                if mask.evaluate(&isectl) == 0.0 { return false }
            }
        }

        // Fill in Surface interaction from triangle hit
        *isect = SurfaceInteraction::new(
            &phit, &perror, &uvhit, &(-r.d), &dpdu, &dpdv,
            &Normal3f::default(), &Normal3f::default(), r.time, s);

        // Override surface normal in isect for trianlge
        let nn = Normal3f::from(dp02.cross(&dp12).normalize());
        isect.n = nn;
        isect.shading.n = nn;
        isect.wo = -r.d;

        if self.reverse_orientation ^ self.transform_swapshandedness {
            isect.n = -nn;
            isect.shading.n = -nn;
        }

        if !self.mesh.n.is_empty() || !self.mesh.s.is_empty() {
            // Initialize Triangle shading geometry

            // Compute shading normal ns for triangle
            let ns = if !self.mesh.n.is_empty() {
                let mut ns =
                    self.mesh.n[self[0]] * b0 +
                    self.mesh.n[self[1]] * b1 +
                    self.mesh.n[self[2]] * b2;
                if ns.length_squared() > 0.0 {
                    ns = ns.normalize();
                } else {
                    ns = isect.n;
                }

                ns
            } else {
                isect.n
            };

            // Compute shading tangent ss for triangle
            let mut ss = if !self.mesh.s.is_empty() {
                let mut ss =
                    self.mesh.s[self[0]] * b0 +
                    self.mesh.s[self[1]] * b1 +
                    self.mesh.s[self[2]] * b2;

                if ss.length_squared() > 0.0 {
                    ss = ss.normalize();
                } else {
                    ss = isect.dpdu.normalize();
                }

                ss
            } else {
                isect.dpdu.normalize()
            };

            // Compute shading bitangent ts for triangle and adjust ss
            let mut ts = ss.cross_norm(&ns);
            if ts.length_squared() > 0.0 {
                ts = ts.normalize();
                ss = ts.cross_norm(&ns);
            } else {
                vec3_coordinate_system(&Vector3f::from(ns), &mut ss, &mut ts);
            }

            // Compute dndu and dndv for triangle shading geometry;
            let (dndu, dndv) = if !self.mesh.n.is_empty() {
                // Compute deltas for triangle partial derivatives of normal
                let duv02 = uv[0] - uv[2];
                let duv12 = uv[1] - uv[2];
                let dn1 = self.mesh.n[self[0]] - self.mesh.n[self[2]];
                let dn2 = self.mesh.n[self[1]] - self.mesh.n[self[2]];
                let det = duv02[0] * duv12[1] - duv02[1] * duv12[0];
                let degenuv = det.abs() < 1.0e-8;

                if degenuv {
                    // We can still compute dndu and dndv, with respect to the
                    // same arbitrary coordinate system we use to compute dpdu
                    // and dpdv when this happens. It's important to do this
                    // (rather than giving up) so that ray differentials for
                    // rays reflected from triangles with degenerate
                    // parameterizations are still reasonable.
                    let v1 = Vector3f::from(self.mesh.n[self[2]] - self.mesh.n[self[0]]);
                    let v2 = Vector3f::from(self.mesh.n[self[1]] - self.mesh.n[self[0]]);
                    let dn = v1.cross(&v2);
                    if dn.length_squared() == 0.0 {
                        (Normal3f::default(), Normal3f::default())
                    } else {
                        let mut dnu = Vector3f::default();
                        let mut dnv = Vector3f::default();
                        vec3_coordinate_system(&dn, &mut dnu, &mut dnv);
                        (Normal3f::from(dnu), Normal3f::from(dnv))
                    }
                } else {
                    let invdet = 1.0 / det;
                    (
                        (dn1 * duv12[1] - dn2 * duv02[1]) * invdet,
                        (dn1 * -duv12[0] + dn2 * duv02[0]) * invdet
                    )
                }
            } else {
                (Normal3f::default(), Normal3f::default())
            };

            if self.reverse_orientation { ts = -ts; }

            isect.set_shading_geometry(&ss, &ts, &dndu, &dndv, true);
        }

        *t_hit = t;

        true

    }

    fn intersect_p(&self, r: &Ray, test_alpha_texture: bool) -> bool {
        // TODO: ProfilePhase
        // get triable vertices in p0, p1, and p2
        let [p0, p1, p2] = self.get_positions();
        // Transform triangle vertices to ray coordinate space

        // Translate vertices based on ray origin
        let mut p0t = p0 - Vector3f::from(r.o);
        let mut p1t = p1 - Vector3f::from(r.o);
        let mut p2t = p2 - Vector3f::from(r.o);


        // permute components of triangle vertices and ray direction
        let kz = r.d.abs().max_dimension();
        let mut kx = kz + 1;
        if kx == 3 { kx = 0; }
        let mut ky = kx + 1;
        if ky == 3 { ky = 0; }
        let d = r.d.permute(kx, ky, kz);
        p0t = p0t.permute(kx, ky, kz);
        p1t = p1t.permute(kx, ky, kz);
        p2t = p2t.permute(kx, ky, kz);

        // Apply shear transformation to translated vertex positions
        let Sx = -d.x / d.z;
        let Sy = -d.y / d.z;
        let Sz = 1.0 / d.z;
        p0t.x += Sx * p0t.z;
        p0t.y += Sy * p0t.z;
        p1t.x += Sx * p1t.z;
        p1t.y += Sy * p1t.z;
        p2t.x += Sx * p2t.z;
        p2t.y += Sy * p2t.z;

        // compute edge function coefficients e0, e1 and e2
        let mut e0 = p1t.x * p2t.y - p1t.y * p2t.x;
        let mut e1 = p2t.x * p0t.y - p2t.y * p0t.x;
        let mut e2 = p0t.x * p1t.y - p0t.y * p1t.x;

        // fall back to double precision test at triangle edge
        if std::mem::size_of::<f32>() == std::mem::size_of::<Float>() && (e0 == 0.0 || e1 == 0.0 || e2 == 0.0) {
            let p2txp1ty = p2t.x as f64 * p1t.y as f64;
            let p2typ1tx = p2t.y as f64 * p1t.x as f64;
            e0 = (p2typ1tx - p2txp1ty) as f32;
            let p0txp2ty = p0t.x as f64 * p2t.y as f64;
            let p0typ2tx = p0t.y as f64 * p2t.x as f64;
            e1 = (p0typ2tx - p0txp2ty) as f32;
            let p1txp0ty = p1t.x as f64 * p0t.y as f64;
            let p1typ0tx = p1t.y as f64 * p0t.x as f64;
            e2 = (p1typ0tx - p1txp0ty) as f32;

        }

        // perform triangle edge and determinant tests
        if (e0 < 0.0 || e1 < 0.0 || e2 < 0.0) && (e0 > 0.0 || e1 > 0.0 || e2 > 0.0) { return false; }

        let det = (e0 + e1 + e2) as Float;
        if det == 0.0 { return false;}

        // Compute scaled hit distance to triangle and test against ray t range
        p0t.z *= Sz;
        p1t.z *= Sz;
        p2t.z *= Sz;
        let tscaled = e0 * p0t.z + e1 * p1t.z + e2 * p2t.z;
        if det < 0.0 && (tscaled >= 0.0 || tscaled < r.t_max * det) {
            return false;
        } else if det > 0.0 && (tscaled <= 0.0 || tscaled >= r.t_max * det) {
            return false;
        }

        // Compute barycentric coordinates and t value for triangle intersection
        let invdet = 1.0 / det;
        let b0 = e0 * invdet;
        let b1 = e1 * invdet;
        let b2 = e2 * invdet;
        let t = tscaled * invdet;

        // Ensure that computed triangle t is conservatively greater than zero

        // Compute deltaz term for triangle error bounds
        let maxzt = Vector3f::new(p0t.z, p1t.z, p2t.z).abs().max_component();
        let deltaz = gamma(3) * maxzt;

        // Compute deltax and deltay terms for triangle t error bounds
        let maxxt = Vector3f::new(p0t.x, p1t.x, p2t.x).abs().max_component();
        let maxyt = Vector3f::new(p0t.y, p1t.y, p2t.y).abs().max_component();
        let deltax = gamma(5) * (maxxt + maxzt);
        let deltay = gamma(5) * (maxyt + maxzt);

        // Compute deltae term for triangle t error bounds
        let deltae = 2.0 * (gamma(2) * maxxt * maxyt + deltay * maxxt + deltax * maxyt);

        // Compute deltat term for triangle t error bounds and check t
        let maxe = Vector3f::new(e0, e1, e2).abs().max_component();
        let deltat = 3.0 * (gamma(3) * maxe * maxzt + deltae * maxzt + deltaz * maxe) * invdet.abs();
        if t <= deltat { return false; }

        if test_alpha_texture && (self.mesh.alpha_mask.is_some() || self.mesh.shadow_alpha_mask.is_some()) {
            // Compute trianlge partial derivatives
            let mut dpdu = Vector3f::default();
            let mut dpdv = Vector3f::default();
            let uv = self.get_uvs();

            // Compute deltas for triangle partial derivatives
            let duv02 = uv[0] - uv[2];
            let duv12 = uv[1] - uv[2];
            let dp02 = p0 - p2;
            let dp12 = p1 - p2;
            let determinant = duv02[0] * duv12[1] - duv02[1] * duv12[0];
            let degenerateuv = determinant.abs() < 1.0e-8;

            if !degenerateuv {
                let invdet = 1.0 / determinant;
                dpdu = (dp02 * duv12[1] - dp12 * duv02[1]) * invdet;
                dpdv = (dp02 * -duv12[0] + dp12 * duv02[0]) * invdet;
            }

            if degenerateuv || (dpdu.cross(&dpdv)).length_squared() == 0.0 {
                // Handle zero determinant for triangle partial derivative matric
                let ng = (p2 - p0).cross(&(p1 - p0));
                if ng.length_squared() == 0.0 {
                    // The triangle is actually degenerate; the intersection is bogus
                    return false
                }

                vec3_coordinate_system(&ng.normalize(), &mut dpdu, &mut dpdv);
            }

            // Interpolate (u, v) parametric coordinates and hit point
            let phit = p0 * b0 + p1 * b1 + p2 * b2;
            let uvhit = uv[0] * b0 + uv[1] * b1 + uv[2] * b2;
            let isectl = SurfaceInteraction::new(
                &phit, &Vector3f::default(), &uvhit, &(-r.d),
                &dpdu, &dpdv, &Normal3f::default(), &Normal3f::default(),
                r.time, None);

            // Test intersection agains alpha texture if present

            if let Some(ref mask) = self.mesh.alpha_mask {
                if mask.evaluate(&isectl) == 0.0 { return false }
            }

            if let Some(ref mask) = self.mesh.shadow_alpha_mask {
                if mask.evaluate(&isectl) == 0.0 { return false }
            }
        }

        true
    }

    fn area(&self) -> Float {
        let [p0, p1, p2] = self.get_positions();

        0.5 * (p1 -p0).cross(&(p2 - p0)).length()
    }

    fn sample(&self, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let b = uniform_sample_triangle(u);

        // Get triangle vertices in P0, p1 and p2
        let p0 = &self.mesh.p[self[0]];
        let p1 = &self.mesh.p[self[1]];
        let p2 = &self.mesh.p[self[2]];

        let mut it = InteractionData::default();
        it.p = *p0 * b[0] + *p1 * b[1] + *p2 * (1.0 - b[0] - b[1]);
        // Compute surface normal for sampled point on triangle
        it.n = Normal3f::from((*p1 - *p0).cross(&(*p2 - *p0))).normalize();
        // Ensure correct orientation of the geometric normal; follow the same
        // approach as was used in Triangle::intersect()
        if !self.mesh.n.is_empty() {
            let ns = self.mesh.n[self[0]] * b[0] + self.mesh.n[self[1]] * b[1] +
                self.mesh.n[self[2]] * (1.0 - b[0] - b[1]);
            it.n = it.n.face_foward(&ns);
        } else if self.reverse_orientation ^ self.transform_swapshandedness {
            it.n *= -1.0;
        }

        // Compute error bounds for sampled point on triangle
        let pabs_sum = (*p0 * b[0]).abs() + (*p1 * b[1]).abs() + (*p2 * (1.0 - b[0] - b[1])).abs();
        it.p_error = Vector3f::new(pabs_sum.x, pabs_sum.y, pabs_sum.z) * gamma(6);
        *pdf = 1.0 / self.area();

        it
    }

    fn solid_angle(&self, p: &Point3f, _nsamples: usize) -> Float {
        // Project the vertices into the unit sphere around p
        let pshere = [
            (self.mesh.p[self[0]] - *p).normalize(),
            (self.mesh.p[self[1]] - *p).normalize(),
            (self.mesh.p[self[2]] - *p).normalize(),
        ];

        // http://math.stackexchange.com/questions/9819/area-of-a-spherical-triangle
        // Girard's theorem: surface area of a spherical triangle on a unit
        // sphere is the 'excess angle' alpha+beta+gamma-pi, where
        // alpha/beta/gamma are the interior angles at the vertices.
        //
        // Given three vertices on the sphere, a, b, c, then we can compute,
        // for example, the angle c->a->b by
        //
        // cos theta =  Dot(Cross(c, a), Cross(b, a)) /
        //              (Length(Cross(c, a)) * Length(Cross(b, a))).
        //
        let mut cross01 = pshere[0].cross(&pshere[1]);
        let mut cross12 = pshere[1].cross(&pshere[2]);
        let mut cross20 = pshere[2].cross(&pshere[0]);

        // Some of these vectors may be degenerate. In this case, we don't want
        // to normalize them so that we don't hit an assert. This is fine,
        // since the corresponding dot products below will be zero.
        if cross01.length_squared() > 0.0 { cross01 = cross01.normalize(); }
        if cross12.length_squared() > 0.0 { cross12 = cross12.normalize(); }
        if cross20.length_squared() > 0.0 { cross20 = cross20.normalize(); }

        // We only need to do three cross products to evaluate the angles at
        // all three vertices, though, since we can take advantage of the fact
        // that Cross(a, b) = -Cross(b, a).
        (
            clamp(cross01.dot(&(-cross12)), -1.0, 1.0).acos() +
            clamp(cross12.dot(&(-cross20)), -1.0, 1.0).acos() +
            clamp(cross20.dot(&(-cross01)), -1.0, 1.0).acos() - PI
        ).abs()
    }


    fn reverse_orientation(&self) -> bool {
        self.reverse_orientation
    }

    fn transform_swapshandedness(&self) -> bool {
        self.transform_swapshandedness
    }

    fn object_to_world(&self) -> Arc<Transform> {
        self.object_to_world.clone()
    }

    fn world_to_object(&self) -> Arc<Transform> {
        self.world_to_object.clone()
    }
}

impl Index<usize> for Triangle {
    type Output = usize;

    fn index(&self, i: usize) -> &Self::Output {
        match i {
            0 => &self.mesh.vertex_indices[self.v],
            1 => &self.mesh.vertex_indices[self.v + 1],
            2 => &self.mesh.vertex_indices[self.v + 2],
            _ => panic!("Wrong argument. i >= 0 && i < 2")
        }
    }
}

pub fn create_trianglemesh_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool, params:
    &ParamSet, ftextures: &HashMap<String, Arc<TextureFloat>>) -> Vec<Arc<Shapes>> {
    let mut nvi = 0;
    let mut npi = 0;
    let mut nsi = 0;
    let mut nni = 0;
    let vi: Vec<usize> = params.find_int("indices", &mut nvi)
        .unwrap_or_default()
        .iter()
        .map(|x| *x as usize)
        .collect();
    let P = params.find_point3f("P", &mut npi).unwrap_or_default();
    let uvs = params.find_point2f("uv", &mut nvi)
        .or_else(|| params.find_point2f("st", &mut nvi))
        .or_else(|| params.find_float("uv", &mut nvi)
            .or_else(|| params.find_float("st", &mut nvi))
            .map(|fuvs| fuvs.chunks(2).map(|s| Point2f::new(s[0], s[1])).collect()))
        .unwrap_or_default();

    if !uvs.is_empty() {
        match uvs.len().cmp(&P.len()) {
            Ordering::Less    =>
                error!(
                    "Not enough of \"uv\"s for triangle mesh. Expected {}, \
                    found {}. Discarding", npi, uvs.len()),
            Ordering::Greater =>
                warn!(
                    "More \"uv\"s provided than will be used for triangle \
                    mesh. ({} expected, {} found)", npi, uvs.len()),
            _                 => ()
        }
    }

   if vi.is_empty() {
       error!("Vertex indices \"indices not provided with triangle mesh shape\"");
       return vec![];
   }
   if P.is_empty() {
       error!("Vertex positions \"P\" not provided with triangle mesh shape");
       return vec![]
   }

   let S = params.find_vector3f("S", &mut nsi).and_then(|s| {
       if s.len() != P.len() {
           error!("Number of \"S\"s for triangle mesh must match \"P\"s");
           None
       } else {
           Some(s)
       }
   }).unwrap_or_default();

   let N = params.find_normal3f("N", &mut nni).and_then(|n| {
       if n.len() != P.len() {
           error!("Number of \"N\"s for triangle mesh must match \"P\"s");
           None
       } else {
           Some(n)
       }
   }).unwrap_or_default();

    for v in vi.iter() {
        if *v >= P.len() {
            error!(
                "trianglemesh has out of-bounds vertex index {} ({} \"P\" \
                values were given", v, P.len());
            return vec![];
        }
    }

    let mut alphatex: Option<Arc<TextureFloat>> = None;
    let texname = params.find_texture("alpha", "".to_owned());

    if !texname.is_empty() {
        if let Some(tex) = ftextures.get(&texname) {
            alphatex = Some(tex.clone());
        } else {
            error!("Couldn't find float texture \"{}\" for \"alpha\" parameter", texname);
        }
    } else if params.find_one_float("alpha", 1.0) == 0.0 {
        alphatex = Some(Arc::new(ConstantTexture::new(0.0).into()));
    }

    let mut shadow_alphatex: Option<Arc<TextureFloat>> = None;
    let texname = params.find_texture("shadowalpha", "".to_owned());

    if !texname.is_empty() {
        if let Some(tex) = ftextures.get(&texname) {
            shadow_alphatex = Some(tex.clone());
        } else {
            error!(
                "Couldn't find float texture \"{}\" for \"shadowalpha\"\
                parameter", texname);
        }
    } else if params.find_one_float("shadowalpha", 1.0) == 0.0 {
        shadow_alphatex = Some(Arc::new(ConstantTexture::new(0.0).into()));
    }

    create_trianglemesh(
        o2w, w2o, reverse_orientation,
        vi.len() / 3, vi, P.len(),
        P,S, N, uvs, alphatex, shadow_alphatex )


}