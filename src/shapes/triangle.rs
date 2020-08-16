use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::geometry::normal::Normal3f;
use crate::core::geometry::vector::Vector3f;
use std::sync::Arc;
use crate::core::texture::Texture;
use crate::core::pbrt::Float;
use crate::core::transform::Transform;
use crate::core::shape::{Shape, IShape};
use crate::core::interaction::{Interaction, SurfaceInteraction};
use crate::core::geometry::ray::{Ray, BaseRay};
use crate::core::geometry::bounds::Bounds3f;

pub struct TriangleMesh {
    n_triangles: usize,
    n_vertices: usize,
    vertex_indices: Vec<usize>,
    p: Vec<Point3f>,
    n: Vec<Normal3f>,
    s: Vec<Vector3f>,
    uv: Vec<Point2f>,
    alpha_mask: Option<Arc<dyn Texture<Float> +  Sync + Send>>
}

impl TriangleMesh {
    pub fn new(
        object_to_world: &Transform,
        n_triangles: usize,
        vertex_indices: Vec<usize>,
        n_vertices: usize,
        p: Vec<Point3f>,
        s: Vec<Vector3f>,
        n: Vec<Normal3f>,
        uv: Vec<Point2f>,
        alpha_mask: Option<Arc<dyn Texture<Float> + Sync + Send>>
    ) -> Self {
        let mut p = p;
        let mut n = n;
        let mut s = s;

        for i in 0..n_vertices {
            p[i] = object_to_world.transform_point(&p[i]);
            n[i] = object_to_world.transform_normal(&n[i]);
            s[i] = object_to_world.transform_vector(&s[i]);
        }

        Self {
            n_triangles,
            n_vertices,
            vertex_indices,
            p,
            n,
            s,
            uv,
            alpha_mask
        }
    }
}

pub struct Triangle {
    mesh: Arc<TriangleMesh>,
    v: usize,
    object_to_world: Transform,
    world_to_object: Transform,
    reverse_orientation: bool,
    transform_swapshandedness: bool

}

impl Triangle {
    pub fn new(
        object_to_world: Transform,
        world_to_object: Transform,
        reverse_orientation: bool,
        tri_number: usize,
        mesh: Arc<TriangleMesh>
    ) -> Self {
        Self {
            object_to_world,
            world_to_object,
            reverse_orientation,
            v: tri_number * 3,
            mesh,
            transform_swapshandedness: false,
        }
    }

    pub fn create_trianglemesh(
        object_to_world: &Transform,
        world_to_object: &Transform,
        reverse_orientation: bool,
        n_triangles: usize,
        vertex_indices: Vec<usize>,
        n_vertices: usize,
        p: Vec<Point3f>,
        s: Vec<Vector3f>,
        n: Vec<Normal3f>,
        uv: Vec<Point2f>,
        alpha_mask: Option<Arc<dyn Texture<Float> + Sync + Send>>
    ) -> Vec<Arc<Triangle>> {
        let mesh = Arc::new(
            TriangleMesh::new(object_to_world, n_triangles, vertex_indices, n_vertices, p, s, n, uv, alpha_mask)
        );

        let mut tris = vec![];

        for i in 0..n_triangles {
            let tri = Triangle::new(*object_to_world, *world_to_object, reverse_orientation, i, mesh.clone());

            tris.push(Arc::new(tri));
        }

        tris
    }
}

impl IShape for Triangle {
    fn object_bound(&self) -> Bounds3f {
        // get triable vertices in p0, p1, and p2
        let p0 = self.mesh.p[self.mesh.vertex_indices[self.v]];
        let p1 = self.mesh.p[self.mesh.vertex_indices[self.v + 1]];
        let p2 = self.mesh.p[self.mesh.vertex_indices[self.v + 2]];

        let b = Bounds3f::from_points(
            self.world_to_object.transform_point(&p0),
            self.world_to_object.transform_point(&p1)
        );

        b.union_point(&self.world_to_object.transform_point(&p2))

    }

    fn world_bound(&self) -> Bounds3f {
        // get triable vertices in p0, p1, and p2
        let p0 = self.mesh.p[self.mesh.vertex_indices[self.v]];
        let p1 = self.mesh.p[self.mesh.vertex_indices[self.v + 1]];
        let p2 = self.mesh.p[self.mesh.vertex_indices[self.v + 2]];

        Bounds3f::from_points(p0, p1).union_point(&p2)
    }

    fn intersect(&self, r: &Ray, t_hit: &mut f32, isect: &mut SurfaceInteraction, test_aphatexture: bool) -> bool {
        // get triable vertices in p0, p1, and p2
        let p0 = self.mesh.p[self.mesh.vertex_indices[self.v]];
        let p1 = self.mesh.p[self.mesh.vertex_indices[self.v + 1]];
        let p2 = self.mesh.p[self.mesh.vertex_indices[self.v + 2]];

        // Transform triangle vertices to ray coordinate space

        // Translate vertices based on ray origin
        let mut p0t = p0 - Vector3f::from(r.o());
        let mut p1t = p1 - Vector3f::from(r.o());
        let mut p2t = p2 - Vector3f::from(r.o());

        // permute components of triangle vertices and ray direction
        let kz = r.d().abs().max_dimension();
        let mut kx = kz + 1;
        if kx == 3 { kx = 0; }
        let mut ky = kx + 1;
        if ky == 3 { ky = 0; }
        let d = r.d().permute(kx, ky, kz);
        p0t = p0t.permute(kx, ky, kz);
        p1t = p1t.permute(kx, ky, kz);
        p2t = p2t.permute(kx, ky, kz);

        // Apply shear transformation to translated vertex positions
    }

    fn area(&self) -> f32 {
        unimplemented!()
    }

    fn sample(&self, u: &Point2f) -> Interaction {
        unimplemented!()
    }

    fn sample_interaction(&self, i: &Interaction, u: &Point2f) -> Interaction {
        unimplemented!()
    }

    fn pdf(&self, i: &Interaction, wi: &Vector3f) -> f32 {
        unimplemented!()
    }

    fn reverse_orientation(&self) -> bool {
        self.reverse_orientation
    }

    fn transform_swapshandedness(&self) -> bool {
        self.transform_swapshandedness
    }

    fn object_to_world(&self) -> Transform {
        self.object_to_world
    }

    fn world_to_object(&self) -> Transform {
        self.world_to_object
    }
}