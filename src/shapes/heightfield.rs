use crate::core::transform::Transform;
use crate::core::paramset::ParamSet;
use crate::core::shape::Shapes;
use std::sync::Arc;
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::pbrt::Float;
use crate::shapes::triangle::create_trianglemesh;

pub fn create_heightfield(
    o2w: Arc<Transform>, w2o: Arc<Transform>,
    reverse_orientation: bool,
    params: &ParamSet) -> Vec<Arc<Shapes>> {
    let nx = params.find_one_int("nu", -1);
    let ny = params.find_one_int("nv", -1);
    let mut nitems = 0;
    let z = params.find_float("Pz", &mut nitems);
    assert_eq!(nitems as isize, nx * ny);
    assert!(nx != -1 && ny != -1 && z.is_some());

    let nverts = (nx * ny) as usize;
    let ntris = 2 * (nx - 1) as usize * (ny - 1) as usize;
    let mut indices = vec![0; 3 * ntris];
    let mut P = vec![Point3f::default(); nverts];
    let mut  uvs = vec![Point2f::default(); nverts];
    let mut pos = 0;
    let z = z.unwrap();

    // Compute heightfield vertex positions
    for y in 0..ny as usize {
        for x in 0..nx as usize {
            let xval = x as Float / (nx - 1) as Float;
            let yval = y as Float / (ny - 1) as Float;
            P[pos].x = xval;
            uvs[pos].x = xval;
            P[pos].y = yval;
            uvs[pos].y = yval;
            P[pos].z = z[pos];
            pos += 1;
        }
    }

    // Fill in heightfield vextex offset array
    let mut p = 0;
    macro_rules! vert {
        ($x:expr, $y:expr) => {{
            $x + $y * nx as usize
        }}
    }

    for y in 0..(ny - 1) as usize {
        for x in 0..(nx - 1) as usize {
            indices[p] = vert!(x, y);
            p += 1;
            indices[p] = vert!(x + 1, y);
            p += 1;
            indices[p] = vert!(x + 1, y + 1);
            p += 1;

            indices[p] = vert!(x, y);
            p += 1;
            indices[p] = vert!(x + 1, y + 1);
            p += 1;
            indices[p] = vert!(x, y + 1);
            p += 1;
        }
    }

    create_trianglemesh(
        o2w, w2o, reverse_orientation, ntris,
        indices, nverts, P, Vec::new(), Vec::new(),
        uvs, None, None)

}