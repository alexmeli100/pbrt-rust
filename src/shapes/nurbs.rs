use crate::core::pbrt::{Float, lerp};
use crate::core::geometry::vector::Vector3f;
use smallvec::{SmallVec, smallvec};
use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::transform::Transform;
use crate::core::paramset::ParamSet;
use bumpalo::core_alloc::sync::Arc;
use crate::core::shape::Shapes;
use log::error;
use crate::core::geometry::normal::Normal3f;
use crate::shapes::triangle::create_trianglemesh;

pub fn knot_offset(knot: &[Float], order: isize, np: isize, t: Float) -> usize {
    let first_knot = order - 1;

    let mut knot_offset = first_knot as usize;
    while t > knot[knot_offset + 1] { knot_offset += 1; }
    assert!(knot_offset < np as usize);
    assert!(t >= knot[knot_offset] && t <= knot[knot_offset + 1]);

    knot_offset
}

// doesn't handle flat out discontinuities in the curve...
#[derive(Default, Copy, Clone)]
struct Homogeneous3 {
    x: Float,
    y: Float,
    z: Float,
    w: Float
}

impl Homogeneous3 {
    pub fn new(xx: Float, yy: Float, zz: Float, ww: Float) -> Self {
        Self {
            x: xx, y: yy, z: zz, w: ww
        }
    }
}

fn nurbs_evaluate(
    order: isize, knot: &[Float], cp: &[Homogeneous3], cp_start: isize, np: isize,
    cp_stride: isize, t: Float, deriv: Option<&mut Vector3f>) -> Homogeneous3 {
    let mut alpha: Float;

    let knot_offset = knot_offset(knot, order, np, t);

    let cp_offset = knot_offset - order as usize + 1;
    assert!(cp_offset < np as usize);

    let mut cpwork: SmallVec<[Homogeneous3; 256]> = smallvec![Default::default(); order as usize];
    for i in 0..order as usize {
        cpwork[i] = cp[(cp_start + (cp_offset + i)  as isize * cp_stride) as usize];
    }

    for i in 0..(order - 2) {
        for j in 0..(order - 1 - i) {
            alpha =
                (knot[knot_offset + (1 + j) as usize] - t) /
                (knot[knot_offset + (1 + j) as usize] - knot[knot_offset + (j + 2 - order + i) as usize]);
            assert!(alpha >= 0.0 && alpha <= 1.0);

            cpwork[j as usize].x = cpwork[j as usize].x * alpha + cpwork[(j + 1) as usize].x * (1.0 - alpha);
            cpwork[j as usize].y = cpwork[j as usize].y * alpha + cpwork[(j + 1) as usize].y * (1.0 - alpha);
            cpwork[j as usize].z = cpwork[j as usize].z * alpha + cpwork[(j + 1) as usize].z * (1.0 - alpha);
            cpwork[j as usize].w = cpwork[j as usize].w * alpha + cpwork[(j + 1) as usize].w * (1.0 - alpha);
        }
    }

    alpha = (knot[knot_offset + 1] - t) / (knot[knot_offset + 1] - knot[knot_offset]);
    assert!(alpha >= 0.0 && alpha <= 1.0);

    let val = Homogeneous3::new(
        cpwork[0].x * alpha + cpwork[1].x * (1.0 - alpha),
        cpwork[0].y * alpha + cpwork[1].y * (1.0 - alpha),
        cpwork[0].z * alpha + cpwork[1].z * (1.0 - alpha),
        cpwork[0].w * alpha + cpwork[1].w * (1.0 - alpha));

    if let Some(d) = deriv {
        let factor = (order - 1) as Float / (knot[knot_offset + 1] - knot[knot_offset]);
        let delta = Homogeneous3::new(
            (cpwork[1].x - cpwork[0].x) * factor,
            (cpwork[1].y - cpwork[0].y) * factor,
            (cpwork[1].z - cpwork[0].z) * factor,
            (cpwork[1].w - cpwork[0].w) * factor);

            d.x = delta.x / val.w - (val.x * delta.w / (val.w * val.w));
            d.y = delta.y / val.w - (val.y * delta.w / (val.w * val.w));
            d.z = delta.z / val.w - (val.z * delta.w / (val.w * val.w));
    }

    val
}

fn nurbs_evaluate_surface(
    uorder: isize, uknot: &[Float], ucp: isize, u: Float,
    vorder: isize, vknot: &[Float], vcp: isize, v: Float,
    cp: &[Homogeneous3], dpdu: Option<&mut Vector3f>,
    dpdv: Option<&mut Vector3f>) -> Point3f {
    let mut iso: SmallVec<[Homogeneous3; 256]> =
        smallvec![Default::default(); std::cmp::max(uorder, vorder) as usize];

    let uoffset = knot_offset(uknot, uorder, ucp, u);
    let ufirst_cp = uoffset - uorder as usize + 1;

    for i in 0..uorder {
        iso[i as usize] = nurbs_evaluate(
            vorder, vknot, cp, ufirst_cp as isize + i,
            vcp, ucp, v, None);
    }

    let voffset = knot_offset(vknot, vorder, vcp, v);
    let vfirst_cp = voffset - vorder as usize + 1;
    assert!(vfirst_cp + vorder as usize - 1 < vcp as usize);

    let P = nurbs_evaluate(
        uorder, uknot, &iso, -(ufirst_cp as isize),
        ucp, 1, u, dpdu);

    if let Some(d) = dpdv {
        for i in 0..vorder {
            iso[i as usize] = nurbs_evaluate(
                uorder, uknot, cp, vfirst_cp as isize + i,
                ucp, 1, u, None);
        }

        nurbs_evaluate(
            vorder, vknot, &iso,  -(vfirst_cp as isize),
            vcp, 1, v, Some(d));
    }

    Point3f::new(P.x / P.w, P.y / P.w, P.z / P.w)
}

pub fn create_nurbs(
    o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool,
    params: &ParamSet) -> Vec<Arc<Shapes>> {
    let nu = params.find_one_int("nu", -1);
    if nu == -1 {
        error!("Must provide number of control points \"nu\" with NURBS shape.");
        return Vec::new();
    }

    let uorder = params.find_one_int("uorder", -1);
    if uorder == -1 {
        error!("Must provide u order \"uorder\" with NURBS shape.");
        return vec![]
    }

    let mut nuknots = 0; let mut nvknots = 0;
    let uknots = params.find_float("uknots", &mut nuknots);
    if uknots.is_none() {
        error!("Must provide u knot vector \"uknots\" with NURBS shape.");
        return vec![]
    }

    let uknots = uknots.unwrap();

    if nuknots as isize != nu + uorder {
        error!(
            "Number of knots in u vector {} doesn't match sum of \
            number of u control points {} and u order {}.",
            nuknots, nu, uorder);
        return vec![];
    }

    let u0 = params.find_one_float("u0", uknots[uorder as usize - 1]);
    let u1 = params.find_one_float("u1", uknots[nu as usize]);

    let nv = params.find_one_int("nv", -1);
    if nv == -1 {
        error!("Must provide number of control points \"nv\" with NURBS shape.");
        return vec![];
    }

    let vorder = params.find_one_int("vorder", -1);
    if vorder == -1 {
        error!("Must provide v order \"vorder\" with NURBS shape.");
        return vec![];
    }

    let vknots = params.find_float("vknots", &mut nvknots);
    if vknots.is_none() {
        error!("Must provide v knot vector \"vknots\" with NURBS shape");
        return vec![];
    }

    if nvknots as isize != nv + vorder {
        error!(
            "Number of knots in v knot vector {} doesn't match sum of \
            number of v control points {} and v order {}.",
            nvknots, nv, vorder);
    }

    let vknots = vknots.unwrap();
    let v0 = params.find_one_float("v0", vknots[vorder as usize - 1]);
    let v1 = params.find_one_float("v1", vknots[vorder as usize]);

    let mut is_homogeneous = false;
    let mut npts = 0;
    let p = params.find_point3f("P", &mut npts);
    let mut pw = Vec::new();

    if  p.is_none() {
        let pp = params.find_float("Pw", &mut npts);

        if pp.is_none() {
            error!(
                "Must provide control points via \"P\" or \"Pw\" parameter to \
                NURBS shape");
            return vec![];
        }
        if npts % 4 != 0 {
            error!(
                "Number of \"Pw\" control points provided to NURBS shape must \
                be multiple of four");
            return vec![];
        }

        npts /= 4;
        is_homogeneous = true;
        pw = pp.unwrap();
    };

    if npts != (nu * nv) as usize {
        error!(
            "NURBS shape was expecting {}x{}={} control points, was given {}.",
            nu, nv, nu * nv, npts);
        return vec![];
    }

    let diceu = 30;
    let dicev = 30;
    let mut ueval: Vec<Float> = vec![0.0; diceu];
    let mut veval: Vec<Float> = vec![0.0; dicev];
    let mut evalps = Vec::with_capacity(diceu * dicev);
    let mut evalns = Vec::with_capacity(diceu * dicev);

    for (i, ue) in ueval.iter_mut().enumerate() {
        *ue = lerp(i as Float / (diceu - 1) as Float, u0, u1);
    }

    for (i, ve) in veval.iter_mut().enumerate() {
        *ve = lerp(i as Float / (dicev - 1) as Float, v0, v1);
    }

    // Evaluate NURBS over grid of points
    let mut uvs = vec![Point2f::default(); diceu * dicev];

    // Turn NURBS into triangles
    let mut Pw = Vec::with_capacity((nu * nv) as usize);

    if is_homogeneous {
        for i in 0..(nu * nv) as usize {
            Pw.push(Homogeneous3::new(
                pw[4 * i],
                pw[4 * i + 1],
                pw[4 * i + 2],
                pw[4 * i + 3],
            ));
        }
    } else {
        let v = p.as_ref().unwrap();

        for item in v.iter().take((nu * nv) as usize) {
            Pw.push(Homogeneous3::new(item.x, item.y, item.z, 1.0));
        }
    }

    for v in 0..dicev {
        for u in 0..diceu {
            uvs[v * diceu + u].x = ueval[u];
            uvs[v * dicev + u].y = veval[v];

            let mut dpdu = Vector3f::default();
            let mut dpdv = Vector3f::default();
            let pt = nurbs_evaluate_surface(
                uorder, &uknots, nu, ueval[u],
                vorder, &vknots, nv, veval[v],
                &Pw, Some(&mut dpdu), Some(&mut dpdv));

            evalps.push(Point3f::new(pt.x, pt.y, pt.z));
            evalns.push(Normal3f::from(dpdu.cross(&dpdv).normalize()));
        }
    }

    // Generate points-polygons mesh
    let ntris = 2 * (diceu - 1) * (dicev - 1);
    let mut vertices: Vec<usize> = Vec::with_capacity(3 * ntris);

    macro_rules! vn {
        ($u:expr, $v:expr) => {{
            $v * diceu + $u
        }}
    }

    for v in 0..(dicev - 1) {
        for u in 0..(diceu - 1) {
            vertices.push(vn!(u, v));
            vertices.push(vn!(u + 1, v));
            vertices.push(vn!(u + 1, v + 1));
            vertices.push(vn!(u, v));
            vertices.push(vn!(u + 1, v + 1));
            vertices.push(vn!(u, v + 1));
        }
    }

    let nverts = diceu * dicev;

    create_trianglemesh(
        o2w, w2o, reverse_orientation, ntris,
        vertices, nverts, evalps, Vec::new(),
        evalns, uvs, None, None)
}