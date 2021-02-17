use crate::core::geometry::point::{Point3f, Point2f};
use crate::core::pbrt::{Float, clamp, lerp, float_to_bits};
use crate::core::geometry::normal::Normal3f;
use crate::core::geometry::vector::{Vector3f, Vector2f, vec3_coordinate_system};
use crate::core::transform::Transform;
use crate::core::paramset::ParamSet;
use bumpalo::core_alloc::sync::Arc;
use crate::core::geometry::ray::Ray;
use crate::core::interaction::{SurfaceInteraction, InteractionData};
use crate::core::shape::{Shape, Shapes};
use crate::core::geometry::bounds::Bounds3f;
use crate::*;
use log::{warn, error};

stat_memory_counter!("Memory/Curves", curve_bytes);
stat_percent!("Intersections/Ray-curve intersection tests", nhits_test);
stat_int_distribution!("Intersections/Curve refinement level", refinement_level);
stat_counter!("Scene/Curves", ncurves);
stat_counter!("Scene/Split curves", nsplit_curves);

pub fn init_stats() {
    curve_bytes::init();
    nhits_test::init();
    refinement_level::init();
    ncurves::init();
    nsplit_curves::init();
}


fn blossom_bezier(p: &[Point3f; 4], u0: Float, u1: Float, u2: Float) -> Point3f {
    let a = [
        lerp(u0, p[0], p[1]),
        lerp(u0, p[1], p[2]),
        lerp(u0, p[2], p[3])
    ];
    let b = [ lerp(u1, a[0], a[1]), lerp(u1, a[1], a[2]) ];

    lerp(u2, b[0], b[1])
}

fn subdivide_bezier(cp: &[Point3f; 4], cp_split: &mut [Point3f; 7]) {
    cp_split[0] = cp[0];
    cp_split[1] = (cp[0] + cp[1]) / 2.0;
    cp_split[2] = (cp[0] + cp[1] * 2.0 + cp[2]) / 4.0;
    cp_split[3] = (cp[0] + cp[1] * 3.0 + cp[2] * 3.0 + cp[3]) / 8.0;
    cp_split[4] = (cp[1] + cp[2] * 2.0 + cp[3]) / 4.0;
    cp_split[5] = (cp[2] + cp[3]) / 2.0;
    cp_split[6] = cp[3];
}

fn eval_bezier(cp: &[Point3f; 4], u: Float, deriv: Option<&mut Vector3f>) -> Point3f {
    let cp1 = [
        lerp(u, cp[0], cp[1]),
        lerp(u, cp[1], cp[2]),
        lerp(u, cp[2], cp[3])
    ];
    let cp2 = [ lerp(u, cp1[0], cp1[1]), lerp(u, cp1[1], cp1[2]) ];

    if let Some(d) = deriv {
        if (cp2[1] - cp2[0]).length_squared() > 0.0 {
            *d = (cp2[1] - cp2[0]) * 3.0
        } else {
            // For a cubic Bezier, if the first three control points (say) are
            // coincident, then the derivative of the curve is legitimately (0,0,0)
            // at u=0.  This is problematic for us, though, since we'd like to be
            // able to compute a surface normal there.  In that case, just punt and
            // take the difference between the first and last control points, which
            // ain't great, but will hopefully do.
            *d = cp[3] - cp[0];
        }
    }

    lerp(u, cp2[0], cp2[1])
}

#[derive(Copy, Clone, Ord, PartialOrd, PartialEq, Eq)]
pub enum CurveType { Flat, Cylinder, Ribbon }

#[derive(Copy, Clone)]
pub struct CurveCommon {
    ty                  : CurveType,
    cpobj               : [Point3f; 4],
    width               : [Float; 2],
    n                   : [Normal3f; 2],
    normal_angle        : Float,
    inv_sin_normal_angle: Float
}

impl CurveCommon {
    pub fn new(
        c: [Point3f; 4], width0: Float, width1: Float,
        ty: CurveType, norm: Option<[Normal3f; 2]>) -> Self {
        let width = [width0, width1];
        let mut n = [Normal3f::default(); 2];
        let mut normal_angle: Float = 0.0;
        let mut inv_sin_normal_angle = 0.0;

        if let Some(no) = norm {
            n[0] = no[0].normalize();
            n[1] = no[1].normalize();
            normal_angle = clamp(n[0].dot(&n[1]), 0.0, 1.0).acos();
            inv_sin_normal_angle = 1.0 / normal_angle.sin();
        }
        ncurves::inc();

        Self {
            ty, width, n, normal_angle,
            inv_sin_normal_angle,
            cpobj: c

        }
    }
}

#[derive(Clone)]
pub struct Curve {
    common                       : Arc<CurveCommon>,
    umin                         : Float,
    umax                         : Float,
    pub object_to_world          : Arc<Transform>,
    pub world_to_object          : Arc<Transform>,
    pub reverse_orientation      : bool,
    pub transform_swapshandedness: bool
}

impl Curve {
    pub fn new(
        object_to_world: Arc<Transform>, world_to_object: Arc<Transform>,
        reverse_orientation: bool, common: Arc<CurveCommon>,
        umin: Float, umax: Float) -> Self {
        let sh = object_to_world.swaps_handedness();

        Self {
            object_to_world, world_to_object,
            reverse_orientation, common, umin, umax,
            transform_swapshandedness: sh
        }
    }

    fn recursive_intersect(
        &self, s: Option<Arc<Shapes>>, ray: &Ray, thit: &mut Float,
        isect: &mut SurfaceInteraction, cp: &[Point3f; 4],
        r2o: &Transform, u0: Float, u1: Float, depth: usize) -> bool {
        let ray_length = ray.d.length();

        if depth > 0 {
            // Split curve segment into sub-segments and test for intersection
            let mut cp_split = [Point3f::default(); 7];
            subdivide_bezier(cp, &mut cp_split);

            // For each of the two segments, see if the ray's bounding box
            // overlaps the segment before recursively checking for
            // intersection with it.
            let mut hit = false;
            let u = [u0, (u0 + u1) / 2.0, u1];
            // Pointer to the 4 control points for the current segment

            for seg in 0..2 {
                let cps: &[Point3f] = &cp_split[seg * 3..seg * 3 + 4];

                let max_width =
                    (lerp(u[seg], self.common.width[0], self.common.width[1])).max(
                        lerp(u[seg + 1], self.common.width[0], self.common.width[1]));

                // As above, check y first, since it most commonly lets us exit out early
                if cps[0].y.max(cps[1].y)
                    .max(cps[2].y.max(cps[3].y)) + 0.5 * max_width < 0.0 ||
                   cps[0].y.min(cps[1].y)
                       .min(cps[2].y.min(cps[3].y)) - 0.5 * max_width > 0.0 {
                    continue;
                }

                if cps[0].x.max(cps[1].x)
                    .max(cps[2].x.max(cps[3].x)) + 0.5 * max_width < 0.0 ||
                   cps[0].x.min(cps[1].x)
                       .min(cps[2].x.min(cps[3].x)) - 0.5 * max_width > 0.0 {
                    continue;
                }

                let zmax = ray_length * ray.t_max;

                if cps[0].z.max(cps[1].z)
                    .max(cps[2].z.max(cps[3].z)) + 0.5 * max_width < 0.0 ||
                   cps[0].z.min(cps[1].z)
                       .min(cps[2].z.min(cps[3].z)) - 0.5 * max_width > zmax {
                    continue;
                }

                hit |= self.recursive_intersect(
                    s.clone(), ray,  thit, isect, &[cps[0], cps[1], cps[2], cps[3]], r2o,
                    u[seg], u[seg + 1], depth - 1);

                // If we found an intersection and this is a shadow ray,
                // we can exit out immediately.
                if hit && *thit == 0.0 { return true; }
            }

            hit
        } else {
            // Intersect ray with curve segment

            // Test against segment endpoint boundaries

            // Test sample point against tangent perpendicular at curve start
            let mut edge = (cp[1].y - cp[0].y) * -cp[0].y + cp[0].x * (cp[0].x - cp[1].x);
            if edge < 0.0 { return false; }

            // Test sample point against tangent perpendicular at curve end
            edge = (cp[2].y - cp[3].y) * -cp[3].y + cp[3].x * (cp[3].x - cp[2].x);
            if edge < 0.0 { return false; }

            // Compute line w that gives minimum distance to sample point
            let segment_dir = Point2f::new(cp[3].x, cp[3].y) - Point2f::new(cp[0].x, cp[0].y);
            let denom = segment_dir.length_squared();
            if denom == 0.0 { return false; }
            let w = (-Vector2f::new(cp[0].x, cp[0].y)).dot(&segment_dir) / denom;

            // Compute u coordinate of curve intersection point and hitWidth
            let u = clamp(lerp(w, u0, u1), u0, u1);
            let mut hit_width = lerp(u, self.common.width[0], self.common.width[1]);
            let mut nhit = Normal3f::default();

            if let CurveType::Ribbon = self.common.ty {
                // Scale hitWidth based on ribbon orientation
                let sin0 = ((1.0 - u) * self.common.normal_angle).sin() * self.common.inv_sin_normal_angle;
                let sin1 = (u * self.common.normal_angle) * self.common.inv_sin_normal_angle;
                nhit = self.common.n[0] * sin0  + self.common.n[1] * sin1;
                hit_width *= nhit.abs_dot_vec(&ray.d) / ray_length;
            }

            // Test intersection point against curve width
            let mut dpcdw = Vector3f::default();
            let pc = eval_bezier(cp, clamp(w, 0.0, 1.0), Some(&mut dpcdw));
            let ptcurve_dist2 = pc.x * pc.x + pc.y * pc.y;
            if ptcurve_dist2 > hit_width * hit_width * 0.25 { return false; }
            let zmax = ray_length * ray.t_max;
            if pc.z < 0.0 || pc.z > zmax { return false; }

            // Compute v coordinate of curve intersection point
            let ptcurve_dist = ptcurve_dist2.sqrt();
            let edge_func = dpcdw.x * -pc.y + pc.x * dpcdw.y;
            let v = if edge_func > 0.0 {
                0.5 + ptcurve_dist / hit_width
            } else {
                0.5 - ptcurve_dist / hit_width
            };

            // Compute hit t and partial derivatives for curve intersection
            // FIXME: this tHit isn't quite right for ribbons...
            *thit = pc.z / ray_length;
            // Compute error bounds for curve intersection
            let perror = Vector3f::new(2.0 * hit_width, 2.0 * hit_width, 2.0 * hit_width);

            // Compute dpdu and dpdv for curve intersection
            let mut dpdu = Vector3f::default();
            let dpdv: Vector3f;
            eval_bezier(&self.common.cpobj, u, Some(&mut dpdu));
            assert_ne!(
                Vector3f::default(), dpdu, "u = {}, cp = {}, {}, {}, {}" ,
                u, self.common.cpobj[0], self.common.cpobj[1],
                self.common.cpobj[2], self.common.cpobj[3]);

            if let CurveType::Ribbon = self.common.ty {
                dpdv = nhit.cross_vec(&dpdu).normalize() * hit_width;
            } else {
                // Compute curve dpdv for flat and cylinder curves
                let dpdu_plane = Transform::inverse(r2o).transform_vector(&dpdu);
                let mut dpdv_plane = Vector3f::new(-dpdu_plane.y, dpdu_plane.x, 0.0).normalize() * hit_width;

                if let CurveType::Cylinder = self.common.ty {
                    // Rotate dpdvPlane to give cylindrical appearance
                    let theta = lerp(v, -90.0, 90.0);
                    let rot = Transform::rotate(-theta, &dpdu_plane);
                    dpdv_plane = rot.transform_vector(&dpdv_plane);
                }
                dpdv = r2o.transform_vector(&dpdv_plane);
            }

            let phit = ray.find_point(*thit);
            let dndu = Normal3f::default();
            let dndv = Normal3f::default();
            let uv = Point2f::new(u, v);
            *isect = SurfaceInteraction::new(&phit, &perror, &uv, &(-ray.d), &dpdu, &dpdv, &dndu, &dndv, ray.time, s);
            nhits_test::inc_num();

            true
        }
    }
}

impl Shape for Curve {
    fn object_bound(&self) -> Bounds3f {
        // Compute object-space control points for curve segment, cpobj
        let cpobj = [
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umin),
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umax),
            blossom_bezier(&self.common.cpobj, self.umin, self.umax, self.umax),
            blossom_bezier(&self.common.cpobj, self.umax, self.umax, self.umax)
        ];
        let b = Bounds3f::from_points(cpobj[0], cpobj[1]).union_bounds(&Bounds3f::from_points(cpobj[2], cpobj[3]));
        let width = [
            lerp(self.umin, self.common.width[0], self.common.width[1]),
            lerp(self.umax, self.common.width[0], self.common.width[1])
        ];

        b.expand(width[0].max(width[1]) * 0.5)
    }

    fn intersect(
        &self, r: &Ray, t_hit: &mut f32, isect: &mut SurfaceInteraction,
        _test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool {
        // TODO ProfilePhase
        nhits_test::inc_den();

        let mut oerr = Vector3f::default();
        let mut derr = Vector3f::default();
        let ray = self.world_to_object.transform_ray_error(r, &mut oerr, &mut derr);

        // Compute object-space control points for curve segment cpobj
        let cpobj = [
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umin),
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umax),
            blossom_bezier(&self.common.cpobj, self.umin, self.umax, self.umax),
            blossom_bezier(&self.common.cpobj, self.umax, self.umax, self.umax)
        ];

        // Project curve control points to plane perpendicular to ray

        // Be careful to set the "up" direction passed to LookAt() to equal the
        // vector from the first to the last control points.  In turn, this
        // helps orient the curve to be roughly parallel to the x axis in the
        // ray coordinate system.
        //
        // In turn (especially for curves that are approaching stright lines),
        // we get curve bounds with minimal extent in y, which in turn lets us
        // early out more quickly in recursiveIntersect().
        let mut dx = ray.d.cross(&(cpobj[3] - cpobj[0]));
        if dx.length_squared() == 0.0 {
            // If the ray and the vector between the first and last control
            // points are parallel, dx will be zero.  Generate an arbitrary xy
            // orientation for the ray coordinate system so that intersection
            // tests can proceeed in this unusual case.
            let mut dy = Vector3f::default();
            vec3_coordinate_system(&ray.d, &mut dx, &mut dy);
        }

        let o2r = Transform::look_at(&ray.o, &(ray.o + ray.d), &dx);
        let cp = [
            o2r.transform_point(&cpobj[0]), o2r.transform_point(&cpobj[1]),
            o2r.transform_point(&cpobj[2]), o2r.transform_point(&cpobj[3])
        ];

        // Before going any further, see if the ray's bounding box intersects
        // the curve's bounding box. We start with the y dimension, since the y
        // extent is generally the smallest (and is often tiny) due to our
        // careful orientation of the ray coordinate ysstem above.
        let max_width = lerp(self.umin, self.common.width[0], self.common.width[1])
                        .max(lerp(self.umax, self.common.width[0], self.common.width[1]));

        if cp[0].y.max(cp[1].y).max(cp[2].y.max(cp[3].y)) + 0.5 * max_width < 0.0 ||
           cp[0].y.min(cp[1].y).min(cp[2].y.min(cp[3].y)) - 0.5 * max_width > 0.0 {
            return false;
        }

        // Check for non-overlap in x.
        if cp[0].x.max(cp[1].x).max(cp[2].x.max(cp[3].x)) + 0.5 * max_width < 0.0 ||
           cp[0].x.min(cp[1].x).min(cp[2].x.min(cp[3].x)) - 0.5 * max_width > 0.0 {
            return false;
        }

        // Check for non-overlap in z.
        let ray_length = ray.d.length();
        let zmax = ray_length * ray.t_max;

        if cp[0].z.max(cp[1].z).max(cp[2].z.max(cp[3].z)) + 0.5 * max_width < 0.0 ||
           cp[0].z.min(cp[1].z).min(cp[2].z.min(cp[3].z)) + 0.5 * max_width > zmax {
            return false;
        }

        // Compute refinement depth for curve maxDepth
        let mut L0: Float = 0.0;

        for i in 0..2 {
            L0 = L0.max(
                (cp[i].x - 2.0 * cp[i + 1].x + cp[i + 2].x)
                    .abs()
                    .max((cp[i].y - 2.0 * cp[i + 1].y + cp[i + 2].y).abs())
                    .max((cp[i].z - 2.0 * cp[i + 1].z + cp[i + 2].z).abs()));
        }

        let eps = self.common.width[0].max(self.common.width[1]) * 0.05;
        let log2 = |v: Float| -> isize {
            if v < 1.0 { return 0; }
            let bits = float_to_bits(v) as isize;
            // https://graphics.stanford.edu/~seander/bithacks.html#IntegerLog
            // (With an additional add so get round-to-nearest rather than
            // round down.)
            (bits >> 23) - 127 + (bits & (if (1 << 22) > 0 { 1 } else { 0 }))
        };
        // Compute log base 4 by dividing log2 in half
        let r0 = log2(1.41421356237 * 6.0 * L0 / (8.0 * eps)) / 2;
        let max_depth = clamp(r0, 0, 10);
        let r2o = Transform::inverse(&o2r);
        refinement_level::report_value(max_depth as u64);

        self.recursive_intersect(s, &ray, t_hit, isect, &cp, &r2o, self.umin, self.umax, max_depth as usize)
    }

    fn area(&self) -> f32 {
        // Compute object-space control points for curve segment cpobj
        let cpobj = [
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umin),
            blossom_bezier(&self.common.cpobj, self.umin, self.umin, self.umax),
            blossom_bezier(&self.common.cpobj, self.umin, self.umax, self.umax),
            blossom_bezier(&self.common.cpobj, self.umax, self.umax, self.umax)
        ];
        let width0 = lerp(self.umin, self.common.width[0], self.common.width[1]);
        let width1 = lerp(self.umax, self.common.width[0], self.common.width[1]);
        let avg_width = (width0 + width1) *  0.5;
        let mut approx_length = 0.0;

        for i in 0..3 {
            approx_length += cpobj[i].distance(&cpobj[i + 1]);
        }

        approx_length * avg_width
    }

    fn sample(&self, _u: &Point2f, _pdf: &mut f32) -> InteractionData {
        error!("Curve::Sample not implemented");

        InteractionData::default()
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

pub fn create_curve(
    o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool,
    c: &[Point3f], w0: Float, w1: Float, ty: CurveType,
    norm: Option<[Normal3f; 2]>, split_depth: usize) -> Vec<Arc<Shapes>> {
    let cs = [c[0], c[1], c[2], c[3]];
    let common = Arc::new(CurveCommon::new(cs, w0, w1, ty, norm));
    let nsegments = 1 << split_depth;
    let mut segments = Vec::with_capacity(nsegments);

    for i in 0..nsegments {
        let umin = i as Float / nsegments as Float;
        let umax = (i + 1) as Float / nsegments as Float;
        let curve: Shapes = Curve::new(
            o2w.clone(), w2o.clone(), reverse_orientation,
            common.clone(), umin, umax).into();
        segments.push(Arc::new(curve));
        nsplit_curves::inc();
    }

    let t = std::mem::size_of::<CurveCommon>() + nsegments * std::mem::size_of::<Curve>();
    curve_bytes::add(t as u64);

    segments
}

pub fn create_curve_shape(
    o2w: Arc<Transform>, w2o: Arc<Transform>, reverse_orientation: bool,
    params: &ParamSet) -> Vec<Arc<Shapes>> {
    let width = params.find_one_float("width", 1.0);
    let width0 = params.find_one_float("width0", width);
    let width1 = params.find_one_float("width1", width);

    let degree = params.find_one_int("degree", 3);
    if degree != 2 && degree != 3 {
        error!(
            "Invalid degree {}: only degree 2 and 3 curves are supported.",
            degree);
        return vec![];
    }

    let basis = params.find_one_string("basis", "bezier".to_owned());
    if basis != "bezier" && basis != "bspline" {
        error!("Invalid basis \"{}\": only \"bezier\" and \"bspline\" are \
        supported.", basis);
        return vec![];
    }

    let mut ncp = 0;
    let cp = params.find_point3f("P", &mut ncp);
    let nsegments: usize;

    if basis == "bezier" {
        // After the first segment, which uses degree+1 control points,
        // subsequent segments reuse the last control point of the previous
        // one and then use degree more control points.
        if (ncp as isize - 1 - degree) % degree != 0 {
            error!(
                "Invalid number of control points {}: for the degree {} \
                Bezier basis {} + n * {} are required, for n >= 0.",
                ncp, degree, degree + 1, degree);
            return vec![];
        }
        nsegments = (ncp - 1) / degree as usize;
    } else {
        if ncp < degree as usize + 1 {
            error!(
                "Invalid number of control points {}: for the degree {} \
                b-spline basis, must have >= {}.", ncp, degree, degree + 1);
            return vec![];
        }
        nsegments = ncp - degree as usize;
    }

    let curvety = params.find_one_string("type", "flat".to_owned());
    let ty = match curvety.as_str() {
        "flat"     => CurveType::Flat,
        "ribbon"   => CurveType::Ribbon,
        "culinder" => CurveType::Cylinder,
        _          => {
            error!("Unknown curve type \"{}\", Using \"cylinder\".", curvety);
            CurveType::Cylinder
        }
    };

    let mut nnorm = 0;
    let mut n = params.find_normal3f("N", &mut nnorm);

    if n.is_some() {
        if ty != CurveType::Ribbon {
            warn!("Curve normals are only used with \"ribbon\" type curves");
            n = None;
        } else if nnorm != nsegments + 1 {
            error!(
                "Invalid number of normals {}: must provide {} normals for ribbon \
                curves with {} segments", nnorm, nsegments + 1, nsegments);
            return vec![];
        }
    } else if ty == CurveType::Ribbon {
        error!("Must provice normals \"N\" at curve endpoints with ribbon curves.");
        return vec![];
    }

    let d = params.find_one_float("splitdepth", 3.0) as isize;
    let sd = params.find_one_int("splitdepth", d);
    let mut curves = vec![];
    let cp = cp.unwrap();
    let mut cpbase = 0;

    for seg in 0..nsegments {
        let mut seg_cp_bezier = [Point3f::default(); 4];
        // First, compute the cubic Bezier control points for the current
        // segment and store them in segCpBezier. (It is admittedly
        // wasteful storage-wise to turn b-splines into Bezier segments and
        // wasteful computationally to turn quadratic curves into cubics,
        // but yolo.)
        if basis == "bezier" {
            if degree == 2 {
                // Elevate to degree 3.
                seg_cp_bezier[0] = cp[cpbase];
                seg_cp_bezier[1] = lerp(2.0 / 3.0, cp[cpbase], cp[cpbase + 1]);
                seg_cp_bezier[2] = lerp(1.0 / 3.0, cp[cpbase + 1], cp[cpbase + 2]);
                seg_cp_bezier[3] = cp[cpbase + 2];
            } else {
                // Allset
                for i in 0..4 {
                    seg_cp_bezier[i] = cp[cpbase + i];
                }
            }

            cpbase += degree as usize;
        } else {
            // Uniform b-spline
            if degree == 2 {
                // First compute equivalent Bezier control points via some
                // blossiming.  We have three control points and a uniform
                // knot vector; we'll label the points p01, p12, and p23.
                // We want the Bezier control points of the equivalent
                // curve, which are p11, p12, and p22.
                let p01 = cp[cpbase];
                let p12 = cp[cpbase + 1];
                let p23 = cp[cpbase + 2];

                // We already have p12.
                let p11 = lerp(0.5, p01, p12);
                let p22 = lerp(0.5, p12, p23);

                // Now elevate to degree 3.
                seg_cp_bezier[0] = p11;
                seg_cp_bezier[1] = lerp(2.0 / 3.0, p11, p12);
                seg_cp_bezier[2] = lerp(1.0 / 3.0, p12, p22);
                seg_cp_bezier[3] = p22;
            } else {
                // Otherwise we will blossom from p012, p123, p234, and p345
                // to the Bezier control points p222, p223, p233, and p333.
                // https://people.eecs.berkeley.edu/~sequin/CS284/IMGS/cubicbsplinepoints.gif
                let p012 = cp[cpbase];
                let p123 = cp[cpbase + 1];
                let p234 = cp[cpbase + 2];
                let p345 = cp[cpbase + 3];

                let p122 = lerp(2.0 / 3.0, p012, p123);
                let p223 = lerp(1.0 / 3.0, p123, p234);
                let p233 = lerp(2.0 / 3.0, p123, p234);
                let p334 = lerp(1.0 / 3.0, p234, p345);

                let p222 = lerp(0.5, p122, p223);
                let p333 = lerp(0.5, p233, p334);

                seg_cp_bezier[0] = p222;
                seg_cp_bezier[1] = p223;
                seg_cp_bezier[2] = p233;
                seg_cp_bezier[3] = p333;
            }
            cpbase += 1;
        }

        let w0 = lerp(seg as Float / nsegments as Float, width0, width1);
        let w1 = lerp((seg + 1) as Float / nsegments as Float, width0, width1);
        let ns = if let Some(ref nn) = n {
            Some([nn[0], nn[1]])
        } else {
            None
        };
        let mut c = create_curve(
            o2w.clone(), w2o.clone(), reverse_orientation, &seg_cp_bezier,
            w0, w1, ty, ns, sd as usize);
        curves.append(&mut c);
    }

    curves
}