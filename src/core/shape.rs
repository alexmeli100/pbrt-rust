use crate::core::geometry::bounds::Bounds3f;
use crate::core::pbrt::{Float};
use crate::core::geometry::ray::Ray;
use enum_dispatch::enum_dispatch;
use crate::core::interaction::{SurfaceInteraction, Interaction, InteractionData};
use crate::core::geometry::point::{Point2f, Point3f};
use crate::core::geometry::vector::Vector3f;
use crate::core::transform::Transform;
use crate::shapes::sphere::Sphere;
use crate::shapes::cylinder::Cylinder;
use crate::shapes::disk::Disk;
use crate::shapes::triangle::Triangle;
use crate::shapes::curve::Curve;
use crate::shapes::cone::Cone;
use crate::shapes::paraboloid::Paraboloid;
use crate::core::lowdiscrepancy::radical_inverse;
use crate::shapes::hyperboloid::Hyperboloid;
use std::sync::Arc;

#[enum_dispatch]
pub trait Shape {
    fn object_bound(&self) -> Bounds3f;
    fn world_bound(&self) -> Bounds3f {
        self.object_to_world().transform_bounds(&self.object_bound())
    }
    fn intersect(
        &self, r: &Ray, t_hit: &mut Float,
        isect: &mut SurfaceInteraction,
        test_aphatexture: bool, s: Option<Arc<Shapes>>) -> bool;

    fn intersect_p(&self, r: &Ray, test_alpha_texture: bool) -> bool {
        let mut t_hit = r.t_max;
        let mut isect = SurfaceInteraction::default();

        self.intersect(r, &mut t_hit, &mut isect, test_alpha_texture, None)
    }

    fn area(&self) -> Float;
    fn sample(&self, u: &Point2f, pdf: &mut Float) -> InteractionData;
    fn sample_interaction(&self, i: &InteractionData, u: &Point2f, pdf: &mut Float) -> InteractionData {
        let intr = self.sample(u, pdf);
        let mut wi = intr.p - i.p;


        if wi.length_squared() == 0.0 {
            *pdf = 0.0;
        } else {
            wi = wi.normalize();
            // Convert from area measure, as returned by the Sample() call
            // above, to solid angle measure.
            *pdf *= i.p.distance_squared(&intr.p) / intr.n.abs_dot_vec(&(-wi));

            if pdf.is_infinite() { *pdf = 0.0 }
        }


        intr
    }
    fn pdf(&self, _i: &InteractionData) -> Float {
        1.0 / self.area()
    }

    fn pdf_wi(&self, re: &InteractionData, wi: &Vector3f) -> Float {
        // Intersect sample ray with area light geometry
        let ray = re.spawn_ray(wi);
        let mut thit = 0.0;
        let mut isect_light = SurfaceInteraction::default();

        // Ignore any alpha textures used for trimming the shape when performing
        // this intersection. Hack for the "San Miguel" scene, where this is used
        // to make an invisible area light.
        if !self.intersect(&ray, &mut thit, &mut isect_light, false, None) { return 0.0 }

        // Convert light sample weight to solid angle measure
        let mut pdf =
            re.p.distance_squared(&isect_light.p) /
            (isect_light.n.dot_vec(&(-*wi)) * self.area());

        if pdf.is_infinite() { pdf = 0.0; }

        pdf
    }

    fn solid_angle(&self, p: &Point3f, nsamples: usize) -> Float {
        let re = InteractionData {
            p: *p,
            time: 0.0,
            wo: Vector3f::new(0.0, 0.0, 1.0),
            medium_interface: Some(Default::default()),
            ..Default::default()
        };

        let mut solid_angle = 0.0f64;

        for i in 0..nsamples {
            let u = Point2f::new(radical_inverse(0, i as u64), radical_inverse(1, i as u64));
            let mut pdf = 0.0;
            let pshape = self.sample_interaction(&re, &u, &mut pdf);

            //println!("{}", pdf);

            let r = Ray::new(&p, &(pshape.p - *p), 0.999, 0.0, None, None);
            if pdf > 0.0 && !self.intersect_p(&r, true) {
                solid_angle += 1.0f64 / pdf as f64;
                //println!("{}: {}", i, pdf);
            }
        }

        (solid_angle / nsamples as f64) as Float
    }

    fn reverse_orientation(&self) -> bool;
    fn transform_swapshandedness(&self) -> bool;

    fn object_to_world(&self) -> Arc<Transform>;
    fn world_to_object(&self) -> Arc<Transform>;
}

pub fn shape_pdfwi<S: Shape>(shape: &S, re: &InteractionData, wi: &Vector3f) -> Float {
    // Intersect sample ray with area light geometry
    let ray = re.spawn_ray(wi);
    let mut thit = 0.0;
    let mut isect_light = SurfaceInteraction::default();

    // Ignore any alpha textures used for trimming the shape when performing
    // this intersection. Hack for the "San Miguel" scene, where this is used
    // to make an invisible area light.
    if !shape.intersect(&ray, &mut thit, &mut isect_light, false, None) { return 0.0 }

    // Convert light sample weight to solid angle measure
    let mut pdf =
        re.p.distance_squared(&isect_light.p) /
            (isect_light.n.dot_vec(&(-*wi)) * shape.area());

    if pdf.is_infinite() { pdf = 0.0; }

    pdf
}

#[enum_dispatch(Shape)]
pub enum Shapes {
    Cone(Cone),
    Sphere(Sphere),
    Cylinder(Cylinder),
    Disk(Disk),
    Curve(Curve),
    Triangle(Triangle),
    Hyperboloid(Hyperboloid),
    Paraboloid(Paraboloid)
}