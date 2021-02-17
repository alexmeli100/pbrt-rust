
#[cfg(test)]
mod shapes {
    use pbrt_rust::core::rng::RNG;
    use pbrt_rust::core::pbrt::{Float, lerp, PI, INFINITY};
    use pbrt_rust::core::geometry::point::{Point3f, Point2f};
    use pbrt_rust::core::geometry::geometry::spherical_direction;
    use pbrt_rust::core::transform::Transform;
    use pbrt_rust::core::shape::{Shape, Shapes};
    use std::sync::Arc;
    use pbrt_rust::shapes::triangle::{create_trianglemesh, Triangle};
    use pbrt_rust::core::sampling::{uniform_sample_sphere, uniform_sphere_pdf};
    use pbrt_rust::core::geometry::ray::Ray;
    use pbrt_rust::core::interaction::{SurfaceInteraction, Interaction, InteractionData};
    use pbrt_rust::core::lowdiscrepancy::radical_inverse;
    use pbrt_rust::core::geometry::vector::Vector3f;
    use pbrt_rust::shapes::sphere::Sphere;
    use pbrt_rust::shapes::cylinder::Cylinder;
    use pbrt_rust::shapes::disk::Disk;
    use pbrt_rust::core::geometry::bounds::Bounds3f;
    use pbrt_rust::core::medium::phase_hg;
    use approx::relative_eq;
    use pbrt_rust::integrators::bdpt::VertexType::Surface;

    fn pexp(rng: &mut RNG, exp: Float) -> Float {
        let logu = lerp(rng.uniform_float(), -exp, exp);

        10.0f32.powf(logu)
    }

    fn punif(rng: &mut RNG, range: Float) -> Float {
        lerp(rng.uniform_float(), -range, range)
    }

    #[test]
    fn triangle_watertight() {
        let mut rng = RNG::new(12111);
        let ntheta = 16;
        let nphi = 16;

        // Make a triangle mesh representing a triangulated sphere (with
        // vertices randomly offset along their normal), centered at the
        // origin.
        let nvertices = ntheta * nphi;
        let mut vertices = Vec::with_capacity(nvertices);

        for t in 0..ntheta {
            let theta = PI * t as Float / (ntheta - 1) as Float;
            let cos_theta = theta.cos();
            let sin_theta = theta.sin();

            for p in 0..nphi {
                let phi = 2.0 * PI * p as Float / (nphi - 1) as Float;
                let mut radius = 1.0;
                // Make sure all of the top and bottom vertices are coincident.
                if t == 0 {
                    vertices.push(Point3f::new(0.0, 0.0, radius));
                } else if t == ntheta - 1 {
                    vertices.push(Point3f::new(0.0, 0.0, -radius));
                } else if t == nphi - 1 {
                    // Close it up exactly at the end
                    vertices.push(vertices[vertices.len() - (nphi - 1)]);
                } else {
                    radius += 5.0 * rng.uniform_float();
                    vertices.push(
                        Point3f::new(0.0, 0.0, 0.0) +
                            spherical_direction(sin_theta, cos_theta, phi)  * radius);
                }
            }
        }

        assert_eq!(nvertices, vertices.len());
        let mut indices = Vec::with_capacity(nvertices * 3);
        // fan at top
        let offset = |t: usize, p: usize| t * nphi + p;

        for p in 0..(nphi - 1) {
            indices.push(offset(0, 0));
            indices.push(offset(1, p));
            indices.push(offset(1, p + 1));
        }

        // quads in the middle rows
        for t in 1..(ntheta - 2) {
            for p in 0..(nphi - 1) {
                indices.push(offset(t, p));
                indices.push(offset(t + 1, p));
                indices.push(offset(t + 1, p + 1));

                indices.push(offset(t, p));
                indices.push(offset(t + 1, p + 1));
                indices.push(offset(t, p + 1));
            }
        }

        // fan at bottom
        for p in 0..nphi - 1 {
            indices.push(offset(ntheta - 1, 0));
            indices.push(offset(ntheta - 2, p));
            indices.push(offset(ntheta - 2, p + 1));
        }

        let identity = Arc::new(Transform::default());
        let tris = create_trianglemesh(
            identity.clone(), identity, false,
            indices.len() / 3, indices, nvertices, vertices.clone(),
            vec![], vec![], vec![], None, None);

        for i in 0..100000 {
            let mut r = RNG::new(i);
            // Choose a random point in sphere of radius 0.5 around the origin.
            let mut u = Point2f::default();
            u[0] = r.uniform_float();
            u[1] = r.uniform_float();
            let p = Point3f::default() + uniform_sample_sphere(&u) * 0.5;

            //println!("{}", p);

            // Choose a random direction
            u[0] = r.uniform_float();
            u[1] = r.uniform_float();
            let mut ray = Ray::new(&p, &uniform_sample_sphere(&u), INFINITY, 0.0, None, None);
            let mut nhits = 0;

            for tri in tris.iter() {
                let mut thit = 0.0;
                let mut isect = SurfaceInteraction::default();

                if tri.intersect(&ray, &mut thit, &mut isect, false, Some(tri.clone())) { nhits += 1; }
            }

            assert!(nhits >= 1, "Failed at {}", i);

            // Now tougher: shoot directly at a vertex.
            let pvertex = vertices[r.uniform_int32_2(vertices.len() as u32) as usize];
            //println!("{}", pvertex);
            ray.d = pvertex - ray.o;
            nhits = 0;

            for tri in tris.iter() {
                let mut thit = 0.0;
                let mut isect = SurfaceInteraction::default();

                if tri.intersect(&ray, &mut thit, &mut isect, false, Some(tri.clone())) { nhits += 1; }
            }

            assert!(nhits >= 1, "Failed at {}", i);
        }
    }

    fn get_random_trianlge<F: FnMut() -> Float>(mut value: F) -> Option<Arc<Shapes>> {
        let mut v = [Point3f::default(); 3];

        for j in 0..3 {
            for k in 0..3 {
                v[j][k] = value();
            }
        }

        if ((v[1] - v[0]).cross(&(v[2] - v[0]))).length_squared() < 1.0e-20 {
            return None;
        }

        let identity = Arc::new(Transform::default());
        let indices = vec![0, 1, 2];
        let trivec = create_trianglemesh(
            identity.clone(), identity, false,
            1, indices, 3, v.to_vec(), vec![],
            vec![], vec![], None, None);

        assert_eq!(1, trivec.len());

        Some(trivec[0].clone())
    }

    #[test]
    fn triangle_reintersect() {
        for i in 0..1000 {
            let mut rng = RNG::new(i);
            let tri = get_random_trianlge(|| pexp(&mut rng, 8.0));

            if tri.is_none() { continue; }
            let tr = tri.unwrap();

            let mut u = Point2f::default();
            u[0] = rng.uniform_float();
            u[1] = rng.uniform_float();
            let mut pdf = 0.0;
            let ptri = tr.sample(&u, &mut pdf);

            // Choose ray origin
            let mut o = Point3f::default();
            for j in 0..3 { o[j] = pexp(&mut rng, 8.0) }

            // Intersect the ray with the triangle.
            let r = Ray::new(&o, &(ptri.p - o), INFINITY, 0.0,  None, None);
            let mut thit = 0.0;
            let mut isect = SurfaceInteraction::default();

            if !tr.intersect(&r, &mut thit, &mut isect, false, Some(tr.clone())) { continue }

            // Now trace a bunch of rays leaving the intersection point
            for j in 0..10000 {
                // Random direction leaving the intersection point.
                let mut u = Point2f::default();
                u[0] = rng.uniform_float();
                u[1] = rng.uniform_float();
                let w = uniform_sample_sphere(&u);
                let mut rout = isect.spawn_ray(&w);
                assert!(!tr.intersect_p(&rout, true));

                let mut thit = 0.0;
                assert!(!tr.intersect(&rout, &mut thit, &mut isect, false, Some(tr.clone())));

                // Choose a random point to trace rays to
                let mut p2 = Point3f::default();
                for k in 0..3 {
                    p2[k] = pexp(&mut rng, 8.0);
                }

                rout = isect.spawn_rayto_point(&p2);

                assert!(!tr.intersect_p(&rout, true));
                assert!(!tr.intersect(&rout, &mut thit, &mut isect, false, Some(tr.clone())));
            }
        }
    }

    #[test]
    fn triangle_sampling() {
        for i in 0..30 {
            let range = 10.0;
            let mut rng = RNG::new(i);
            let tropt = get_random_trianlge(|| punif(&mut rng, range));

            if tropt.is_none() { continue; }
            let tri = tropt.unwrap();

            // Ensure that the reference point isn't too close to the
            // triangle's surface (which makes the Monte Carlo stuff have more
            // variance, thus requiring more samples).
            let mut pc = Point3f::new(punif(&mut rng, range), punif(&mut rng, range), punif(&mut rng, range));
            let idx = rng.uniform_int32() % 3;

            pc[idx as usize] = if rng.uniform_float() > 0.5 {
                (-range - 3.0)
            } else {
                range + 3.0
            };

            // Compute reference value using Monte Carlo with uniform spherical
            // sampling.
            let count = 512 * 1024;
            let mut hits = 0;

            for j in 0..count {
                let u = Point2f::new(radical_inverse(0, j), radical_inverse(1, j));
                let w = uniform_sample_sphere(&u);
                let ray = Ray::new(&pc, &w, INFINITY, 0.0, None, None);
                if tri.intersect_p(&ray, true) { hits += 1; }
            }

            println!("{}", hits);

            let unif_estimate = hits as f64 / (count as f64 * uniform_sphere_pdf() as f64);

            let refi = InteractionData::new(
                pc, Default::default(), Default::default(),
                Vector3f::new(0.0, 0.0, 1.0), 0.0, Default::default());
            let mut tri_sample_estimate = 0.0f64;

            for j in 0..count {
                let u = Point2f::new(radical_inverse(0, j), radical_inverse(1, j));
                let mut pdf = 0.0;
                let _ = tri.sample_interaction(&refi, &u, &mut pdf);
                assert!(pdf > 0.0);
                tri_sample_estimate += 1.0 / (count as f64 * pdf as f64);
            }

            // Now make sure that the two computed solid angle values are
            // fairly close.
            // Absolute error for small solid angles, relative for large.
            let error = |a: Float, b: Float| -> Float {
                if a.abs() < 1.0e-4 || b.abs() < 1.0e-4 {
                    return (a - b).abs();
                }

                ((a - b) / b).abs()
            };

            // Don't compare really small triangles, since uniform sampling
            // doesn't get a good estimate for them.
            if tri_sample_estimate > 1.0e-3 {
                // The error tolerance is fairly large so that we can use a
                // reasonable number of samples.  It has been verified that for
                // larger numbers of Monte Carlo samples, the error continues to
                // tighten.
                assert!(
                    error(tri_sample_estimate as Float, unif_estimate as Float) < 0.1,
                    "Unif sampling: {}, triangle sampling: {}, tri index: {}",
                    unif_estimate, tri_sample_estimate, i);
            }
        }
    }

    #[test]
    fn triangle_solid_angle() {
        for i in 0..50 {
            let range = 10.0;
            let mut rng = RNG::new(100 + i);
            let tropt = get_random_trianlge(|| punif(&mut rng, range));

            if tropt.is_none() { continue; }
            let tri = tropt.unwrap();

            // triangle's surface (which makes the Monte Carlo stuff have more
            // variance, thus requiring more samples).
            let mut pc = Point3f::new(punif(&mut rng, range), punif(&mut rng, range), punif(&mut rng, range));
            let idx = rng.uniform_int32() % 3;

            pc[idx as usize] = if rng.uniform_float() > 0.5 {
                (-range - 3.0)
            } else {
                range + 3.0
            };

            let count = 64 * 1024;
            let refi = InteractionData::new(
                pc, Default::default(), Default::default(),
                Vector3f::new(0.0, 0.0, 1.0), 0.0, Default::default());
            let mut tri_sample_estimate = 0.0f64;

            for j in 0..count {
                let u = Point2f::new(radical_inverse(0, j), radical_inverse(1, j));
                let mut pdf = 0.0;
                let _ = tri.sample_interaction(&refi, &u, &mut pdf);
                assert!(pdf > 0.0);
                tri_sample_estimate += 1.0 / (count as f64 * pdf as f64);
            }

            let error = |a: Float, b: Float| -> Float {
                if a.abs() < 1.0e-4 || b.abs() < 1.0e-4 {
                    return (a - b).abs();
                }

                ((a - b) / b).abs()
            };

            // Now compute the subtended solid angle of the triangle in closed
            // form.
            let spherical_area = tri.solid_angle(&pc, 0);
            assert!(
                error(spherical_area, tri_sample_estimate as Float) < 0.015,
                "spherical area: {}, tri sampling: {}, tri index: {}",
                spherical_area, tri_sample_estimate, i);
        }
    }

    fn mc_solid_angle(p: &Point3f, shape: &Shapes, nsamples: usize) -> Float {
        let mut nhits = 0;

        for i in 0..nsamples {
            let u = Point2f::new(
                radical_inverse(0, i as u64),
                radical_inverse(1, i as u64));
            let w = uniform_sample_sphere(&u);
            let ray = Ray::new(p, &w, INFINITY, 0.0, None, None);
            if shape.intersect_p(&ray, false) { nhits += 1; }
        }

        //println!("{}", nhits);

        nhits as Float / (uniform_sphere_pdf() * nsamples as Float)
    }

    #[test]
    fn sphere_solid_angle() {
        let tr = Transform::translate(&Vector3f::new(1.0, 0.5, -0.8)) * Transform::rotate_x(30.0);
        let tr_inv = Transform::inverse(&tr);
        let sphere: Shapes = Sphere::new(
            Arc::new(tr), Arc::new(tr_inv),
            false, 1.0, -1.0, 1.0, 360.0).into();

        // Make sure we get a subtended solid angle of 4pi for a point
        // inside the sphere.
        let pinside = Point3f::new(1.0, 0.9, -0.8);
        let nsamples = 128 * 1024;
        assert!(mc_solid_angle(&pinside, &sphere, nsamples).abs() - 4.0 * PI < 0.01);
        assert!((sphere.solid_angle(&pinside, nsamples)).abs() - 4.0 * PI < 0.01);

        // Now try a point out side the sphere
        let p = Point3f::new(-1.25, -1.0, 0.8);
        let mcsa = mc_solid_angle(&p, &sphere, nsamples);
        let sphere_sa = sphere.solid_angle(&p, nsamples);
        assert!((mcsa - sphere_sa).abs() < 0.001);
    }

    #[test]
    fn cylinder_solid_angle() {
        let tr = Transform::translate(&Vector3f::new(1.0, 0.5, -0.8)) * Transform::rotate_x(30.0);
        let tr_inv = Transform::inverse(&tr);
        let cyl: Shapes = Cylinder::new(
            Arc::new(tr), Arc::new(tr_inv),
            false, 0.25, -1.0, 1.0, 360.0).into();
        let p = Point3f::new(0.5, 0.25, 0.5);
        let nsamples = 128 * 1024;
        let solid_angle = mc_solid_angle(&p, &cyl, nsamples);
        let csa = cyl.solid_angle(&p, nsamples);


        assert!((solid_angle - csa).abs() < 0.001);
    }

    #[test]
    fn disk_solid_angle() {
        let tr = Transform::translate(&Vector3f::new(1.0, 0.5, -0.8)) * Transform::rotate_x(30.0);
        let tr_inv = Transform::inverse(&tr);
        let disk: Shapes = Disk::new(
            Arc::new(tr), Arc::new(tr_inv),
            false, 0.0, 1.25, 0.0, 360.0).into();
        let p = Point3f::new(0.5, -0.8, 0.5);
        let nsamples = 128 * 1024;
        let solid_angle = mc_solid_angle(&p, &disk, nsamples);
        let dsa = disk.solid_angle(&p, nsamples);
        assert!((solid_angle - dsa).abs() < 0.001);
    }

    fn test_reintersect_convex(shape: &Arc<Shapes>, rng: &mut RNG) {
        let mut o = Point3f::default();
        for i in 0..3 { o[i] = pexp(rng, 8.0); }

        // Destination: a random point in the shape's bounding box
        let bbox: Bounds3f = shape.world_bound();
        let mut t = Point3f::default();
        for c in 0..3 { t[c] = rng.uniform_float(); }
        let p2 = bbox.lerp(&t);

        // Ray to intersect with the shape
        let mut r = Ray::new(&o, &(p2 - o), INFINITY, 0.0, None, None);
        if rng.uniform_float() < 0.5 { r.d = r.d.normalize(); }

        // We should usually (but not always) find an intersection.
        let mut isect = SurfaceInteraction::default();
        let mut thit = 0.0;
        if !shape.intersect(&r, &mut thit, &mut isect, false, Some(shape.clone())) { return; }

        // Now trace a bunch of rays leaving the intersection point
        for j in 0..10000 {
            // Random direction leaving the intersection point.
            let mut u = Point2f::default();
            u[0] = rng.uniform_float();
            u[1] = rng.uniform_float();
            let mut w = uniform_sample_sphere(&u);
            //println!("{:?}", isect.n);
            // Make sure it's in the same hemisphere as the surface normal
            w = w.face_foward_norm(&isect.n);

            let mut rout = isect.spawn_ray(&w);
            //println!("{:?}, {:?}", rout.o, rout.d);
            assert!(!shape.intersect_p(&rout, false));

            let mut spawn_isect = SurfaceInteraction::default();
            let mut spawnhit = 0.0;
            assert!(!shape.intersect(&rout, &mut spawnhit, &mut spawn_isect, false, Some(shape.clone())));

            // Choose random point to trace rays to
            let mut p3 = Point3f::default();
            for i in 0..3 { p3[i] = pexp(rng, 8.0); }
            // Make sure that the point we're tracing rays toward is in the
            // hemisphere about the intersection point's surface normal.
            w = p3 - isect.p;
            w = w.face_foward_norm(&isect.n);
            p3 = isect.p + w;
            rout = isect.spawn_rayto_point(&p3);

            assert!(!shape.intersect_p(&rout, false));
            assert!(!shape.intersect(&rout, &mut thit, &mut isect, false, Some(shape.clone())));
        }
    }

    #[test]
    fn full_sphere_reintersect() {
        for i in 0..100 {
            let mut rng = RNG::new(i);
            let iden = Arc::new(Transform::default());
            let radius = pexp(&mut rng, 4.0);
            let zmin = -radius;
            let zmax = radius;
            let phi_max = 360.0;
            let sphere: Arc<Shapes> = Arc::new(Sphere::new(
                iden.clone(), iden.clone(), false,
                radius, zmin, zmax, phi_max).into());

            test_reintersect_convex(&sphere, &mut rng);
        }
    }

    #[test]
    fn partial_sphere_normal() {
        for i in 0..10000 {
            let mut rng = RNG::new(i);
            let iden = Arc::new(Transform::default());
            let radius = pexp(&mut rng, 4.0);
            let zmin = if rng.uniform_float() < 0.5 {
                -radius
            } else {
                lerp(rng.uniform_float(), -radius, radius)
            };
            let zmax = if rng.uniform_float() < 0.5 {
                radius
            } else {
                lerp(rng.uniform_float(), -radius, radius)
            };
            let phimax = if rng.uniform_float() < 0.5 {
                360.0
            } else {
                rng.uniform_float() * 360.0
            };
            let sphere: Arc<Shapes> = Arc::new(Sphere::new(
                iden.clone(), iden.clone(), false,
                radius, zmin, zmax, phimax).into());

            // Ray origin
            let mut o = Point3f::default();
            for i in 0..3 { o[i] = pexp(&mut rng, 8.0); }

            // Destination: a random point in the shape's bounding box
            let bbox: Bounds3f = sphere.world_bound();
            let mut t = Point3f::default();
            for c in 0..3 { t[c] = rng.uniform_float(); }
            let p2 = bbox.lerp(&t);

            // Ray to intersect with the shape
            let mut r = Ray::new(&o, &(p2 - o), INFINITY, 0.0, None, None);
            if rng.uniform_float() < 0.5 { r.d = r.d.normalize(); }

            // We should usually (but not always) find an intersection.
            let mut isect = SurfaceInteraction::default();
            let mut thit = 0.0;
            if !sphere.intersect(&r, &mut thit, &mut isect, false, Some(sphere.clone())) { continue; }

            let dot = isect.n.normalize().dot_vec(&Vector3f::from(isect.p).normalize());
            relative_eq!(1.0, dot, epsilon = 1.0e-5);
        }
    }

    #[test]
    fn partial_sphere_reintersect() {
        for i in 0..100 {
            let mut rng = RNG::new(i);
            let iden = Arc::new(Transform::default());
            let radius = pexp(&mut rng, 4.0);
            let zmin = if rng.uniform_float() < 0.5 {
                -radius
            } else {
                lerp(rng.uniform_float(), -radius, radius)
            };
            let zmax = if rng.uniform_float() < 0.5 {
                radius
            } else {
                lerp(rng.uniform_float(), -radius, radius)
            };
            let phimax = if rng.uniform_float() < 0.5 {
                360.0
            } else {
                rng.uniform_float() * 360.0
            };
            let sphere: Arc<Shapes> = Arc::new(Sphere::new(
                iden.clone(), iden.clone(), false,
                radius, zmin, zmax, phimax).into());

            test_reintersect_convex(&sphere, &mut rng);
        }
    }

    #[test]
    fn cylinder_reintersect() {
        for i in 0..100 {
            let mut rng = RNG::new(i);
            let iden = Arc::new(Transform::default());
            let radius = pexp(&mut rng, 4.0);
            let zmin = pexp(&mut rng, 4.0) * if rng.uniform_float() < 0.5 { -1.0 } else { 1.0 };
            let zmax = pexp(&mut rng, 4.0) * if rng.uniform_float() < 0.5 { -1.0 } else { 1.0 };
            let phimax = if rng.uniform_float() < 0.5 { 360.0 } else { rng.uniform_float() * 360.0 };

            let cyl: Arc<Shapes> = Arc::new(Cylinder::new(
                iden.clone(), iden, false,
                radius, zmin, zmax, phimax).into());

            test_reintersect_convex(&cyl, &mut rng)
        }

    }

    #[test]
    fn triangle_badcases() {
        let indices = vec![0, 1, 2];
        let iden = Arc::new(Transform::default());
        let p = vec![
            Point3f::new( -1113.45459, -79.049614, -56.2431908),
            Point3f::new(-1113.45459, -87.0922699, -56.2431908),
            Point3f::new(-1113.45459, -79.2090149, -56.2431908)
        ];
        let mesh = create_trianglemesh(
            iden.clone(), iden, false, 1, indices,
                3, p, vec![], vec![], vec![], None, None);

        let ray = Ray::new(
            &Point3f::new(-1081.47925, 99.9999542, 87.7701111),
            &Vector3f::new(-32.1072998, -183.355865, -144.607635),
            0.9999, 0.0, None, None);
        let mut thit = 0.0;
        let mut isect = SurfaceInteraction::default();

        assert!(!mesh[0].intersect(&ray, &mut thit, &mut isect, true, Some(mesh[0].clone())));
    }
}

