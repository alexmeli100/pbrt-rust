
mod test_scenes {
    use pbrt_rust::core::transform::{Transform, AnimatedTransform};
    use std::sync::Arc;
    use pbrt_rust::core::shape::Shapes;
    use pbrt_rust::shapes::sphere::Sphere;
    use pbrt_rust::core::texture::{TextureFloat, TextureSpec};
    use pbrt_rust::textures::constant::ConstantTexture;
    use pbrt_rust::core::material::Materials;
    use pbrt_rust::materials::matte::MatteMaterial;
    use pbrt_rust::core::spectrum::Spectrum;
    use pbrt_rust::core::medium::MediumInterface;
    use pbrt_rust::core::primitive::{Primitives, GeometricPrimitive};
    use pbrt_rust::accelerators::bvh::{BVHAccel, SplitMethod};
    use pbrt_rust::core::light::Lights;
    use pbrt_rust::lights::point::PointLight;
    use pbrt_rust::core::pbrt::{PI, INFINITY};
    use pbrt_rust::core::scene::Scene;
    use pbrt_rust::samplers::zerotwosequence::ZeroTwoSequenceSampler;
    use pbrt_rust::core::sampler::Samplers;
    use pbrt_rust::core::geometry::point::{Point2i, Point2f, Point3f};
    use pbrt_rust::core::filter::Filters;
    use pbrt_rust::filters::boxfilter::BoxFilter;
    use pbrt_rust::core::geometry::vector::Vector2f;
    use pbrt_rust::core::film::Film;
    use pbrt_rust::core::geometry::bounds::Bounds2f;
    use std::path::PathBuf;
    use pbrt_rust::init_stats;
    use pbrt_rust::cameras::perspective::PerspectiveCamera;
    use pbrt_rust::core::camera::Cameras;
    use pbrt_rust::integrators::path::PathIntegrator;
    use pbrt_rust::integrators::directlighting::{DirectLightingIntegrator, LightStrategy};
    use pbrt_rust::core::integrator::SamplerIntegrator;
    use pbrt_rust::materials::glass::GlassMaterial;
    use pbrt_rust::lights::distant::DistantLight;

    #[test]
    fn test_scene() {
        init_stats();
        let id = Arc::new(Transform::default());
        let sphere: Arc<Shapes> = Arc::new(Sphere::new(
            id.clone(), id.clone(),
            true, 1.0, -1.0, 1.0, 360.0).into());
        let kr: Arc<TextureSpec> = Arc::new(ConstantTexture::new(Spectrum::new(1.0)).into());
        let kt: Arc<TextureSpec> = Arc::new(ConstantTexture::new(Spectrum::new(1.0)).into());
        let eta: Arc<TextureFloat> = Arc::new(ConstantTexture::new(1.5).into());
        let roughu: Arc<TextureFloat> = Arc::new(ConstantTexture::new(0.0).into());
        let roughv: Arc<TextureFloat> = Arc::new(ConstantTexture::new(0.0).into());
        let mat: Arc<Materials> = Arc::new(GlassMaterial::new(
            kr, kt, roughu, roughv, eta, None,
        true).into());
        let mi = MediumInterface::default();
        let mut prims: Vec<Arc<Primitives>> = Vec::new();
        let gprim: Arc<Primitives> =
            Arc::new(GeometricPrimitive::new(sphere, Some(mat), None, mi).into());
        prims.push(gprim);
        let bvh: Arc<Primitives> = Arc::new(BVHAccel::new(
            prims, 1, SplitMethod::SAH).into());
        let mut lights: Vec<Arc<Lights>> = Vec::new();
        let from = Point3f::new(0.0, 10.0, 0.0);
        let to = Point3f::default();
        let wlight = from - to;
        let l: Arc<Lights> = Arc::new(DistantLight::new(
            &Transform::default(), &Spectrum::new(3.141593),
            &wlight).into());
        lights.push(l);
        let scene = Scene::new(bvh, lights);
        let sampler: Samplers = ZeroTwoSequenceSampler::new(
            1, 4).into();
        let resolution = Point2i::new(10, 10);
        let identity = AnimatedTransform::new(
            Default::default(), Default::default(),
            0.0, 1.0);
        let filter: Filters = BoxFilter::new(Vector2f::new(0.5, 0.5)).into();
        let crop = Bounds2f::new(&Point2f::new(0.0, 0.0), &Point2f::new(1.0, 1.0));
        let out = PathBuf::from("test.exr");
        let film = Film::new(
            &resolution, &crop, filter, 1.0,
            out, 1.0, INFINITY);
        let swindow = Bounds2f::new(&Point2f::new(-1.0, -1.0), &Point2f::new(1.0, 1.0));
        let bounds = film.cropped_pixel_bounds;
        let camera: Cameras = PerspectiveCamera::new(
            identity, &swindow, 0.0,
            1.0, 0.0,10.0, 45.0, Arc::new(film), None).into();
        let mut integrator = DirectLightingIntegrator::new(
            LightStrategy::UniformSampleAll, 8, Arc::new(camera),
            Box::new(sampler), &bounds);

        integrator.render(&scene);

    }
}