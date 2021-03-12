#[cfg(test)]
mod plymesh {
    use pbrt_rust::core::paramset::{ParamSet, ParamSetItem};
    use std::cell::Cell;
    use pbrt_rust::core::transform::Transform;
    use std::sync::Arc;
    use pbrt_rust::shapes::plymesh::create_plymesh;
    use std::collections::HashMap;

    #[test]
    fn test_plymesh() {
        let mut params = ParamSet::default();
        let file = "C:\\Users\\alexm\\Documents\\code\\Rust\\pbrt-rust\\src\\scenes\\geometry\\mesh_00001.ply".to_owned();
        let f = ParamSetItem {
            name: "filename".to_string(),
            values: vec![file],
            nvalues: 1,
            looked_up: Cell::new(false)
        };
        params.strings.push(f);
        let textures = HashMap::new();
        let iden = Arc::new(Transform::default());
        let triangles = create_plymesh(
            iden.clone(), iden.clone(),
            true, &params, &textures);

    }
}