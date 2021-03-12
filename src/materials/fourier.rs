use crate::core::reflection::{FourierBSDFTable, BSDF, FourierBSDF};
use std::sync::Arc;
use std::sync::Mutex;
use std::collections::HashMap;
use bumpalo_herd::Member;
use lazy_static::lazy_static;
use crate::core::texture::{TextureFloat};
use crate::core::material::{Material, TransportMode, bump, Materials};
use crate::core::interaction::SurfaceInteraction;
use crate::core::paramset::TextureParams;

lazy_static! {
    static ref LOADED_BSDFS: Mutex<HashMap<String, Arc<FourierBSDFTable>>> = Mutex::new(HashMap::new());
}

pub struct FourierMaterial {
    table   : Arc<FourierBSDFTable>,
    bump_map: Option<Arc<TextureFloat>>
}

impl FourierMaterial {
    pub fn new(filename: &str, bump_map: Option<Arc<TextureFloat>>) -> Self {
        let mut loaded = LOADED_BSDFS.lock().unwrap();

        if !loaded.contains_key(filename) {
            let mut table = FourierBSDFTable::default();
            FourierBSDFTable::read(filename, &mut table);
            loaded.insert(filename.to_owned(), Arc::new(table));
        }

        Self {
            bump_map,
            table: loaded.get(filename).unwrap().clone()
        }
    }
}

impl Material for FourierMaterial {
    fn compute_scattering_functions<'b: 'b>(
        &self, si: &mut SurfaceInteraction<'b>, arena: & Member<'b>,
        _mat: Option<Arc<Materials>>, mode: TransportMode, _allow_multiple_lobes: bool) {
        // Perform bump mapping with bumpMap if present
        if self.bump_map.is_some() { bump(self.bump_map.as_ref().unwrap(), si); }

        let mut bsdf = BSDF::new(si, 1.0);

        // Checking for zero channels works as a proxy for checking whether the
        // table was successfully read from the file
        if self.table.nchannels > 0 {
            bsdf.add(arena.alloc(FourierBSDF::new(self.table.clone(), mode).into()));
        }

        si.bsdf = Some(bsdf)
    }
}

pub fn create_fourier_material(mp: &mut TextureParams) -> Materials {
    let bump_map = mp.get_floattexture_ornull("bumpmap");
    let filename = mp.find_filename("bsdffile", "");

    FourierMaterial::new(&filename, bump_map).into()
}