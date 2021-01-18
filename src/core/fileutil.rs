use lazy_static::lazy_static;
use std::sync::Mutex;
use std::path::{Path, PathBuf};


lazy_static! {
    static ref SEARCH_DIR: Mutex<Option<String>> = Mutex::new(None);
}

pub fn absolute_path(filename: &str) -> String {
    let path = PathBuf::from(filename);
    
    path.canonicalize()
        .ok()
        .and_then(|p| p.to_str().map(|res| res.to_owned()))
        .unwrap_or_else(|| filename.to_owned())
}

pub fn resolve_filename(filename: &str) -> String {
    let sdir = SEARCH_DIR.lock().unwrap();
    let p = Path::new(filename);

    if sdir.is_none() || filename.is_empty() ||p.is_absolute() {
        return filename.to_owned()
    }

    let pp: PathBuf = (*sdir).clone().unwrap().into();

    pp.canonicalize()
        .ok()
        .and_then(|p| p.to_str().map(|res| res.to_owned()))
        .unwrap_or_else(|| filename.to_owned())
}

pub fn has_extension<P: AsRef<Path>>(name: P, ext: &str) -> bool {
    name
        .as_ref()
        .extension()
        .map(|x| x == ext)
        .unwrap_or(false)
}