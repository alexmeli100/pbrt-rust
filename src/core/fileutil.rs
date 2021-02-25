use lazy_static::lazy_static;
use std::sync::Mutex;
use std::path::{Path, PathBuf};
use log::debug;


lazy_static! {
    static ref SEARCH_DIR: Mutex<Option<PathBuf>> = Mutex::new(None);
}

pub fn set_search_directory<P: AsRef<Path>>(dir: P) {
    let mut sdir = SEARCH_DIR.lock().unwrap();
    sdir.get_or_insert(PathBuf::from(dir.as_ref()));
    debug!("Set search directory to {}", dir.as_ref().display());

}

pub fn absolute_path(filename: &str) -> String {
    let path = PathBuf::from(filename);
    
    path.canonicalize()
        .ok()
        .and_then(|p| p.to_str().map(|res| res.to_owned()))
        .unwrap_or_else(|| filename.to_owned())
}

pub fn directory_containing<P: AsRef<Path>>(path: P) -> PathBuf {
    let path = path.as_ref();
    let res = path
        .canonicalize()
        .unwrap()
        .parent()
        .unwrap_or_else(|| {
            panic!("{}",
                format!("Failed to get parent directory of input file {}",
                path.display()))
        })
        .to_owned();
    res
}

pub fn resolve_filename(filename: &str) -> String {
    let sdir = SEARCH_DIR.lock().unwrap();
    let p = Path::new(filename);

    if sdir.is_none() || filename.is_empty() ||p.is_absolute() {
        return filename.to_owned()
    }

    let mut pp = (*sdir).clone().unwrap();
    pp.push(filename);

    pp
        .as_path()
        .canonicalize()
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