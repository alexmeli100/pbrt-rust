pub mod directlighting;
pub mod path;
pub mod volpath;
pub mod whitted;
pub mod sppm;
pub mod bdpt;
pub mod mlt;
pub mod ao;

pub fn init_stats() {
    sppm::init_stats();
    bdpt::init_stats();
    mlt::init_stats();
    path::init_stats();
    volpath::init_stats();
}