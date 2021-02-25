pub mod orthographic;
pub mod perspective;
pub mod environment;
pub mod realistic;

pub fn init_stats() {
    realistic::init_stats();
}