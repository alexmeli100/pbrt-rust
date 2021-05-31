#![feature(iter_partition_in_place)]
#![feature(slice_partition_at_index)]
#![feature(const_fn)]
#![allow(incomplete_features)]
#![feature(const_evaluatable_checked)]
#![feature(const_generics)]
#![feature(const_fn_trait_bound)]

// lints
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]


// clippy
#![cfg_attr(
    feature = "cargo-clippy",
    allow(
        clippy::upper_case_acronyms,
        clippy::many_single_char_names,
        clippy::too_many_arguments,
        clippy::excessive_precision,
        clippy::float_cmp
    )
)]

pub mod core;
pub mod materials;
pub mod shapes;
pub mod accelerators;
pub mod cameras;
pub mod samplers;
pub mod textures;
pub mod filters;
pub mod pbrtparser;
pub mod media;
pub mod lights;
pub mod integrators;

pub fn init_stats() {
    core::stats::init_stats();
    core::api::init_stats();
    core::integrator::init_stats();
    core::lightdistrib::init_stats();
    core::mipmap::init_stats();
    core::scene::init_stats();
    accelerators::init_stats();
    cameras::init_stats();
    shapes::init_stats();
    integrators::init_stats();
    media::init_stats();
}


