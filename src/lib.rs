#![feature(iter_partition_in_place)]
#![feature(slice_partition_at_index)]
#![feature(const_fn)]
#![allow(incomplete_features)]
#![feature(const_evaluatable_checked)]
#![feature(const_generics)]

// lints
#![allow(non_snake_case)]
#![allow(non_upper_case_globals)]


// clippy
#![cfg_attr(
    feature = "cargo-clippy",
    allow(
        clippy::upper_case_acronyms,
        clippy::many_single_char_names,
        clippy::too_many_arguments
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


