# pbrt-rust

[![Build Status](https://github.com/alexmeli100/pbrt-rust/actions/workflows/rust.yml/badge.svg?branch=master)](https://github.com/alexmeli100/pbrt-rust/actions)
[![Github License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://github.com/alexmeli100/pbrt-rust/blob/master/LICENSE-APACHE)
[![Github License](https://img.shields.io/badge/License-MIT-yellow.svg)](https://github.com/alexmeli100/pbrt-rust/blob/master/LICENSE-MIT)

Another implementation of PBRT in Rust based on the PBRT book and the C++ version by Matt Pharr, Wenzel Jacob:

http://www.pbrt.org

## Usage
```shell
> cargo build --release
> ./target/release/pbrt-rust --help
pbrt-rust 0.1
Parse a PBRT scene description file and render it

USAGE:
    pbrt-rust.exe [FLAGS] [OPTIONS] <input>

FLAGS:
    -c, --cat            Print a reformatted version of the input file(s) to standard output. Does not render an image
    -h, --help           Prints help information
    -e, --logtostderr    Print all logging messages to stderr
    -t, --toply          Print a formatted version of input file(s) to standard output and convert all triangle meshes
                         to PLY files. Does not render and image
    -V, --version        Prints version information
    -v, --verbose        set LOG verbosity

OPTIONS:
    -w, --cropwindow <x0> <x1> <y0> <y1>    Specify an image crop window
    -l, --logdir <logdir>                   Specify directory that log files should be writtend to. Default: system temp
                                            directory (e.g $TMPDIR or /tmp)
    -n, --nthreads <nthreads>               Use specified number of threads for rendering [default: 0]
    -o, --outfile <outfile>                 Write the final image to the given filename

ARGS:
    <input>    Path to PBRT scene description file
```

## Example scenes
These are the first few scenes rendered using pbrt-rust. More scenes will be rendered as I fix the remaining bugs in the system if there are any

## First scene rendered in pbrt-rust

![Two spheres with glass and mirror material](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/spheres.png)

## Ganesha statue

![Ganesha statue](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/ganesha.png)

## Subsurface Scattering with dragon

![Subsurface Scattering](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/dragon.png)

## Stochastic Progressive Photon Mapping

![SPPM with caustic glass](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/glass.png)

## Country Kitchen by [Jay-Artist][jay-artist]

![Kitchen rendered with pbrt-rust](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/kitchen.png)

## The Wooden Staircase by [Wig42][wig42]

![Staircase rendered with pbrt-rust](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/staircase.png)

## Japanese Classroom by [NovaZeeke][novazeeke]

![Japanese classroom rendered with pbrt-rust](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/classroom.png)

## Ecosystem

![Cover image of first edition of PBRT rendered with pbrt-rust](https://github.com/alexmeli100/pbrt-rust/blob/master/rendered_scenes/ecosystem.png)

## TODO
- [ ] Render more scenes and fix any bugs
- [ ] Add more unit tests
- [ ] Do atleast some of the exercises in the book as most of them add new features to the system or improve performance
- [ ] Benchmark pbrt-rust against the C++ version
- [ ] Add a system profiler equivalent to the C++ version
- [ ] SIMD optimizations
- [ ] Add feature to create animations
- [ ] Update parts of the system to PBRTv4 when the book releases
- [ ] Implement parsing and rendering of blend files

## Other implementations
You can find other implementations of PBRT in rust here
* https://github.com/wahn/rs_pbrt (Very well documented)
* https://github.com/abusch/rustracer

## License

Licensed under either of

 * Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.



[jay-artist]:           https://www.blendswap.com/user/Jay-Artist
[wig42]:                https://www.blendswap.com/user/Wig42
[novazeeke]:            https://www.blendswap.com/user/NovaZeeke
