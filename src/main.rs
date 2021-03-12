use structopt::StructOpt;
use pbrt_rust::core::pbrt::{Float, Options, get_progress_bar};
use pbrt_rust::init_stats;
use std::path::PathBuf;
use num_cpus;
use anyhow::Result;
use fern::colors::{ColoredLevelConfig, Color};
use pbrt_rust::pbrtparser::pbrtparser::pbrt_parse;
use fern::Output;
use std::io::Write;

#[derive(StructOpt, Debug)]
#[structopt(name = "pbrt")]
struct Args {
    /// set LOG verbosity
    #[structopt(short, long)]
    verbose: bool,

    /// Specify directory that log files should be writtend to.
    /// Default: system temp directory (e.g $TMPDIR or /tmp).
    #[structopt(short, long)]
    logdir: Option<PathBuf>,

    /// Print all logging messages to stderr
    #[structopt(short = "e", long)]
    logtostderr: bool,

    /// Print a reformatted version of the input file(s)
    /// to standard output. Does not render an image.
    #[structopt(short, long)]
    cat: bool,

    /// Print a formatted version of input file(s) to
    /// standard output and convert all triangle meshes to
    /// PLY files. Does not render and image.
    #[structopt(short, long)]
    toply: bool,

    /// Use specified number of threads for rendering
    #[structopt(short, long, default_value = "0")]
    nthreads: u8,

    /// Specify an image crop window
    #[structopt(short = "w", long, number_of_values = 4, value_names = &["x0", "x1", "y0", "y1"])]
    cropwindow: Option<Vec<Float>>,

    #[structopt(short, long, parse(from_os_str))]
    /// Write the final image to the given filename
    outfile: Option<PathBuf>,

    #[structopt(parse(from_os_str))]
    /// Path to PBRT scene description file
    input: PathBuf
}

fn setup_logging(verbose: bool, logdir: PathBuf, stderr: bool) -> Result<()> {
    let colors = ColoredLevelConfig::new()
        .error(Color::Red)
        .warn(Color::Yellow);
    let clevel = colors.clone().info(Color::Green);

    let mut base_config = fern::Dispatch::new();

    let level = if verbose {
        log::LevelFilter::Debug
    } else {
        log::LevelFilter::Info
    };

    base_config = base_config.level(level);

    let file_config = fern::Dispatch::new()
        .format(|out, message, record| {
            out.finish(format_args!(
                "[{}] {}",
                record.level(),
                message
            ))
        })
        .chain(fern::log_file(logdir)?);

    let stderr_config = fern::Dispatch::new()
        .format(move |out, message, record| {
            out.finish(format_args!(
                "{color_line}[{level}] {message}\x1B[0m",
                color_line = format_args!("\x1B[{}m", colors.get_color(&record.level()).to_fg_str()),
                level = clevel.color(record.level()),
                message = message,
            ));
        })
        .level(level)
        .chain(
            Output::call(|record| {
                if let Some(pb) = get_progress_bar() {
                    pb.println(record.args().to_string());
                } else {
                    writeln!(std::io::stderr(), "{}", record.args()).ok();
                }
            })

        );

    base_config = base_config.chain(file_config);
    if stderr { base_config = base_config.chain(stderr_config); }
    base_config.apply()?;

    Ok(())

}

fn main() -> Result<()> {
    let mut opts = Options::new();
    let args: Args = Args::from_args();

    let nthreads = match args.nthreads {
        0 => num_cpus::get(),
        n => n as usize
    };

    rayon::ThreadPoolBuilder::new().num_threads(nthreads).build_global().unwrap();

    if let Some(w) = args.cropwindow {
        opts.crop_window = [[w[0], w[1]], [w[2], w[3]]];
    }

     if let Some(f) = args.outfile {
         opts.image_file = f
    }

    opts.to_ply = args.toply;
    opts.cat = args.cat;

    let logdir = if let Some(dir) = args.logdir {
        dir
    } else {
        PathBuf::from(String::from("pbrt.log"))
    };

    setup_logging(args.verbose, logdir, args.logtostderr)?;
    let filename = args.input;
    // Initialize statistics counter
    init_stats();

    pbrt_parse(&filename, opts)
}
