//! Console logging for a process with no status view of its own.

use std::io::Write;

use crate::format::{Format, Record, SOURCE_VAR, colour_stderr};

/// Install a console logger in the house format.
///
/// `source` is what this process calls itself; it is printed only when a
/// parent asked for it through `MT_LOG_FORMAT`, so a direct run stays terse
/// and a captured one is unambiguous. `default_spec` is used when `RUST_LOG`
/// says nothing.
///
/// Fails silently if a logger is already installed: an unexpected second call
/// should not take a process down over its logging.
pub fn init(source: &str, default_spec: &str) {
    init_filtered(source, "RUST_LOG", default_spec, &[]);
}

/// As [`init`], with the filter read from `var` and `quiet` applied on top of
/// whatever it says.
///
/// The modules in `quiet` are capped *after* the environment is parsed, so a
/// `RUST_LOG=debug` run still does not drown in transport chatter.
pub fn init_filtered(
    source: &str,
    var: &str,
    default_spec: &str,
    quiet: &[(&str, log::LevelFilter)],
) {
    let format = Format::from_env();
    let source = std::env::var(SOURCE_VAR).unwrap_or_else(|_| source.to_owned());
    let colour = colour_stderr();

    let mut builder = env_logger::Builder::from_env(
        env_logger::Env::new().filter_or(var, default_spec.to_owned()),
    );
    for (module, level) in quiet {
        builder.filter_module(module, *level);
    }
    let _ = builder
        .format(move |buf, record| {
            let line = Record::new(record.level().into(), record.args().to_string())
                .with_source(source.clone())
                .with_target(record.target())
                .line(format, colour);
            writeln!(buf, "{line}")
        })
        .target(env_logger::Target::Stderr)
        .try_init();
}

/// The transport modules every process in the stack shares, and the levels
/// past which none of them is worth reading. Zenoh narrates its own routing
/// at info, which buries everything a run is actually about.
pub const QUIET_ZENOH: &[(&str, log::LevelFilter)] = &[
    ("zenoh", log::LevelFilter::Warn),
    ("zenoh::api::admin", log::LevelFilter::Off),
    ("zenoh::api::session", log::LevelFilter::Off),
    ("zenoh::net::routing::hat::peer", log::LevelFilter::Error),
    ("zenoh_transport", log::LevelFilter::Warn),
    ("zenoh_link", log::LevelFilter::Warn),
    ("zenoh_protocol", log::LevelFilter::Warn),
];
