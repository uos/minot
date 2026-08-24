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
    // Held at error, not off. These narrate their normal operation and are
    // worth nothing at warn, but a module that has genuinely failed still has
    // something to say, and silencing it wholesale is how a real fault turns
    // into a process that merely stops working with no explanation.
    ("zenoh::api::admin", log::LevelFilter::Error),
    ("zenoh::api::session", log::LevelFilter::Error),
    ("zenoh::net::routing::hat::peer", log::LevelFilter::Error),
    // A bounded query is bounded by being allowed to expire, and Zenoh reports
    // every expiry from both ends at warn: the router noting the deadline, the
    // session then receiving a reply for a query it has already dropped. One
    // best-effort peer walking away makes every publisher in the process
    // narrate the same non-event. Real errors still come through.
    (
        "zenoh::net::routing::dispatcher::queries",
        log::LevelFilter::Error,
    ),
    ("zenoh_transport", log::LevelFilter::Warn),
    ("zenoh_link", log::LevelFilter::Warn),
    ("zenoh_protocol", log::LevelFilter::Warn),
];

/// Add [`QUIET_ZENOH`] to a filter spec, so the transport stays out of it.
///
/// Takes either a bare level or a full spec, because a process is handed one
/// or the other depending on whether its level came from a config file or from
/// `RUST_LOG`. Anything the spec already says is left alone: these are appended
/// as more specific directives, and the longest match wins.
///
/// Each module is *capped*, never raised. Appending `zenoh=warn` to a spec that
/// asked for `error` would turn chatter back on for the level that asked for
/// the least of it.
pub fn quieted(spec: &str) -> String {
    // The global level is the directive with no module name. A spec that sets
    // none leaves the libraries at their own default, which is info.
    let base = spec
        .split(',')
        .find(|directive| !directive.contains('='))
        .and_then(|level| level.trim().parse::<log::LevelFilter>().ok())
        .unwrap_or(log::LevelFilter::Info);
    QUIET_ZENOH
        .iter()
        .fold(spec.to_owned(), |spec, (module, cap)| {
            format!("{spec},{module}={}", base.min(*cap))
        })
}
