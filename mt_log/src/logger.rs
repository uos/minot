//! A `log` implementation for a process whose status view owns the terminal.
//!
//! Records go to the buffer the pane draws while the view holds the screen,
//! and to stderr in the house format while it does not, so a run that fails
//! before the view opens still says why.
//!
//! Filtering goes through `env_filter`, the same engine `env_logger` uses, so
//! `RUST_LOG` keeps its per-module directives. Capturing formatted text
//! instead would cost the level and target the pane colours and filters by.

use std::io::Write;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, RwLock};

use crate::buffer::LogBuffer;
use crate::format::{Format, LogLevel, colour_stderr};

/// Decides whether one record is dropped before it reaches the buffer, for
/// the chatter a dependency emits that no reader ever wants.
pub type Suppressor = fn(LogLevel, &str, &str) -> bool;

pub struct Logger {
    /// Rebuilt when the spec changes: a process that reads its filter from a
    /// config file installs the logger before it has one.
    filter: RwLock<env_filter::Filter>,
    buffer: Arc<LogBuffer>,
    owns_screen: Arc<AtomicBool>,
    suppress: Option<Suppressor>,
    /// What this process calls itself, printed only when a parent asked for
    /// it through the format.
    source: String,
    format: Format,
}

impl Logger {
    /// Install as the global logger, returning a handle for later changes to
    /// the filter.
    pub fn install(
        source: &str,
        spec: &str,
        buffer: Arc<LogBuffer>,
        owns_screen: Arc<AtomicBool>,
    ) -> Result<&'static Logger, log::SetLoggerError> {
        Self::install_with(source, spec, buffer, owns_screen, None)
    }

    pub fn install_with(
        source: &str,
        spec: &str,
        buffer: Arc<LogBuffer>,
        owns_screen: Arc<AtomicBool>,
        suppress: Option<Suppressor>,
    ) -> Result<&'static Logger, log::SetLoggerError> {
        let filter = env_filter::Builder::new().parse(spec).build();
        let max_level = filter.filter();
        // The global logger outlives everything anyway, and a leaked
        // reference is what lets the caller keep talking to it.
        let logger: &'static Logger = Box::leak(Box::new(Self {
            filter: RwLock::new(filter),
            buffer,
            owns_screen,
            suppress,
            source: source.to_owned(),
            format: Format::from_env(),
        }));
        log::set_logger(logger)?;
        log::set_max_level(max_level);
        Ok(logger)
    }

    /// Replace the filter, for a process that learns its log spec from a
    /// config file after the logger is already up.
    pub fn set_spec(&self, spec: &str) {
        let filter = env_filter::Builder::new().parse(spec).build();
        let max_level = filter.filter();
        if let Ok(mut current) = self.filter.write() {
            *current = filter;
        }
        log::set_max_level(max_level);
    }

    fn admits(&self, record: &log::Record) -> bool {
        match self.filter.read() {
            Ok(filter) => filter.matches(record),
            Err(_) => true,
        }
    }
}

impl log::Log for Logger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        match self.filter.read() {
            Ok(filter) => filter.enabled(metadata),
            Err(_) => true,
        }
    }

    fn log(&self, record: &log::Record) {
        if !self.admits(record) {
            return;
        }
        let level = LogLevel::from(record.level());
        let target = record.target();
        let message = record.args().to_string();
        if let Some(suppress) = self.suppress {
            if suppress(level, target, &message) {
                return;
            }
        }

        if self.owns_screen.load(Ordering::Relaxed) {
            self.buffer.push(level, target, message);
        } else {
            // Captured child output includes the source label.
            let line = self
                .buffer
                .record(level, target, &message)
                .with_source(self.source.clone())
                .line(self.format, colour_stderr());
            let _ = writeln!(std::io::stderr(), "{line}");
        }
    }

    fn flush(&self) {}
}
