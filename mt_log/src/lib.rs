//! One log line format for every process in the Minot stack.
//!
//! Minot, Pelorus and Polarstern run as a chain of parent and child
//! processes, and each one used to flatten its records to text in its own
//! shape and guess the next one's back out of it. This crate holds the one
//! format they all write and the one reader they all use:
//!
//! ```text
//! [LEVEL] [Source] message
//! ```
//!
//! A process logging for itself prints `[INFO] started`. A parent that wants
//! its children's output back without guessing gives them [`child_env`], and
//! they answer in a format [`parse`] reads exactly — level, source and target
//! intact, rather than recovered by looking for the word "error".
//!
//! # Features
//!
//! - `json` (default): structured records on the wire.
//! - `tui`: [`LogBuffer`], the [`Logger`] that feeds it, and the [`pane`].
//! - `env`: [`init`], for a process that logs straight to the console.

mod format;

pub use format::{
    FORMAT_VAR, Format, LogLevel, Record, SOURCE_VAR, child_env, colour_stderr, parse, sanitize,
};

#[cfg(feature = "tui")]
mod buffer;
#[cfg(feature = "tui")]
mod logger;
#[cfg(feature = "tui")]
pub mod pane;

#[cfg(feature = "tui")]
pub use buffer::{LogBuffer, LogEntry};
#[cfg(feature = "tui")]
pub use logger::{Logger, Suppressor};
#[cfg(feature = "tui")]
pub use pane::LogView;

#[cfg(feature = "env")]
mod init;
#[cfg(feature = "env")]
pub use init::{QUIET_ZENOH, init, init_filtered, quieted};

#[cfg(test)]
mod tests;
