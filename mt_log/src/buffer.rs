//! The records a status view keeps to draw.

use std::io::Write;
use std::sync::Mutex;
use std::time::Instant;

use crate::format::{LogLevel, Record, colour_stderr, sanitize};

/// Entries retained for the pane. Older ones are dropped.
const MAX_LOG_ENTRIES: usize = 10_000;

#[derive(Debug, Clone)]
pub struct LogEntry {
    /// Seconds since the run started. Wall clock is not useful while watching
    /// a live run. Time since start lines up with everything else on screen.
    pub timestamp: f64,
    pub level: LogLevel,
    pub target: String,
    pub message: String,
}

pub struct LogBuffer {
    entries: Mutex<Vec<LogEntry>>,
    started: Instant,
    /// Target reported for records this process emitted itself. Everything
    /// else counts as foreign and can be filtered out in the pane.
    own_target: String,
}

impl LogBuffer {
    pub fn new(own_target: impl Into<String>) -> Self {
        Self {
            entries: Mutex::new(Vec::new()),
            started: Instant::now(),
            own_target: own_target.into(),
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Vec<LogEntry>> {
        match self.entries.lock() {
            Ok(entries) => entries,
            Err(poisoned) => poisoned.into_inner(),
        }
    }

    pub fn push(&self, level: LogLevel, target: impl Into<String>, message: impl Into<String>) {
        let entry = LogEntry {
            timestamp: self.started.elapsed().as_secs_f64(),
            level,
            target: target.into(),
            message: sanitize(message.into()),
        };
        let mut entries = self.lock();
        entries.push(entry);
        if entries.len() > MAX_LOG_ENTRIES {
            let excess = entries.len() - MAX_LOG_ENTRIES;
            entries.drain(..excess);
        }
    }

    /// Add a record relayed from another process, tagged with its source.
    pub fn push_record(&self, target: impl Into<String>, record: &Record) {
        self.push(record.level, target, record.pane_message());
    }

    pub fn snapshot(&self) -> Vec<LogEntry> {
        self.lock().clone()
    }

    pub fn len(&self) -> usize {
        self.lock().len()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn owns(&self, target: &str) -> bool {
        target == self.own_target || target.starts_with(&self.own_target)
    }

    pub fn is_own(&self, entry: &LogEntry) -> bool {
        self.owns(&entry.target)
    }

    /// One entry as a record.
    ///
    /// The target is kept only when it is not ours: a foreign crate's name is
    /// worth the width, and our own module paths are not — the process is
    /// already identified by the line it is printed on.
    pub fn record(&self, level: LogLevel, target: &str, message: &str) -> Record {
        let mut record = Record::new(level, message);
        if !self.owns(target) {
            record.target = Some(target.to_owned());
        }
        record
    }

    /// Format one record the way the pane does, for the console.
    pub fn console_line(&self, level: LogLevel, target: &str, message: &str) -> String {
        self.record(level, target, message)
            .console_line(colour_stderr())
    }

    /// Write every retained entry to stderr and forget it.
    ///
    /// Used when the view never got to display them, so a run whose terminal
    /// setup failed does not silently swallow its own log.
    pub fn drain_to_stderr(&self) {
        let lines: Vec<String> = {
            let mut entries = self.lock();
            entries
                .drain(..)
                .map(|entry| self.console_line(entry.level, &entry.target, &entry.message))
                .collect()
        };
        let mut stderr = std::io::stderr();
        for line in lines {
            let _ = writeln!(stderr, "{line}");
        }
    }
}
