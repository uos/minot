//! The line format every process in the stack writes and every parent reads.
//!
//! ```text
//! [LEVEL] [Source] message
//! ```
//!
//! The level is always there. The source is the process the record came from,
//! and appears only when that is not the process doing the printing: a run
//! nobody is capturing prints `[INFO] started`, and the same run under a
//! parent prints `[INFO] [Minot] started`. So a tag always means "not mine",
//! and a parent relaying the line has nothing to invent.

use std::io::IsTerminal;
use std::sync::OnceLock;

/// Environment variable a parent sets on a child to pick the child's output
/// format. Unset means plain text without a source tag.
pub const FORMAT_VAR: &str = "MT_LOG_FORMAT";
/// Environment variable naming the process, used for the source tag. A
/// process that knows its own name passes it to [`crate::init`] instead.
pub const SOURCE_VAR: &str = "MT_LOG_SOURCE";

/// How a process writes its records to the console.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum Format {
    /// `[LEVEL] message`, for a human reading a terminal.
    #[default]
    Text,
    /// `[LEVEL] [Source] message`, for a parent capturing the stream as text.
    Tagged,
    /// One JSON record per line, for a parent that wants the fields intact.
    #[cfg(feature = "json")]
    Json,
}

impl Format {
    /// Read the format a parent asked for, defaulting to plain text.
    ///
    /// Unrecognised values use text formatting. A log
    /// format is never worth aborting a run over.
    pub fn from_env() -> Self {
        match std::env::var(FORMAT_VAR).ok().as_deref() {
            Some("tagged") => Format::Tagged,
            #[cfg(feature = "json")]
            Some("json") => Format::Json,
            _ => Format::Text,
        }
    }

    /// The value a parent puts in [`FORMAT_VAR`] to ask for this format.
    pub fn as_str(self) -> &'static str {
        match self {
            Format::Text => "text",
            Format::Tagged => "tagged",
            #[cfg(feature = "json")]
            Format::Json => "json",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    /// The bare word, as it appears between the brackets.
    pub fn name(self) -> &'static str {
        match self {
            LogLevel::Trace => "TRACE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warn => "WARN",
            LogLevel::Error => "ERROR",
        }
    }

    /// Bracketed and padded to a fixed width, so a column of these lines up.
    pub fn tag(self) -> &'static str {
        match self {
            LogLevel::Trace => "[TRACE]",
            LogLevel::Debug => "[DEBUG]",
            LogLevel::Info => "[INFO] ",
            LogLevel::Warn => "[WARN] ",
            LogLevel::Error => "[ERROR]",
        }
    }

    /// Three-column stand-in for the tag, for a pane too narrow to spend
    /// seven columns saying what the colour already says.
    pub fn badge(self) -> &'static str {
        match self {
            LogLevel::Trace => "[T]",
            LogLevel::Debug => "[D]",
            LogLevel::Info => "[I]",
            LogLevel::Warn => "[W]",
            LogLevel::Error => "[E]",
        }
    }

    /// SGR parameter for the console. The pane picks its own colours from
    /// whatever palette its terminal library offers.
    pub fn ansi(self) -> &'static str {
        match self {
            LogLevel::Trace | LogLevel::Debug => "90",
            LogLevel::Info => "36",
            LogLevel::Warn => "33",
            LogLevel::Error => "31",
        }
    }

    /// Recognise a level word, however it was cased.
    pub fn from_name(word: &str) -> Option<Self> {
        [
            LogLevel::Trace,
            LogLevel::Debug,
            LogLevel::Info,
            LogLevel::Warn,
            LogLevel::Error,
        ]
        .into_iter()
        .find(|level| word.eq_ignore_ascii_case(level.name()))
    }
}

impl From<LogLevel> for log::Level {
    fn from(level: LogLevel) -> Self {
        match level {
            LogLevel::Trace => log::Level::Trace,
            LogLevel::Debug => log::Level::Debug,
            LogLevel::Info => log::Level::Info,
            LogLevel::Warn => log::Level::Warn,
            LogLevel::Error => log::Level::Error,
        }
    }
}

impl From<log::Level> for LogLevel {
    fn from(level: log::Level) -> Self {
        match level {
            log::Level::Trace => LogLevel::Trace,
            log::Level::Debug => LogLevel::Debug,
            log::Level::Info => LogLevel::Info,
            log::Level::Warn => LogLevel::Warn,
            log::Level::Error => LogLevel::Error,
        }
    }
}

/// One record, however it arrived: emitted by this process, parsed out of a
/// child's text, or decoded from a child's JSON.
#[derive(Debug, Clone, PartialEq)]
pub struct Record {
    pub level: LogLevel,
    /// The process the record came from, absent for our own records.
    pub source: Option<String>,
    /// The module or crate inside that process, when it said.
    pub target: Option<String>,
    pub message: String,
}

impl Record {
    pub fn new(level: LogLevel, message: impl Into<String>) -> Self {
        Self {
            level,
            source: None,
            target: None,
            message: message.into(),
        }
    }

    pub fn with_source(mut self, source: impl Into<String>) -> Self {
        self.source = Some(source.into());
        self
    }

    pub fn with_target(mut self, target: impl Into<String>) -> Self {
        self.target = Some(target.into());
        self
    }

    /// The console line for this record: `[LEVEL] [Source] target: message`,
    /// with the level coloured when `colour` is set. Source and target appear
    /// only when the record has them.
    pub fn console_line(&self, colour: bool) -> String {
        let mut line = String::with_capacity(self.message.len() + 24);
        if colour {
            line.push('[');
            line.push_str("\u{1b}[");
            line.push_str(self.level.ansi());
            line.push('m');
            line.push_str(self.level.name());
            line.push_str("\u{1b}[0m]");
        } else {
            line.push('[');
            line.push_str(self.level.name());
            line.push(']');
        }
        line.push(' ');
        line.push_str(&self.pane_message());
        line
    }

    /// One JSON object on one line. Field names are short because every
    /// record pays for them.
    #[cfg(feature = "json")]
    pub fn json_line(&self) -> String {
        let wire = wire::Wire {
            lvl: self.level.name(),
            src: self.source.as_deref(),
            tgt: self.target.as_deref(),
            msg: &self.message,
        };
        serde_json::to_string(&wire).unwrap_or_else(|_| self.console_line(false))
    }

    /// Name the process this record came from, unless it already names one.
    /// A parent relaying a child's output calls this: Pelorus relaying Minot
    /// hands us a record that already says `Minot`, and the innermost process
    /// is the one the reader needs.
    pub fn or_source(mut self, source: &str) -> Self {
        if self.source.is_none() {
            self.source = Some(source.to_owned());
        }
        self
    }

    /// Everything but the level, which a pane prints in its own column.
    pub fn pane_message(&self) -> String {
        let mut text = String::with_capacity(self.message.len() + 16);
        if let Some(source) = &self.source {
            text.push('[');
            text.push_str(source);
            text.push_str("] ");
        }
        if let Some(target) = &self.target {
            text.push_str(target);
            text.push_str(": ");
        }
        text.push_str(&self.message);
        text
    }

    /// The line this record should be written as under `format`.
    pub fn line(&self, format: Format, colour: bool) -> String {
        match format {
            Format::Text => Record {
                level: self.level,
                source: None,
                target: self.target.clone(),
                message: self.message.clone(),
            }
            .console_line(colour),
            Format::Tagged => self.console_line(colour),
            #[cfg(feature = "json")]
            Format::Json => self.json_line(),
        }
    }
}

#[cfg(feature = "json")]
mod wire {
    use super::{LogLevel, Record};
    use serde::{Deserialize, Serialize};

    #[derive(Serialize)]
    pub struct Wire<'a> {
        pub lvl: &'static str,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub src: Option<&'a str>,
        #[serde(skip_serializing_if = "Option::is_none")]
        pub tgt: Option<&'a str>,
        pub msg: &'a str,
    }

    #[derive(Deserialize)]
    struct Owned {
        lvl: String,
        #[serde(default)]
        src: Option<String>,
        #[serde(default)]
        tgt: Option<String>,
        msg: String,
    }

    /// Decode a record a child wrote as JSON. A line that is not one of ours
    /// comes back as `None` so the caller can fall back to reading it as text.
    pub fn decode(line: &str) -> Option<Record> {
        let owned: Owned = serde_json::from_str(line).ok()?;
        Some(Record {
            level: LogLevel::from_name(&owned.lvl)?,
            source: owned.src,
            target: owned.tgt,
            message: owned.msg,
        })
    }
}

/// Read one line of a child's output back into a record.
///
/// JSON is tried first when the child speaks it, and text otherwise: a
/// leading `[LEVEL]` gives the level, a following `[Source]` gives the
/// process, and a `target:` token gives the module. A line with no tag at all
/// is guessed at by looking for a level word, which is worth doing (an error
/// from a child is exactly what a reader scanning for red is looking for),
/// but only ever as the last resort.
pub fn parse(line: &str) -> Record {
    let line = sanitize(line.to_owned());

    #[cfg(feature = "json")]
    if line.starts_with('{') {
        if let Some(mut record) = wire::decode(&line) {
            normalize(&mut record);
            return record;
        }
    }

    let mut rest = line.as_str();
    let level = match split_tag(rest).and_then(|(tag, tail)| {
        LogLevel::from_name(tag).inspect(|_| {
            rest = tail.trim_start();
        })
    }) {
        Some(level) => level,
        None => guess_level(rest),
    };
    let mut record = Record::new(level, rest);
    normalize(&mut record);
    record
}

/// Pull the source and target out of a record's message, wherever the record
/// came from.
///
/// A relayed line arrives with its origin written into the text. Pelorus
/// re-emits Minot's records as `[Minot] app: read bag`, tagged as its own,
/// and the innermost process is the one the reader needs, so the last tag on
/// the front wins. A target whose first segment repeats that source says
/// nothing the tag did not: `[Minot] minot::app:` is `[Minot] app:`.
fn normalize(record: &mut Record) {
    while let Some((tag, tail)) = split_tag(&record.message) {
        let (tag, tail) = (tag.to_owned(), tail.trim_start().to_owned());
        record.source = Some(tag);
        record.message = tail;
    }
    if record.target.is_none() {
        if let Some((target, tail)) = split_target(&record.message) {
            let (target, tail) = (target.to_owned(), tail.to_owned());
            record.target = Some(target);
            record.message = tail;
        }
    }
    if let (Some(source), Some(target)) = (&record.source, &record.target) {
        record.target = match strip_segment(target, source) {
            "" => None,
            stripped => Some(stripped.to_owned()),
        };
    }
}

/// Split a leading `[word]` off a line, returning the word and the rest. The
/// word must be one token: `[fe80::1] is unreachable` is a message, not a tag.
fn split_tag(line: &str) -> Option<(&str, &str)> {
    let rest = line.strip_prefix('[')?;
    let end = rest.find(']')?;
    let (tag, tail) = (&rest[..end], &rest[end + 1..]);
    // A tag is one token, and something follows it: `[fe80::1]:49507` is an
    // address a message opens with, not a name for the message.
    if tag.is_empty() || tag.contains(' ') || !(tail.is_empty() || tail.starts_with(' ')) {
        return None;
    }
    Some((tag, tail))
}

/// Split a leading `target: ` off a line. A target is one unspaced token
/// ending in a colon and made of the characters a module path is made of, so
/// an ordinary sentence with a colon in it stays a message.
fn split_target(line: &str) -> Option<(&str, &str)> {
    let (head, tail) = line.split_once(": ")?;
    if head.is_empty()
        || head.contains(' ')
        || LogLevel::from_name(head).is_some()
        || !head
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | ':' | '-' | '.'))
    {
        return None;
    }
    Some((head, tail))
}

/// Drop a leading path segment equal to `segment`, however it was cased.
fn strip_segment<'a>(path: &'a str, segment: &str) -> &'a str {
    let rest = match path.get(..segment.len()) {
        Some(head) if head.eq_ignore_ascii_case(segment) => &path[segment.len()..],
        _ => return path,
    };
    match rest.strip_prefix("::") {
        Some(tail) => tail,
        None if rest.is_empty() => rest,
        None => path,
    }
}

/// Last-resort level for a line that carries no tag of its own.
fn guess_level(line: &str) -> LogLevel {
    let upper = line.to_ascii_uppercase();
    if upper.contains("ERROR") {
        LogLevel::Error
    } else if upper.contains("WARN") {
        LogLevel::Warn
    } else if upper.contains("DEBUG") {
        LogLevel::Debug
    } else if upper.contains("TRACE") {
        LogLevel::Trace
    } else {
        LogLevel::Info
    }
}

/// Reduce a captured line to text a cell grid can hold.
///
/// Lines relayed from a child arrive already formatted for a terminal, colour
/// codes and all. Writing those bytes into cells hands them to the real
/// terminal, which moves the cursor and scrambles the pane around them, so the
/// escapes are dropped once, on the way in. Newlines and tabs become spaces
/// as a space so adjacent words stay apart.
pub fn sanitize(message: String) -> String {
    if !message
        .chars()
        .any(|character| character.is_control() || character == '\u{7f}')
    {
        return message;
    }
    let mut clean = String::with_capacity(message.len());
    let mut characters = message.chars();
    while let Some(character) = characters.next() {
        match character {
            // CSI and OSC run until their own terminator. Anything else after
            // the escape is a two-character sequence.
            '\u{1b}' => match characters.next() {
                Some('[') => {
                    for following in characters.by_ref() {
                        if matches!(following, '\u{40}'..='\u{7e}') {
                            break;
                        }
                    }
                }
                Some(']') => {
                    while let Some(following) = characters.next() {
                        if following == '\u{7}' {
                            break;
                        }
                        if following == '\u{1b}' {
                            characters.next();
                            break;
                        }
                    }
                }
                _ => {}
            },
            '\t' | '\n' | '\r' => clean.push(' '),
            character if character.is_control() || character == '\u{7f}' => {}
            character => clean.push(character),
        }
    }
    clean
}

/// Whether stderr should be coloured: only when a terminal is there to read
/// it. Piped output stays plain, so a parent capturing it gets text rather
/// than escapes it has to strip back off.
pub fn colour_stderr() -> bool {
    static COLOUR: OnceLock<bool> = OnceLock::new();
    *COLOUR.get_or_init(|| std::io::stderr().is_terminal())
}

/// The environment a parent should give a child so its output can be read
/// back without guessing: the wire format, and the name to tag it with.
///
/// ```no_run
/// # use std::process::Command;
/// let mut command = Command::new("minot");
/// for (key, value) in mt_log::child_env("Minot") {
///     command.env(key, value);
/// }
/// ```
pub fn child_env(source: &str) -> [(&'static str, String); 2] {
    [
        (FORMAT_VAR, preferred_child_format().to_owned()),
        (SOURCE_VAR, source.to_owned()),
    ]
}

/// JSON when this build can read it, tagged text otherwise. Both are parsed
/// by [`parse`], so a child that only speaks the older text format
/// still comes through.
fn preferred_child_format() -> &'static str {
    #[cfg(feature = "json")]
    {
        Format::Json.as_str()
    }
    #[cfg(not(feature = "json"))]
    {
        Format::Tagged.as_str()
    }
}
