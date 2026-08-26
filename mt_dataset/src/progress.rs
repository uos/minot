use std::io::Write;

/// One progress update emitted by marina operations.
#[derive(Debug, Clone)]
pub struct ProgressEvent {
    /// Stable phase identifier (for example: `push`, `pack`, `pull`, `unpack`).
    pub phase: &'static str,
    /// Human-readable progress message for this phase.
    pub message: String,
}

/// Consumer for progress events from marina.
pub trait ProgressSink {
    /// Receive a single progress event.
    fn emit(&mut self, event: ProgressEvent);
}

/// Helper that routes progress events to an optional sink.
pub struct ProgressReporter<'a> {
    sink: Option<&'a mut dyn ProgressSink>,
}

impl<'a> ProgressReporter<'a> {
    /// Creates a reporter that drops all progress messages.
    pub fn silent() -> Self {
        Self { sink: None }
    }

    /// Creates a reporter that forwards events to `sink`.
    pub fn new(sink: &'a mut dyn ProgressSink) -> Self {
        Self { sink: Some(sink) }
    }

    /// Emits a progress event if a sink is configured.
    pub fn emit(&mut self, phase: &'static str, message: impl Into<String>) {
        if let Some(sink) = self.sink.as_deref_mut() {
            sink.emit(ProgressEvent {
                phase,
                message: message.into(),
            });
        }
    }
}

/// Progress sink that writes messages to a `std::io::Write` target.
pub struct WriterProgress<'a> {
    writer: &'a mut dyn Write,
}

impl<'a> WriterProgress<'a> {
    /// Creates a writer-backed progress sink.
    pub fn new(writer: &'a mut dyn Write) -> Self {
        Self { writer }
    }
}

impl ProgressSink for WriterProgress<'_> {
    fn emit(&mut self, event: ProgressEvent) {
        let tag = color_phase(event.phase);
        let _ = writeln!(self.writer, "{} {}", tag, event.message);
        let _ = self.writer.flush();
    }
}

fn color_phase(phase: &str) -> String {
    let (color, label) = match phase {
        "push" => ("1;34", "PUSH"),
        "pull" => ("1;36", "PULL"),
        "pack" => ("1;33", "PACK"),
        "unpack" => ("1;32", "UNPACK"),
        _ => ("1;37", phase),
    };
    format!("\x1b[{}m[{}]\x1b[0m", color, label)
}
