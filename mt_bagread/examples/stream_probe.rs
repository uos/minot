//! Spike: characterise the byte-range access pattern of a full bag read.
//!
//! MCAP parsing in `mt_bagread` is sans-io — it asks for byte ranges and is
//! handed the bytes. That makes reading a bag on another machine possible, but
//! only if the access pattern is friendly: a few large sequential requests are
//! fine over a 200 ms link, a million small random ones are not.
//!
//! This measures the pattern via a [`BagIo`] that records every read and seek,
//! then models what that pattern would cost at a range of round-trip times.
//!
//! ```text
//! cargo run --release -p mt_bagread --example stream_probe -- <bag.mcap> [max_messages]
//! ```

use std::io::{Read, Seek, SeekFrom};
use std::sync::{Arc, Mutex};

use mt_bagread::Bagfile;

/// One byte-range the reader asked for.
#[derive(Debug, Clone, Copy)]
struct Request {
    offset: u64,
    len: u64,
    /// Whether this request starts exactly where the previous one ended.
    sequential: bool,
}

#[derive(Default)]
struct Trace {
    requests: Vec<Request>,
    seeks: u64,
    cursor: u64,
}

/// A `BagIo` that passes through to an inner source and records the pattern.
struct RecordingIo<T> {
    inner: T,
    trace: Arc<Mutex<Trace>>,
}

impl<T: Read + Seek + Send> Read for RecordingIo<T> {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        let read = self.inner.read(buf)?;
        let mut trace = self.trace.lock().unwrap();
        let offset = trace.cursor;
        let sequential = trace
            .requests
            .last()
            .is_some_and(|prev| prev.offset + prev.len == offset);
        trace.requests.push(Request {
            offset,
            len: read as u64,
            sequential,
        });
        trace.cursor += read as u64;
        Ok(read)
    }
}

impl<T: Read + Seek + Send> Seek for RecordingIo<T> {
    fn seek(&mut self, pos: SeekFrom) -> std::io::Result<u64> {
        let at = self.inner.seek(pos)?;
        let mut trace = self.trace.lock().unwrap();
        trace.seeks += 1;
        trace.cursor = at;
        Ok(at)
    }
}

fn human(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "KiB", "MiB", "GiB", "TiB"];
    let mut value = bytes as f64;
    let mut unit = 0;
    while value >= 1024.0 && unit < UNITS.len() - 1 {
        value /= 1024.0;
        unit += 1;
    }
    format!("{value:.1} {}", UNITS[unit])
}

fn main() -> anyhow::Result<()> {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .ok_or_else(|| anyhow::anyhow!("usage: stream_probe <bag.mcap> [max_messages]"))?;
    let max_messages: usize = args
        .next()
        .map(|v| v.parse())
        .transpose()?
        .unwrap_or(usize::MAX);

    let trace = Arc::new(Mutex::new(Trace::default()));
    let file = std::fs::File::open(&path)?;
    let io = RecordingIo {
        inner: file,
        trace: Arc::clone(&trace),
    };

    // MINOT_PROBE_BUF=<bytes> wraps the source in a BufReader to test whether
    // buffering coalesces the reader's many small requests into few large ones.
    let buffered: Option<usize> = std::env::var("MINOT_PROBE_BUF")
        .ok()
        .map(|v| v.parse())
        .transpose()?;

    let mut bag = Bagfile::default();
    let summary_start = std::time::Instant::now();
    match buffered {
        Some(capacity) => {
            bag.reset_with_io(Box::new(std::io::BufReader::with_capacity(capacity, io)), None)?
        }
        None => bag.reset_with_io(Box::new(io), None)?,
    }
    let summary_time = summary_start.elapsed();

    // The summary read is its own phase: it happens once, before any message,
    // and its cost is paid on every open. Snapshot it separately.
    let summary_requests = trace.lock().unwrap().requests.len();
    let summary_bytes: u64 = trace
        .lock()
        .unwrap()
        .requests
        .iter()
        .map(|request| request.len)
        .sum();
    let summary_seeks = trace.lock().unwrap().seeks;
    let summary_sequential = trace
        .lock()
        .unwrap()
        .requests
        .iter()
        .filter(|request| request.sequential)
        .count();

    let read_start = std::time::Instant::now();
    let mut messages = 0usize;
    while messages < max_messages {
        match bag.next_message_with_timestamp()? {
            Some(_) => messages += 1,
            None => break,
        }
    }
    let read_time = read_start.elapsed();

    let trace = trace.lock().unwrap();
    let message_requests = &trace.requests[summary_requests..];
    let total_bytes: u64 = message_requests.iter().map(|request| request.len).sum();
    let sequential = message_requests
        .iter()
        .filter(|request| request.sequential)
        .count();
    let mut sizes: Vec<u64> = message_requests.iter().map(|request| request.len).collect();
    sizes.sort_unstable();

    let percentile = |p: f64| -> u64 {
        if sizes.is_empty() {
            return 0;
        }
        sizes[((sizes.len() as f64 - 1.0) * p) as usize]
    };

    println!("bag:                 {path}");
    match buffered {
        Some(capacity) => println!("buffering:           BufReader {}", human(capacity as u64)),
        None => println!("buffering:           none (raw)"),
    }
    println!("messages read:       {messages}");
    println!();
    println!("--- summary phase (paid once per open) ---");
    println!("  requests:          {summary_requests}");
    println!("  seeks:             {summary_seeks}");
    println!(
        "  sequential:        {summary_sequential} ({:.1}%)",
        100.0 * summary_sequential as f64 / summary_requests.max(1) as f64
    );
    println!(
        "  mean request:      {}",
        human(summary_bytes / summary_requests.max(1) as u64)
    );
    println!("  bytes:             {}", human(summary_bytes));
    println!("  local wall time:   {summary_time:.2?}");
    println!();
    println!("--- message phase ---");
    println!("  chunk requests:    {}", message_requests.len());
    println!(
        "  sequential:        {sequential} ({:.1}%)",
        100.0 * sequential as f64 / message_requests.len().max(1) as f64
    );
    println!("  seeks:             {}", trace.seeks - summary_seeks);
    println!("  bytes:             {}", human(total_bytes));
    println!("  local wall time:   {read_time:.2?}");
    // Whether offsets only ever move forward decides how hard readahead is: if
    // they do, prefetching is just "fetch the next N index entries".
    let backward = message_requests
        .windows(2)
        .filter(|pair| pair[1].offset < pair[0].offset)
        .count();
    let contiguous = message_requests
        .windows(2)
        .filter(|pair| pair[1].offset == pair[0].offset + pair[0].len)
        .count();
    println!(
        "  monotonic:         {} ({} backward jumps)",
        if backward == 0 { "yes" } else { "no" },
        backward
    );
    println!(
        "  gapless forward:   {contiguous}/{} ({:.1}%)",
        message_requests.len().saturating_sub(1),
        100.0 * contiguous as f64 / message_requests.len().saturating_sub(1).max(1) as f64
    );
    println!(
        "  request size:      p50 {} / p95 {} / max {}",
        human(percentile(0.50)),
        human(percentile(0.95)),
        human(percentile(1.0))
    );
    println!();

    // Model: every request costs one round trip when issued serially. With a
    // readahead window of W, W requests are in flight at once, so the latency
    // cost is amortised across the window. Bandwidth is deliberately ignored —
    // this isolates the question the spike is asking, which is whether latency
    // alone sinks the approach.
    let total_requests = (summary_requests + message_requests.len()) as f64;
    println!("--- modelled latency cost (bandwidth ignored) ---");
    println!("  {:>8}  {:>12}  {:>12}  {:>12}", "RTT", "serial", "W=8", "W=32");
    for rtt_ms in [1.0_f64, 20.0, 100.0, 200.0] {
        let serial = total_requests * rtt_ms / 1000.0;
        println!(
            "  {:>6}ms  {:>10.1}s  {:>10.1}s  {:>10.1}s",
            rtt_ms,
            serial,
            serial / 8.0,
            serial / 32.0
        );
    }

    Ok(())
}
