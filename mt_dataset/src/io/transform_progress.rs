use std::io::IsTerminal;

use indicatif::{ProgressBar, ProgressStyle};

use crate::progress::ProgressReporter;

pub fn make_count_progress_bar(total: usize, label: impl Into<String>, unit: &str) -> ProgressBar {
    if !std::io::stdout().is_terminal() {
        return ProgressBar::hidden();
    }

    if total > 0 {
        let pb = ProgressBar::new(total as u64);
        pb.set_style(
            ProgressStyle::with_template(&format!(
                "{{msg}} [{{bar:40.cyan/blue}}] {{pos}}/{{len}} {unit} ({{eta}})"
            ))
            .unwrap_or_else(|_| ProgressStyle::default_bar()),
        );
        pb.set_message(label.into());
        pb
    } else {
        let pb = ProgressBar::new_spinner();
        pb.set_style(
            ProgressStyle::with_template("{spinner} {msg}")
                .unwrap_or_else(|_| ProgressStyle::default_spinner())
                .tick_chars("|/-\\ "),
        );
        pb.set_message(label.into());
        pb.enable_steady_tick(std::time::Duration::from_millis(100));
        pb
    }
}

pub fn make_byte_progress_bar(total_bytes: u64, label: impl Into<String>) -> ProgressBar {
    if !std::io::stdout().is_terminal() {
        return ProgressBar::hidden();
    }

    if total_bytes > 0 {
        let pb = ProgressBar::new(total_bytes);
        pb.set_style(
            ProgressStyle::with_template(
                "{msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({eta})",
            )
            .unwrap_or_else(|_| ProgressStyle::default_bar()),
        );
        pb.set_message(label.into());
        pb
    } else {
        let pb = ProgressBar::new_spinner();
        pb.set_style(
            ProgressStyle::with_template("{spinner} {msg}")
                .unwrap_or_else(|_| ProgressStyle::default_spinner())
                .tick_chars("|/-\\ "),
        );
        pb.set_message(label.into());
        pb.enable_steady_tick(std::time::Duration::from_millis(100));
        pb
    }
}

pub fn emit_count_progress(
    progress: &mut ProgressReporter<'_>,
    phase: &'static str,
    label: &str,
    current: usize,
    total: usize,
) {
    if total > 0 {
        let pct = (current as f64 / total as f64) * 100.0;
        progress.emit(
            phase,
            format!("{} {}/{} ({:.1}%)", label, current, total, pct),
        );
    } else {
        progress.emit(phase, format!("{} {}", label, current));
    }
}
