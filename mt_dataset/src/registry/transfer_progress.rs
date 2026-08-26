use std::io::IsTerminal;
use std::time::Duration;

use indicatif::{ProgressBar, ProgressDrawTarget, ProgressStyle};

/// Progress indicator shared by registry uploads and downloads.
///
/// Known-size transfers show completion, throughput, and ETA. Unknown-size
/// transfers use a spinner while still reporting bytes and throughput.
pub(super) fn transfer_bar(total: u64, message: &str) -> ProgressBar {
    let pb = if total > 0 {
        ProgressBar::new(total)
    } else {
        ProgressBar::new_spinner()
    };
    if !std::io::stdout().is_terminal() {
        pb.set_draw_target(ProgressDrawTarget::hidden());
    }
    let style = if total > 0 {
        ProgressStyle::with_template(
            "{msg} [{bar:40.cyan/blue}] {bytes}/{total_bytes} {bytes_per_sec} ({eta})",
        )
        .unwrap_or_else(|_| ProgressStyle::default_bar())
    } else {
        ProgressStyle::with_template("{spinner} {msg} {bytes} {bytes_per_sec}")
            .unwrap_or_else(|_| ProgressStyle::default_spinner())
            .tick_chars("|/-\\ ")
    };
    pb.set_style(style);
    pb.set_message(message.to_string());
    pb.enable_steady_tick(Duration::from_millis(100));
    pb
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_size_transfer_tracks_length_and_position() {
        let pb = transfer_bar(128, "download");
        pb.inc(32);
        assert_eq!(pb.length(), Some(128));
        assert_eq!(pb.position(), 32);
        pb.finish_and_clear();
    }

    #[test]
    fn unknown_size_transfer_still_tracks_downloaded_bytes() {
        let pb = transfer_bar(0, "download");
        pb.inc(32);
        assert_eq!(pb.length(), None);
        assert_eq!(pb.position(), 32);
        pb.finish_and_clear();
    }
}
