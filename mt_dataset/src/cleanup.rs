use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};

#[cfg(unix)]
fn restore_terminal_echo_best_effort() {
    use std::fs::File;
    use std::process::{Command, Stdio};

    if let Ok(tty) = File::open("/dev/tty") {
        let _ = Command::new("stty")
            .arg("echo")
            .stdin(Stdio::from(tty))
            .status();
    }
}

#[cfg(not(unix))]
fn restore_terminal_echo_best_effort() {}

fn paths() -> &'static Mutex<Vec<PathBuf>> {
    static PATHS: OnceLock<Mutex<Vec<PathBuf>>> = OnceLock::new();
    PATHS.get_or_init(|| Mutex::new(Vec::new()))
}

/// Install a Ctrl-C handler that removes any registered paths and exits.
/// Should be called once at startup.
pub fn init() {
    let _ = ctrlc::set_handler(|| {
        restore_terminal_echo_best_effort();
        if let Ok(guard) = paths().lock() {
            for path in guard.iter() {
                if path.is_dir() {
                    let _ = std::fs::remove_dir_all(path);
                } else if path.is_file() {
                    let _ = std::fs::remove_file(path);
                }
            }
        }
        std::process::exit(130);
    });
}

/// Register a path for deletion if the process is interrupted.
pub fn register(path: PathBuf) {
    if let Ok(mut guard) = paths().lock() {
        guard.push(path);
    }
}

/// Clear all registered paths (call after a successful operation).
pub fn commit() {
    if let Ok(mut guard) = paths().lock() {
        guard.clear();
    }
}
