use std::path::{Path, PathBuf};
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

fn remove_path(path: &Path) {
    if path.is_dir() {
        let _ = std::fs::remove_dir_all(path);
    } else if path.is_file() {
        let _ = std::fs::remove_file(path);
    }
}

fn unregister(path: &Path) {
    if let Ok(mut guard) = paths().lock()
        && let Some(index) = guard.iter().position(|registered| registered == path)
    {
        guard.swap_remove(index);
    }
}

/// Install a Ctrl-C handler that removes any registered paths and exits.
/// Should be called once at startup.
pub fn init() {
    let _ = ctrlc::set_handler(|| {
        restore_terminal_echo_best_effort();
        remove_registered();
        std::process::exit(130);
    });
}

/// Removes every path whose operation has not finished yet.
///
/// Call this before exiting while operations may still run on other threads,
/// because exiting skips their destructors. The Ctrl-C handler installed by
/// [`init`] calls it as well.
pub fn remove_registered() {
    let registered = match paths().lock() {
        Ok(mut guard) => std::mem::take(&mut *guard),
        Err(_) => return,
    };
    for path in &registered {
        remove_path(path);
    }
}

/// Deletes `path` unless the operation that creates it finishes.
///
/// The path is removed when the returned guard is dropped without
/// [`CleanupGuard::keep`], which covers an error and a cancelled task, and by
/// [`remove_registered`], which covers Ctrl-C and exiting the application.
/// Each guard only ever affects its own path, so concurrent operations do not
/// disturb each other.
#[must_use = "the path is deleted as soon as the guard is dropped"]
pub fn register(path: PathBuf) -> CleanupGuard {
    if let Ok(mut guard) = paths().lock() {
        guard.push(path.clone());
    }
    CleanupGuard { path, armed: true }
}

/// Removes its path on drop unless the operation that creates it finished.
pub struct CleanupGuard {
    path: PathBuf,
    armed: bool,
}

impl CleanupGuard {
    /// Marks the operation as finished, so the path stays.
    pub fn keep(mut self) {
        self.armed = false;
        unregister(&self.path);
    }
}

impl Drop for CleanupGuard {
    fn drop(&mut self) {
        if self.armed {
            unregister(&self.path);
            remove_path(&self.path);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_dropped_guard_removes_only_its_own_path() -> anyhow::Result<()> {
        let dir = tempfile::tempdir()?;
        let finished = dir.path().join("finished");
        let failed = dir.path().join("failed");
        std::fs::create_dir(&finished)?;
        std::fs::create_dir(&failed)?;

        let finished_guard = register(finished.clone());
        let failed_guard = register(failed.clone());
        drop(failed_guard);
        finished_guard.keep();

        assert!(finished.exists());
        assert!(!failed.exists());
        let registered = paths().lock().unwrap();
        assert!(!registered.contains(&finished));
        assert!(!registered.contains(&failed));
        Ok(())
    }
}
