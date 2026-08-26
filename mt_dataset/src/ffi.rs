use std::cell::RefCell;
use std::ffi::{CStr, CString, c_char, c_void};

use crate::{Marina, ProgressEvent, ProgressReporter, ProgressSink, ResolveResult, WriterProgress};

pub const MARINA_RESOLVE_ERROR: i32 = -1;
pub const MARINA_RESOLVE_LOCAL: i32 = 0;
pub const MARINA_RESOLVE_CACHED: i32 = 1;
pub const MARINA_RESOLVE_REMOTE_AVAILABLE: i32 = 2;
pub const MARINA_RESOLVE_AMBIGUOUS: i32 = 3;
pub const MARINA_PROGRESS_MODE_SILENT: i32 = 0;
pub const MARINA_PROGRESS_MODE_STDOUT: i32 = 1;

pub type MarinaProgressCallback =
    Option<extern "C" fn(phase: *const c_char, message: *const c_char, user_data: *mut c_void)>;

#[repr(C)]
pub struct MarinaResolveDetailed {
    pub kind: i32,
    pub path: *mut c_char,
    pub bag: *mut c_char,
    pub registry: *mut c_char,
    pub message: *mut c_char,
}

unsafe fn parse_optional_cstr(
    ptr: *const c_char,
    arg_name: &str,
) -> Result<Option<String>, String> {
    if ptr.is_null() {
        return Ok(None);
    }
    let value = read_cstr(ptr, arg_name)?;
    if value.is_empty() {
        Ok(None)
    } else {
        Ok(Some(value))
    }
}

thread_local! {
    static LAST_ERROR: RefCell<Option<CString>> = const { RefCell::new(None) };
}

fn set_last_error(msg: impl Into<String>) {
    let fallback = CString::new("unknown error").expect("CString literal");
    let cmsg = CString::new(msg.into()).unwrap_or(fallback);
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = Some(cmsg);
    });
}

fn clear_last_error() {
    LAST_ERROR.with(|slot| {
        *slot.borrow_mut() = None;
    });
}

fn read_cstr(ptr: *const c_char, arg_name: &str) -> Result<String, String> {
    if ptr.is_null() {
        return Err(format!("{} was null", arg_name));
    }
    // SAFETY: caller guarantees a valid null-terminated C string pointer.
    let s = unsafe { CStr::from_ptr(ptr) };
    s.to_str()
        .map(|v| v.to_string())
        .map_err(|_| format!("{} was not valid UTF-8", arg_name))
}

fn cstring_from_string(s: String) -> *mut c_char {
    match CString::new(s) {
        Ok(v) => v.into_raw(),
        Err(_) => {
            set_last_error("result string contained interior NUL byte");
            std::ptr::null_mut()
        }
    }
}

fn cstring_from_opt(s: Option<String>) -> *mut c_char {
    match s {
        Some(v) => cstring_from_string(v),
        None => std::ptr::null_mut(),
    }
}

fn parse_registry(registry: *const c_char) -> Result<Option<String>, String> {
    if registry.is_null() {
        Ok(None)
    } else {
        read_cstr(registry, "registry").map(Some)
    }
}

fn parse_bag_ref(bag_ref: *const c_char) -> Result<crate::BagRef, String> {
    let bag_ref = read_cstr(bag_ref, "bag_ref")?;
    bag_ref
        .parse()
        .map_err(|e| format!("invalid bag reference: {e}"))
}

struct CCallbackProgress {
    callback: extern "C" fn(*const c_char, *const c_char, *mut c_void),
    user_data: *mut c_void,
}

impl ProgressSink for CCallbackProgress {
    fn emit(&mut self, event: ProgressEvent) {
        let phase = match CString::new(event.phase) {
            Ok(v) => v,
            Err(_) => return,
        };
        let message = match CString::new(event.message) {
            Ok(v) => v,
            Err(_) => return,
        };
        (self.callback)(phase.as_ptr(), message.as_ptr(), self.user_data);
    }
}

fn do_pull_with_progress(
    bag: crate::BagRef,
    registry: Option<String>,
    progress_mode: i32,
    callback: MarinaProgressCallback,
    user_data: *mut c_void,
) -> *mut c_char {
    let mut marina = match Marina::load() {
        Ok(v) => v,
        Err(e) => {
            set_last_error(format!("failed to load marina: {e}"));
            return std::ptr::null_mut();
        }
    };

    let rt = match tokio::runtime::Runtime::new() {
        Ok(v) => v,
        Err(e) => {
            set_last_error(format!("failed to create tokio runtime: {e}"));
            return std::ptr::null_mut();
        }
    };

    let path = if let Some(cb) = callback {
        let mut sink = CCallbackProgress {
            callback: cb,
            user_data,
        };
        let mut progress = ProgressReporter::new(&mut sink);
        rt.block_on(marina.pull_exact_with_progress(&bag, registry.as_deref(), &mut progress))
    } else if progress_mode == MARINA_PROGRESS_MODE_STDOUT {
        let mut stdout = std::io::stdout();
        let mut sink = WriterProgress::new(&mut stdout);
        let mut progress = ProgressReporter::new(&mut sink);
        rt.block_on(marina.pull_exact_with_progress(&bag, registry.as_deref(), &mut progress))
    } else {
        rt.block_on(marina.pull_exact(&bag, registry.as_deref()))
    };

    match path {
        Ok(v) => cstring_from_string(v.display().to_string()),
        Err(e) => {
            set_last_error(format!("pull failed: {e}"));
            std::ptr::null_mut()
        }
    }
}

fn detailed_error(msg: String) -> MarinaResolveDetailed {
    set_last_error(msg.clone());
    MarinaResolveDetailed {
        kind: MARINA_RESOLVE_ERROR,
        path: std::ptr::null_mut(),
        bag: std::ptr::null_mut(),
        registry: std::ptr::null_mut(),
        message: cstring_from_string(msg),
    }
}

/// # Safety
/// `target` must be a valid, non-null, null-terminated C string.
/// `registry` must be a valid null-terminated C string or null.
pub unsafe extern "C" fn marina_resolve_detailed(
    target: *const c_char,
    registry: *const c_char,
) -> MarinaResolveDetailed {
    clear_last_error();

    let target = match read_cstr(target, "target") {
        Ok(v) => v,
        Err(e) => return detailed_error(e),
    };

    let registry = match unsafe { parse_optional_cstr(registry, "registry") } {
        Ok(v) => v,
        Err(e) => return detailed_error(e),
    };

    let marina = match Marina::load() {
        Ok(v) => v,
        Err(e) => return detailed_error(format!("failed to load marina: {e}")),
    };

    let rt = match tokio::runtime::Runtime::new() {
        Ok(v) => v,
        Err(e) => return detailed_error(format!("failed to create tokio runtime: {e}")),
    };

    match rt.block_on(marina.resolve_target(&target, registry.as_deref())) {
        Ok(ResolveResult::LocalPath(p)) => MarinaResolveDetailed {
            kind: MARINA_RESOLVE_LOCAL,
            path: cstring_from_string(p.display().to_string()),
            bag: std::ptr::null_mut(),
            registry: std::ptr::null_mut(),
            message: cstring_from_string("local path resolved".to_string()),
        },
        Ok(ResolveResult::Cached(p)) => MarinaResolveDetailed {
            kind: MARINA_RESOLVE_CACHED,
            path: cstring_from_string(p.display().to_string()),
            bag: std::ptr::null_mut(),
            registry: std::ptr::null_mut(),
            message: cstring_from_string("cached path resolved".to_string()),
        },
        Ok(ResolveResult::RemoteAvailable { bag, registry, .. }) => MarinaResolveDetailed {
            kind: MARINA_RESOLVE_REMOTE_AVAILABLE,
            path: std::ptr::null_mut(),
            bag: cstring_from_string(bag.to_string()),
            registry: cstring_from_string(registry),
            message: cstring_from_string(
                "remote bag available. Call marina_pull(...) to fetch".to_string(),
            ),
        },
        Ok(ResolveResult::Ambiguous { mut candidates }) => {
            candidates.sort_by(|a, b| a.0.cmp(&b.0));
            let (registry, bag) = candidates.remove(0);
            MarinaResolveDetailed {
                kind: MARINA_RESOLVE_AMBIGUOUS,
                path: std::ptr::null_mut(),
                bag: cstring_from_string(bag.to_string()),
                registry: cstring_from_string(registry),
                message: cstring_from_string(
                    "bag found in multiple registries. First registry selected".to_string(),
                ),
            }
        }
        Err(e) => detailed_error(format!("resolve failed: {e}")),
    }
}

/// # Safety
/// `result` must be a pointer obtained from `marina_resolve_detailed`, or null.
pub unsafe extern "C" fn marina_free_resolve_detailed(result: *mut MarinaResolveDetailed) {
    if result.is_null() {
        return;
    }
    // SAFETY: caller passes pointer to a struct obtained from marina_resolve_detailed.
    unsafe {
        marina_free_string((*result).path);
        marina_free_string((*result).bag);
        marina_free_string((*result).registry);
        marina_free_string((*result).message);
        (*result).path = std::ptr::null_mut();
        (*result).bag = std::ptr::null_mut();
        (*result).registry = std::ptr::null_mut();
        (*result).message = std::ptr::null_mut();
    }
}

/// # Safety
/// `target` must be a valid, non-null, null-terminated C string.
/// `registry` must be a valid null-terminated C string or null.
pub unsafe extern "C" fn marina_resolve(
    target: *const c_char,
    registry: *const c_char,
) -> *mut c_char {
    let mut detailed = unsafe { marina_resolve_detailed(target, registry) };
    let out = if detailed.kind == MARINA_RESOLVE_LOCAL || detailed.kind == MARINA_RESOLVE_CACHED {
        cstring_from_opt(Some(
            // SAFETY: pointer generated by this library.
            unsafe { CStr::from_ptr(detailed.path) }
                .to_string_lossy()
                .to_string(),
        ))
    } else if detailed.kind == MARINA_RESOLVE_REMOTE_AVAILABLE {
        let bag = if detailed.bag.is_null() {
            "<unknown>".to_string()
        } else {
            // SAFETY: pointer generated by this library.
            unsafe { CStr::from_ptr(detailed.bag) }
                .to_string_lossy()
                .to_string()
        };
        let registry = if detailed.registry.is_null() {
            "<unknown>".to_string()
        } else {
            // SAFETY: pointer generated by this library.
            unsafe { CStr::from_ptr(detailed.registry) }
                .to_string_lossy()
                .to_string()
        };
        cstring_from_string(format!("REMOTE:{}@{}", bag, registry))
    } else {
        std::ptr::null_mut()
    };
    // SAFETY: detailed was allocated by marina_resolve_detailed above.
    unsafe { marina_free_resolve_detailed(&mut detailed) };
    out
}

pub extern "C" fn marina_pull(bag_ref: *const c_char, registry: *const c_char) -> *mut c_char {
    clear_last_error();

    let bag = match parse_bag_ref(bag_ref) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    let registry = match parse_registry(registry) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    do_pull_with_progress(
        bag,
        registry,
        MARINA_PROGRESS_MODE_SILENT,
        None,
        std::ptr::null_mut(),
    )
}

pub extern "C" fn marina_pull_with_progress(
    bag_ref: *const c_char,
    registry: *const c_char,
    progress_mode: i32,
) -> *mut c_char {
    clear_last_error();

    let bag = match parse_bag_ref(bag_ref) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    let registry = match parse_registry(registry) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    do_pull_with_progress(bag, registry, progress_mode, None, std::ptr::null_mut())
}

pub extern "C" fn marina_pull_with_callback(
    bag_ref: *const c_char,
    registry: *const c_char,
    callback: MarinaProgressCallback,
    user_data: *mut c_void,
) -> *mut c_char {
    clear_last_error();

    let bag = match parse_bag_ref(bag_ref) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    let registry = match parse_registry(registry) {
        Ok(v) => v,
        Err(e) => {
            set_last_error(e);
            return std::ptr::null_mut();
        }
    };

    do_pull_with_progress(
        bag,
        registry,
        MARINA_PROGRESS_MODE_SILENT,
        callback,
        user_data,
    )
}

pub extern "C" fn marina_last_error_message() -> *mut c_char {
    LAST_ERROR.with(|slot| match &*slot.borrow() {
        Some(err) => match CString::new(err.to_bytes()) {
            Ok(copy) => copy.into_raw(),
            Err(_) => std::ptr::null_mut(),
        },
        None => std::ptr::null_mut(),
    })
}

/// # Safety
/// `ptr` must be a pointer obtained from a marina string-returning function, or null.
pub unsafe extern "C" fn marina_free_string(ptr: *mut c_char) {
    if ptr.is_null() {
        return;
    }
    // SAFETY: pointer must have been allocated by CString::into_raw in this library.
    unsafe {
        let _ = CString::from_raw(ptr);
    }
}
