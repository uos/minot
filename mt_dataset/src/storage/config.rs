use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow};
use log::warn;
use ratslang::millimeter;
use ratslang::uom::si::time::second;
use ratslang::{Rhs, UnitVal, Val, Var, VariableHistory, resolve_var};

const DEFAULT_CONFIG: &str = include_str!("../../default.rl");

#[derive(Debug, Clone)]
pub struct RegistryConfig {
    pub name: String,
    pub kind: String,
    pub uri: String,
    pub auth_env: Option<String>,
    pub proxy_jump: Option<String>,
    pub ssh_transport: Option<String>,
    pub download_mode: RegistryDownloadMode,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum RegistryDownloadMode {
    #[default]
    Adaptive,
    Streaming,
}

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum TimeDisplay {
    #[default]
    Relative,
    Absolute,
}

#[derive(Debug, Clone)]
pub struct Settings {
    pub time_display: TimeDisplay,
    pub default_registry: Option<String>,
    pub completion_cache_ttl_secs: u64,
    pub registry_timeout_secs: u64,
}

impl Default for Settings {
    fn default() -> Self {
        Self {
            time_display: TimeDisplay::Relative,
            default_registry: None,
            completion_cache_ttl_secs: 600,
            registry_timeout_secs: 10,
        }
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub enum ConfigPointcloudMode {
    Off,
    #[default]
    Lossy,
    Lossless,
}

#[derive(Debug, Clone, Copy, Default)]
pub enum ConfigMcapCompression {
    None,
    #[default]
    Zstd,
    Lz4,
}

#[derive(Debug, Clone, Copy, Default)]
pub enum ConfigArchiveCompression {
    #[default]
    None,
    Gzip,
}

#[derive(Debug, Clone, Copy)]
pub struct CompressionConfig {
    pub pointcloud_mode: ConfigPointcloudMode,
    pub pointcloud_accuracy_mm: f64,
    pub packed_mcap_compression: ConfigMcapCompression,
    pub packed_archive_compression: ConfigArchiveCompression,
    pub unpacked_mcap_compression: ConfigMcapCompression,
}

impl Default for CompressionConfig {
    fn default() -> Self {
        Self {
            pointcloud_mode: ConfigPointcloudMode::Lossy,
            pointcloud_accuracy_mm: 1.0,
            packed_mcap_compression: ConfigMcapCompression::Zstd,
            packed_archive_compression: ConfigArchiveCompression::None,
            unpacked_mcap_compression: ConfigMcapCompression::Lz4,
        }
    }
}

#[derive(Debug, Clone)]
pub struct RegistryFile {
    pub registry: Vec<RegistryConfig>,
    pub compression: CompressionConfig,
    pub settings: Settings,
}

/// Satisfies the field-access pattern expected by the `resolve_var!` macro:
/// tries `user` first, falls back to `defaults`.
struct Asts {
    user: VariableHistory,
    defaults: VariableHistory,
}

impl Asts {
    fn filter_ns(&self, path: &[&str]) -> Asts {
        Asts {
            user: self.user.filter_ns(path),
            defaults: self.defaults.filter_ns(path),
        }
    }
}

pub fn config_dir() -> Result<PathBuf> {
    let dir = if let Some(override_dir) = std::env::var_os("MARINA_CONFIG_DIR") {
        PathBuf::from(override_dir)
    } else if let Some(xdg) = std::env::var_os("XDG_CONFIG_HOME") {
        PathBuf::from(xdg).join("marina")
    } else if let Some(home) = std::env::var_os("HOME") {
        PathBuf::from(home).join(".config").join("marina")
    } else {
        dirs::config_dir()
            .context("unable to locate config dir")?
            .join("marina")
    };
    fs::create_dir_all(&dir)?;
    Ok(dir)
}

pub fn cache_dir() -> Result<PathBuf> {
    let dir = if let Some(override_dir) = std::env::var_os("MARINA_CACHE_DIR") {
        PathBuf::from(override_dir)
    } else if let Some(home) = std::env::var_os("HOME") {
        PathBuf::from(home).join(".cache").join("marina")
    } else {
        dirs::cache_dir()
            .context("unable to locate cache dir")?
            .join("marina")
    };
    fs::create_dir_all(&dir)?;
    Ok(dir)
}

pub fn registry_file_path() -> Result<PathBuf> {
    Ok(config_dir()?.join("marina.rl"))
}

pub fn ensure_dir(path: &Path) -> Result<()> {
    fs::create_dir_all(path)?;
    Ok(())
}

pub fn infer_kind_from_uri(uri: &str) -> &'static str {
    if uri.starts_with("minot://") || uri.starts_with("minot+ssh://") {
        "minot"
    } else if uri.starts_with("ssh://") {
        "ssh"
    } else if uri.starts_with("gdrive://") {
        "gdrive"
    } else if uri.starts_with("http://") || uri.starts_with("https://") {
        "http"
    } else if uri.starts_with("directory://") {
        "directory"
    } else {
        "folder"
    }
}

pub const DEFAULT_REGISTRY_NAME: &str = "osnabotics_public";
pub const DEFAULT_GDRIVE_FOLDER_ID: &str = "10hjoMIyWTOVNOo3zDOfHoSb1S55gO3rJ";

fn generate_initial_config() -> String {
    let mut content = DEFAULT_CONFIG.trim_end().to_string();

    #[cfg(feature = "osnabotics-default-registry")]
    {
        content.push_str(concat!(
            "\n\nregistries {\n",
            "  osnabotics_public {\n",
            "    uri = \"gdrive://10hjoMIyWTOVNOo3zDOfHoSb1S55gO3rJ\"\n",
            "    kind = \"gdrive\"\n",
            "  }\n",
            "}\n",
        ));
    }

    #[cfg(not(feature = "osnabotics-default-registry"))]
    {
        content.push('\n');
    }

    content
}

#[allow(clippy::unnecessary_fallible_conversions)]
pub fn load_registries() -> Result<RegistryFile> {
    let path = registry_file_path()?;

    // Migration hint: config.rl → marina.rl
    let old_rl = config_dir()?.join("config.rl");
    if old_rl.exists() && !path.exists() {
        fs::rename(&old_rl, &path).with_context(|| {
            format!("failed renaming {} to {}", old_rl.display(), path.display())
        })?;
        warn!("renamed {} to {}", old_rl.display(), path.display());
    }

    // Migration hint for users upgrading from the old TOML format.
    let old_toml = config_dir()?.join("registries.toml");
    if old_toml.exists() && !path.exists() {
        warn!(
            "found old config at {} — marina now uses {}\n      re-add your registries with `marina registry add`",
            old_toml.display(),
            path.display()
        );
    }

    let user_content = if path.exists() {
        fs::read_to_string(&path).with_context(|| format!("failed reading {}", path.display()))?
    } else {
        let content = generate_initial_config();
        fs::write(&path, &content).with_context(|| format!("failed writing {}", path.display()))?;
        content
    };

    let default_ast = ratslang::compile_code(DEFAULT_CONFIG)
        .context("failed to compile embedded default config")?;
    let user_ast = ratslang::compile_code(&user_content)
        .with_context(|| format!("failed to parse config at {}", path.display()))?;

    let mut default_vars = default_ast.vars;
    default_vars = default_vars.with_drop_warning(false);

    let asts = Asts {
        user: user_ast.vars,
        defaults: default_vars,
    };

    // Settings
    let settings = {
        let ns = asts.filter_ns(&["settings"]);

        let time_display = {
            let s: String = resolve_var!(ns, "time_display", as String,
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => { s })?;
            if s == "absolute" {
                TimeDisplay::Absolute
            } else {
                TimeDisplay::Relative
            }
        };

        let completion_cache_ttl_secs: u64 = resolve_var!(ns, "completion_cache_ttl", as u64,
            Rhs::Val(Val::UnitedVal(UnitVal::TimeVal(t))) => { t.get::<second>() as u64 })?;

        let registry_timeout_secs: u64 = resolve_var!(ns, "registry_timeout", as u64,
            Rhs::Val(Val::UnitedVal(UnitVal::TimeVal(t))) => { t.get::<second>() as u64 })?;

        let default_registry = match ns.user.resolve("default_registry")? {
            Some(Rhs::Val(Val::StringVal(s))) | Some(Rhs::Path(s)) if !s.is_empty() => Some(s),
            _ => None,
        };

        Settings {
            time_display,
            default_registry,
            completion_cache_ttl_secs,
            registry_timeout_secs,
        }
    };

    // Compression
    let compression = {
        let ns = asts.filter_ns(&["compression"]);

        let pointcloud_mode = {
            let s: String = resolve_var!(ns, "pointcloud_mode", as String,
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => { s })?;
            match s.as_str() {
                "off" => ConfigPointcloudMode::Off,
                "lossless" => ConfigPointcloudMode::Lossless,
                _ => ConfigPointcloudMode::Lossy,
            }
        };

        let pointcloud_accuracy_mm: f64 = resolve_var!(ns, "pointcloud_accuracy", as f64,
            Rhs::Val(Val::UnitedVal(UnitVal::LengthVal(l))) => { l.get::<millimeter>() })?;

        let packed_mcap_compression = {
            let s: String = resolve_var!(ns, "packed_mcap_compression", as String,
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => { s })?;
            parse_mcap_compression(&s)
        };

        let packed_archive_compression = {
            let s: String = resolve_var!(ns, "packed_archive_compression", as String,
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => { s })?;
            if s == "gzip" {
                ConfigArchiveCompression::Gzip
            } else {
                ConfigArchiveCompression::None
            }
        };

        let unpacked_mcap_compression = {
            let s: String = resolve_var!(ns, "unpacked_mcap_compression", as String,
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => { s })?;
            parse_mcap_compression(&s)
        };

        CompressionConfig {
            pointcloud_mode,
            pointcloud_accuracy_mm,
            packed_mcap_compression,
            packed_archive_compression,
            unpacked_mcap_compression,
        }
    };

    let registry = parse_registries(&asts.user)?;

    Ok(RegistryFile {
        registry,
        compression,
        settings,
    })
}

fn parse_mcap_compression(s: &str) -> ConfigMcapCompression {
    match s.to_lowercase().as_str() {
        "zstd" => ConfigMcapCompression::Zstd,
        "lz4" => ConfigMcapCompression::Lz4,
        _ => ConfigMcapCompression::None,
    }
}

fn parse_registries(user_vars: &VariableHistory) -> Result<Vec<RegistryConfig>> {
    let pairs = user_vars.resolve_ns(&["registries"]);

    // Collect unique registry identifiers (first namespace component after "registries").
    let mut reg_names: Vec<String> = Vec::new();
    let mut seen = HashSet::new();
    for (var, _) in &pairs {
        let ns = match var {
            Var::User { namespace, .. } | Var::Predef { namespace, .. } => namespace,
        };
        if let Some(first) = ns.first() {
            if seen.insert(first.clone()) {
                reg_names.push(first.clone());
            }
        }
    }

    let mut result = Vec::new();
    for ident in reg_names {
        let reg_ns = user_vars.filter_ns(&["registries", &ident]);

        let uri = match reg_ns.resolve("uri")? {
            Some(Rhs::Val(Val::StringVal(s))) | Some(Rhs::Path(s)) => s,
            Some(_) => return Err(anyhow!("registry '{}': 'uri' must be a string", ident)),
            None => return Err(anyhow!("registry '{}' is missing 'uri'", ident)),
        };

        let kind = reg_ns
            .resolve("kind")?
            .and_then(|r| match r {
                Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) => Some(s),
                _ => None,
            })
            .unwrap_or_else(|| infer_kind_from_uri(&uri).to_string());

        let auth_env = reg_ns.resolve("auth_env")?.and_then(|r| match r {
            Rhs::Val(Val::StringVal(s)) => Some(s),
            _ => None,
        });
        let proxy_jump = reg_ns.resolve("proxy_jump")?.and_then(|r| match r {
            Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) if !s.is_empty() => Some(s),
            _ => None,
        });
        let ssh_transport = reg_ns.resolve("ssh_transport")?.and_then(|r| match r {
            Rhs::Val(Val::StringVal(s)) | Rhs::Path(s) if !s.is_empty() => Some(s),
            _ => None,
        });
        let download_mode = reg_ns
            .resolve("download_mode")?
            .and_then(|r| match r {
                Rhs::Val(Val::StringVal(s)) => match s.as_str() {
                    "adaptive" => Some(RegistryDownloadMode::Adaptive),
                    "streaming" => Some(RegistryDownloadMode::Streaming),
                    _ => None,
                },
                _ => None,
            })
            .unwrap_or(RegistryDownloadMode::Adaptive);

        result.push(RegistryConfig {
            name: ident,
            uri,
            kind,
            auth_env,
            proxy_jump,
            ssh_transport,
            download_mode,
        });
    }

    Ok(result)
}

pub fn save_registries(file: &RegistryFile) -> Result<()> {
    let path = registry_file_path()?;

    let current = if path.exists() {
        fs::read_to_string(&path).with_context(|| format!("failed reading {}", path.display()))?
    } else {
        DEFAULT_CONFIG.to_string()
    };

    let new_block = generate_registries_block(&file.registry);
    let new_content = replace_registries_block(&current, &new_block);

    fs::write(&path, new_content).with_context(|| format!("failed writing {}", path.display()))?;
    Ok(())
}

fn generate_registries_block(registries: &[RegistryConfig]) -> String {
    let mut s = String::from("registries {");
    for reg in registries {
        s.push_str(&format!("\n  {} {{\n", reg.name));
        s.push_str(&format!("    uri = \"{}\"\n", reg.uri));
        s.push_str(&format!("    kind = \"{}\"\n", reg.kind));
        if let Some(auth) = &reg.auth_env {
            s.push_str(&format!("    auth_env = \"{auth}\"\n"));
        }
        if let Some(proxy_jump) = &reg.proxy_jump {
            s.push_str(&format!("    proxy_jump = \"{proxy_jump}\"\n"));
        }
        if let Some(ssh_transport) = &reg.ssh_transport {
            s.push_str(&format!("    ssh_transport = \"{ssh_transport}\"\n"));
        }
        if reg.download_mode == RegistryDownloadMode::Streaming {
            s.push_str("    download_mode = \"streaming\"\n");
        }
        s.push_str("  }");
    }
    s.push_str("\n}");
    s
}

/// Replaces the `registries { … }` top-level block in `content` with `new_block`.
/// If no such block exists the new block is appended.
fn replace_registries_block(content: &str, new_block: &str) -> String {
    let lines: Vec<&str> = content.lines().collect();
    let mut block_start: Option<usize> = None;
    let mut block_end: Option<usize> = None;

    'outer: for (i, line) in lines.iter().enumerate() {
        let trimmed = line.trim();
        if trimmed == "registries {" || trimmed.starts_with("registries {") {
            block_start = Some(i);
            let mut depth = 0usize;
            for (j, &l) in lines[i..].iter().enumerate() {
                for c in l.chars() {
                    match c {
                        '{' => depth += 1,
                        '}' => {
                            depth -= 1;
                            if depth == 0 {
                                block_end = Some(i + j);
                                break 'outer;
                            }
                        }
                        _ => {}
                    }
                }
            }
        }
    }

    if let (Some(start), Some(end)) = (block_start, block_end) {
        let before = lines[..start].join("\n");
        let after = lines[end + 1..].join("\n");

        let mut out = String::new();
        let before_trimmed = before.trim_end();
        if !before_trimmed.is_empty() {
            out.push_str(before_trimmed);
            out.push_str("\n\n");
        }
        out.push_str(new_block);
        out.push('\n');
        let after_trimmed = after.trim();
        if !after_trimmed.is_empty() {
            out.push('\n');
            out.push_str(after_trimmed);
            out.push('\n');
        }
        out
    } else {
        // No existing block -> append
        let mut out = content.trim_end().to_string();
        if !out.is_empty() {
            out.push_str("\n\n");
        }
        out.push_str(new_block);
        out.push('\n');
        out
    }
}

pub fn load_compression_config() -> Result<CompressionConfig> {
    Ok(load_registries()?.compression)
}

pub fn remove_local_state(all: bool) -> Result<()> {
    let cdir = cache_dir()?;
    if cdir.exists() {
        fs::remove_dir_all(&cdir)?;
    }

    if all {
        let cfg = config_dir()?;
        let config_path = cfg.join("marina.rl");
        if config_path.exists() {
            fs::remove_file(&config_path)?;
        }
        // Remove old configs if present from previous installations.
        for old in &["registries.toml", "config.rl"] {
            let p = cfg.join(old);
            if p.exists() {
                fs::remove_file(&p)?;
            }
        }
        let catalog = cfg.join("catalog.json");
        if catalog.exists() {
            fs::remove_file(&catalog)?;
        }
    }

    Ok(())
}
