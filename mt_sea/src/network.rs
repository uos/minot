use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use log::{info, warn};

pub const LOCAL_COORD_ENDPOINT: &str = "tcp/127.0.0.1:7447";
pub const NETWORK_COORD_ENDPOINT: &str = "tcp/0.0.0.0:7447";

static LOCAL_ONLY: AtomicBool = AtomicBool::new(false);
/// Dedicated services can listen on the LAN without merging with every Minot
/// router found through multicast discovery.
static UNICAST_ONLY: AtomicBool = AtomicBool::new(false);

#[cfg(feature = "shm")]
static SHM_RUNTIME_AVAILABLE: AtomicBool = AtomicBool::new(true);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NetworkRole {
    Client,
    Coordinator,
}

/// Restrict subsequently created Minot Zenoh sessions to IPv4 loopback.
///
/// This must be called before starting any clients or coordinators.
pub fn set_local_only(enabled: bool) {
    LOCAL_ONLY.store(enabled, Ordering::SeqCst);
}

pub fn is_local_only() -> bool {
    LOCAL_ONLY.load(Ordering::SeqCst)
}

/// Disable discovery for subsequently created sessions while retaining the
/// network listening address. Must be called before creating any sessions.
pub fn set_unicast_only(enabled: bool) {
    UNICAST_ONLY.store(enabled, Ordering::SeqCst);
}

pub fn is_unicast_only() -> bool {
    UNICAST_ONLY.load(Ordering::SeqCst)
}

#[cfg(feature = "shm")]
pub(crate) fn is_shm_runtime_available() -> bool {
    SHM_RUNTIME_AVAILABLE.load(Ordering::Acquire)
}

#[cfg(feature = "shm")]
pub(crate) fn is_shm_disabled() -> bool {
    std::env::var("MINOT_SHM_DISABLED")
        .map(|value| value == "1" || value.eq_ignore_ascii_case("true"))
        .unwrap_or(false)
}

#[cfg(feature = "shm")]
pub(crate) fn disable_shm_runtime() {
    SHM_RUNTIME_AVAILABLE.store(false, Ordering::Release);
}

/// Return whether something is accepting connections on the local coordinator endpoint.
/// This is used to decide whether an auto-start node must create the router before
/// opening its client session. It preserves local-only configuration.
pub fn local_router_is_running() -> bool {
    let address: SocketAddr = "127.0.0.1:7447"
        .parse()
        .expect("the local coordinator address must be valid");
    TcpStream::connect_timeout(&address, Duration::from_millis(100)).is_ok()
}

pub fn zenoh_config(role: NetworkRole) -> zenoh::Config {
    config_for(
        role,
        is_local_only(),
        is_unicast_only(),
        std::env::var("MINOT_COORD_ADDR").ok(),
        role == NetworkRole::Coordinator && local_router_is_running(),
    )
}

/// Open a Zenoh session, retrying without shared memory if SHM initialization fails.
pub(crate) fn open_zenoh_session(role: NetworkRole) -> anyhow::Result<zenoh::Session> {
    use zenoh::Wait;

    let config = zenoh_config(role);
    #[cfg(feature = "shm")]
    let config = {
        let mut config = config;
        if is_shm_runtime_available() && !is_shm_disabled() {
            disable_implicit_shm_transport(&mut config)?;
        } else {
            disable_shm_in_config(&mut config)?;
        }
        config
    };
    #[cfg(not(feature = "shm"))]
    let config = {
        let mut config = config;
        // Be explicit even though Zenoh was compiled without its SHM implementation:
        // this peer must never advertise SHM capability during link negotiation.
        disable_shm_in_config(&mut config)?;
        config
    };

    match zenoh::open(config).wait() {
        Ok(session) => Ok(session),
        Err(error) => {
            #[cfg(feature = "shm")]
            {
                if !is_shm_initialization_error(&error.to_string()) {
                    return Err(anyhow::anyhow!("Failed to open Zenoh session: {error}"));
                }
                warn!("Zenoh SHM initialization failed: {error}. Retrying with network transport");
                disable_shm_runtime();
                let mut fallback = zenoh_config(role);
                disable_shm_in_config(&mut fallback)?;
                return zenoh::open(fallback)
                    .wait()
                    .map_err(|fallback_error| {
                        anyhow::anyhow!(
                            "Failed to open Zenoh session with SHM ({error}) and without SHM ({fallback_error})"
                        )
                    });
            }

            #[cfg(not(feature = "shm"))]
            Err(anyhow::anyhow!("Failed to open Zenoh session: {error}"))
        }
    }
}

#[cfg(feature = "shm")]
fn is_shm_initialization_error(error: &str) -> bool {
    let error = error.to_ascii_lowercase();
    error.contains("shared memory")
        || error.contains("shared-memory")
        || error.contains("posix shm")
        || error.contains("shm segment")
}

fn disable_shm_in_config(config: &mut zenoh::Config) -> anyhow::Result<()> {
    config
        .insert_json5("transport/shared_memory/enabled", "false")
        .map_err(|error| anyhow::anyhow!("Failed to disable Zenoh shared memory: {error}"))?;
    disable_implicit_shm_transport(config)?;
    Ok(())
}

fn disable_implicit_shm_transport(config: &mut zenoh::Config) -> anyhow::Result<()> {
    config
        .insert_json5(
            "transport/shared_memory/transport_optimization/enabled",
            "false",
        )
        .map_err(|error| {
            anyhow::anyhow!("Failed to disable Zenoh SHM transport optimization: {error}")
        })
}

fn config_for(
    role: NetworkRole,
    local_only: bool,
    unicast_only: bool,
    coordinator_addr: Option<String>,
    local_router_running: bool,
) -> zenoh::Config {
    if local_only {
        if coordinator_addr.is_some() {
            warn!("--local-only selected. Using {}", LOCAL_COORD_ENDPOINT);
        }

        let json5 = match (role, local_router_running) {
            (NetworkRole::Coordinator, false) => format!(
                r#"{{mode:"router",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},listen:{{endpoints:["{}"]}}}}"#,
                LOCAL_COORD_ENDPOINT
            ),
            // An external Zenoh router owns the endpoint. Run the Minot
            // coordinator logic on a client session connected to that router.
            (NetworkRole::Coordinator, true) | (NetworkRole::Client, _) => format!(
                r#"{{mode:"client",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},connect:{{endpoints:["{}"],exit_on_failure:false}}}}"#,
                LOCAL_COORD_ENDPOINT
            ),
        };

        info!("Using local-only networking on {}", LOCAL_COORD_ENDPOINT);
        return zenoh::Config::from_json5(&json5)
            .expect("internal local-only Zenoh configuration must be valid");
    }

    if role == NetworkRole::Coordinator {
        let multicast = if unicast_only { "false" } else { "true" };
        let json5 = format!(
            r#"{{mode:"router",scouting:{{multicast:{{enabled:{multicast}}},gossip:{{enabled:{multicast}}}}},listen:{{endpoints:["{}"]}}}}"#,
            NETWORK_COORD_ENDPOINT
        );
        info!(
            "Using network coordinator endpoint on {}",
            NETWORK_COORD_ENDPOINT
        );
        return zenoh::Config::from_json5(&json5)
            .expect("internal network coordinator Zenoh configuration must be valid");
    }

    if unicast_only && coordinator_addr.is_none() {
        let json5 = format!(
            r#"{{mode:"client",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},connect:{{endpoints:["{}"],exit_on_failure:false}}}}"#,
            LOCAL_COORD_ENDPOINT
        );
        info!("Client using local unicast coordinator: {LOCAL_COORD_ENDPOINT}");
        return zenoh::Config::from_json5(&json5)
            .expect("internal unicast Zenoh configuration must be valid");
    }

    if let Some(addr) = coordinator_addr {
        let json5 = format!(
            r#"{{mode:"peer",scouting:{{multicast:{{enabled:false}}}},connect:{{endpoints:["{}"]}}}}"#,
            addr
        );
        return match zenoh::Config::from_json5(&json5) {
            Ok(config) => {
                info!("Client using unicast coordinator: {}", addr);
                config
            }
            Err(error) => {
                warn!(
                    "MINOT_COORD_ADDR '{}' produced invalid config: {}, falling back to multicast",
                    addr, error
                );
                zenoh::Config::default()
            }
        };
    }

    zenoh::Config::default()
}
