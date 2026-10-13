use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use log::{info, warn};

pub const DEFAULT_ZENOH_PORT: u16 = 7447;

/// Default endpoint; configured sessions honor `MINOT_ZENOH_PORT`.
pub const LOCAL_COORD_ENDPOINT: &str = "tcp/127.0.0.1:7447";
/// Default endpoint; configured sessions honor `MINOT_ZENOH_PORT`.
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

/// TCP port used for the local coordinator probe and coordinator endpoints.
///
/// Set `MINOT_ZENOH_PORT` before creating any Minot sessions. All nodes sharing
/// a coordinator must use the same value. Unset values use 7447; invalid or
/// zero values warn and fall back to 7447. Explicit `MINOT_COORD_ADDR`
/// endpoints retain their own port.
pub fn coordinator_port() -> u16 {
    match std::env::var("MINOT_ZENOH_PORT") {
        Ok(value) => match value.parse::<u16>() {
            Ok(port) if port != 0 => port,
            _ => {
                warn!("Invalid MINOT_ZENOH_PORT '{value}', selecting default {DEFAULT_ZENOH_PORT}");
                DEFAULT_ZENOH_PORT
            }
        },
        Err(std::env::VarError::NotPresent) => DEFAULT_ZENOH_PORT,
        Err(error) => {
            warn!("Invalid MINOT_ZENOH_PORT: {error}, selecting default {DEFAULT_ZENOH_PORT}");
            DEFAULT_ZENOH_PORT
        }
    }
}

/// Return whether something is accepting connections on the local coordinator endpoint.
/// This is used to decide whether an auto-start node must create the router before
/// opening its client session. It preserves local-only configuration.
pub fn local_router_is_running() -> bool {
    let address = SocketAddr::from(([127, 0, 0, 1], coordinator_port()));
    TcpStream::connect_timeout(&address, Duration::from_millis(100)).is_ok()
}

pub fn zenoh_config(role: NetworkRole) -> zenoh::Config {
    config_for(
        role,
        is_local_only(),
        is_unicast_only(),
        std::env::var("MINOT_COORD_ADDR").ok(),
        role == NetworkRole::Coordinator && local_router_is_running(),
        coordinator_port(),
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
    port: u16,
) -> zenoh::Config {
    let local_endpoint = format!("tcp/127.0.0.1:{port}");
    let network_endpoint = format!("tcp/0.0.0.0:{port}");
    if local_only {
        if coordinator_addr.is_some() {
            warn!("--local-only selected. Using {}", local_endpoint);
        }

        let json5 = match (role, local_router_running) {
            (NetworkRole::Coordinator, false) => format!(
                r#"{{mode:"router",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},listen:{{endpoints:["{}"]}}}}"#,
                local_endpoint
            ),
            // An external Zenoh router owns the endpoint. Run the Minot
            // coordinator logic on a client session connected to that router.
            (NetworkRole::Coordinator, true) | (NetworkRole::Client, _) => format!(
                r#"{{mode:"client",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},connect:{{endpoints:["{}"],exit_on_failure:false}}}}"#,
                local_endpoint
            ),
        };

        info!("Using local-only networking on {}", local_endpoint);
        return zenoh::Config::from_json5(&json5)
            .expect("internal local-only Zenoh configuration must be valid");
    }

    if role == NetworkRole::Coordinator {
        let multicast = if unicast_only { "false" } else { "true" };
        let json5 = format!(
            r#"{{mode:"router",scouting:{{multicast:{{enabled:{multicast}}},gossip:{{enabled:{multicast}}}}},listen:{{endpoints:["{}"]}}}}"#,
            network_endpoint
        );
        info!("Using network coordinator endpoint on {}", network_endpoint);
        return zenoh::Config::from_json5(&json5)
            .expect("internal network coordinator Zenoh configuration must be valid");
    }

    if unicast_only && coordinator_addr.is_none() {
        let json5 = format!(
            r#"{{mode:"client",scouting:{{multicast:{{enabled:false}},gossip:{{enabled:false}}}},connect:{{endpoints:["{}"],exit_on_failure:false}}}}"#,
            local_endpoint
        );
        info!("Client using local unicast coordinator: {local_endpoint}");
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn configured_port_reaches_listeners_and_local_clients() {
        for port in [DEFAULT_ZENOH_PORT, 17447] {
            for (local_only, unicast_only) in [(true, false), (false, true)] {
                let router = config_for(
                    NetworkRole::Coordinator,
                    local_only,
                    unicast_only,
                    None,
                    false,
                    port,
                );
                let host = if local_only { "127.0.0.1" } else { "0.0.0.0" };
                assert_eq!(
                    router.get_json("listen/endpoints").unwrap(),
                    format!(r#"["tcp/{host}:{port}"]"#)
                );
                let client = config_for(
                    NetworkRole::Client,
                    local_only,
                    unicast_only,
                    None,
                    false,
                    port,
                );
                assert_eq!(
                    client.get_json("connect/endpoints").unwrap(),
                    format!(r#"["tcp/127.0.0.1:{port}"]"#)
                );
            }
            let shared_router = config_for(NetworkRole::Coordinator, true, false, None, true, port);
            assert_eq!(
                shared_router.get_json("connect/endpoints").unwrap(),
                format!(r#"["tcp/127.0.0.1:{port}"]"#)
            );
        }
    }

    #[test]
    fn explicit_coordinator_address_keeps_its_port() {
        let config = config_for(
            NetworkRole::Client,
            false,
            false,
            Some("tcp/192.0.2.1:23456".into()),
            false,
            17447,
        );
        assert_eq!(
            config.get_json("connect/endpoints").unwrap(),
            r#"["tcp/192.0.2.1:23456"]"#
        );
    }
}
