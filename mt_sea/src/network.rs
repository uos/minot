use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use log::{info, warn};

pub const LOCAL_COORD_ENDPOINT: &str = "tcp/127.0.0.1:7447";
pub const NETWORK_COORD_ENDPOINT: &str = "tcp/0.0.0.0:7447";

static LOCAL_ONLY: AtomicBool = AtomicBool::new(false);

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

#[cfg(feature = "shm")]
pub(crate) fn is_shm_runtime_available() -> bool {
    SHM_RUNTIME_AVAILABLE.load(Ordering::Acquire)
}

#[cfg(feature = "shm")]
pub(crate) fn disable_shm_runtime() {
    SHM_RUNTIME_AVAILABLE.store(false, Ordering::Release);
}

/// Return whether something is accepting connections on the local coordinator endpoint.
/// This is used to decide whether an auto-start node must create the router before
/// opening its client session; it never changes local-only configuration.
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
        if is_shm_runtime_available() {
            disable_implicit_shm_transport(&mut config)?;
        } else {
            disable_shm_in_config(&mut config)?;
        }
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
                warn!("Zenoh SHM initialization failed: {error}; retrying without shared memory");
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

#[cfg(feature = "shm")]
fn disable_shm_in_config(config: &mut zenoh::Config) -> anyhow::Result<()> {
    config
        .insert_json5("transport/shared_memory/enabled", "false")
        .map_err(|error| anyhow::anyhow!("Failed to disable Zenoh shared memory: {error}"))?;
    disable_implicit_shm_transport(config)?;
    Ok(())
}

#[cfg(feature = "shm")]
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
    coordinator_addr: Option<String>,
    local_router_running: bool,
) -> zenoh::Config {
    if local_only {
        if coordinator_addr.is_some() {
            warn!(
                "--local-only overrides MINOT_COORD_ADDR; using {}",
                LOCAL_COORD_ENDPOINT
            );
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
        let json5 = format!(
            r#"{{mode:"router",scouting:{{multicast:{{enabled:true}}}},listen:{{endpoints:["{}"]}}}}"#,
            NETWORK_COORD_ENDPOINT
        );
        info!(
            "Using network coordinator endpoint on {}",
            NETWORK_COORD_ENDPOINT
        );
        return zenoh::Config::from_json5(&json5)
            .expect("internal network coordinator Zenoh configuration must be valid");
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

    fn value(config: &zenoh::Config, key: &str) -> String {
        config.get_json(key).expect("config key should exist")
    }

    #[cfg(feature = "shm")]
    #[test]
    fn classifies_only_shm_startup_errors_for_fallback() {
        assert!(is_shm_initialization_error(
            "Unable to create POSIX shm segment: OS error 1"
        ));
        assert!(is_shm_initialization_error(
            "shared memory initialization failed"
        ));
        assert!(!is_shm_initialization_error(
            "Unable to connect to tcp/127.0.0.1:7447"
        ));
    }

    #[test]
    fn local_coordinator_only_listens_on_loopback() {
        let config = config_for(NetworkRole::Coordinator, true, None, false);
        let endpoints = value(&config, "listen/endpoints");
        assert_eq!(value(&config, "mode"), r#""router""#);
        assert!(endpoints.contains(LOCAL_COORD_ENDPOINT));
        assert!(!endpoints.contains("0.0.0.0"));
        assert_eq!(value(&config, "scouting/multicast/enabled"), "false");
        assert_eq!(value(&config, "scouting/gossip/enabled"), "false");
    }

    #[test]
    fn network_coordinator_listens_on_all_ipv4_interfaces() {
        let config = config_for(NetworkRole::Coordinator, false, None, false);
        let endpoints = value(&config, "listen/endpoints");
        assert_eq!(value(&config, "mode"), r#""router""#);
        assert!(endpoints.contains(NETWORK_COORD_ENDPOINT));
        assert!(!endpoints.contains(LOCAL_COORD_ENDPOINT));
        assert_eq!(value(&config, "scouting/multicast/enabled"), "true");
    }

    #[test]
    fn local_client_connects_to_loopback_and_retries() {
        let config = config_for(NetworkRole::Client, true, None, false);
        assert_eq!(value(&config, "mode"), r#""client""#);
        assert!(value(&config, "connect/endpoints").contains(LOCAL_COORD_ENDPOINT));
        assert_eq!(value(&config, "connect/exit_on_failure"), "false");
        assert_eq!(value(&config, "scouting/multicast/enabled"), "false");
        assert_eq!(value(&config, "scouting/gossip/enabled"), "false");
    }

    #[test]
    fn local_only_overrides_explicit_coordinator() {
        let config = config_for(
            NetworkRole::Client,
            true,
            Some("tcp/192.0.2.1:7447".to_owned()),
            false,
        );
        let endpoints = value(&config, "connect/endpoints");
        assert!(endpoints.contains(LOCAL_COORD_ENDPOINT));
        assert!(!endpoints.contains("192.0.2.1"));
    }

    #[test]
    fn unicast_client_disables_multicast() {
        let config = config_for(
            NetworkRole::Client,
            false,
            Some("tcp/192.0.2.1:7447".to_owned()),
            false,
        );
        assert!(value(&config, "connect/endpoints").contains("tcp/192.0.2.1:7447"));
        assert_eq!(value(&config, "scouting/multicast/enabled"), "false");
    }

    #[test]
    fn enabling_local_only_is_not_overridden() {
        set_local_only(true);
        assert!(is_local_only());
    }

    #[test]
    fn local_coordinator_joins_an_existing_router() {
        let config = config_for(NetworkRole::Coordinator, true, None, true);

        assert_eq!(value(&config, "mode"), r#""client""#);
        assert!(value(&config, "connect/endpoints").contains(LOCAL_COORD_ENDPOINT));
        assert_eq!(value(&config, "scouting/multicast/enabled"), "false");
    }
}
