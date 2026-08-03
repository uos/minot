use std::net::{SocketAddr, TcpStream};
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration;

use log::{info, warn};

pub const LOCAL_COORD_ENDPOINT: &str = "tcp/127.0.0.1:7447";
pub const NETWORK_COORD_ENDPOINT: &str = "tcp/0.0.0.0:7447";

static LOCAL_ONLY: AtomicBool = AtomicBool::new(false);

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
