use anyhow::{Result, anyhow};
use core::net::SocketAddr;
use log::{debug, info};
use mt_sea::ShipKind;
use mt_sea::{net::Packet, ship::NetworkShipImpl, *};
use once_cell::sync::OnceCell;
use std::collections::HashSet;
use std::sync::Arc;
use std::thread;
use std::time;

use tokio::signal::ctrl_c;
use tokio::sync::Mutex;

#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum Qos {
    #[default]
    Reliable,
    /// Reliable wire delivery and ordering, without the fatal failure policy.
    /// See `mt_sea::Qos` for the full split.
    TryReliable,
    BestEffort,
}

#[derive(Debug, Clone)]
pub struct ScopeConfig {
    pub name: String,
    pub mode: Qos,
}

// Singleton Objects to make sure this exists only once in the system
static SCOPE: OnceCell<Arc<Mutex<Scope>>> = OnceCell::new();
static COORDCOMMUNICATION: OnceCell<Arc<Mutex<CoordCommunication>>> = OnceCell::new();
static PACKET_ID: OnceCell<Arc<Mutex<i32>>> = OnceCell::new();
static CLIENTS: OnceCell<Arc<Mutex<HashSet<String>>>> = OnceCell::new();
static BEST_EFFORT_CLIENTS: OnceCell<Arc<Mutex<HashSet<String>>>> = OnceCell::new();

/// How long to wait for the coordinator's answer to a Sonar before giving up on
/// the round.
///
/// Generous on purpose. A timeout here skips one comparison, which costs
/// nothing, while concluding too early that the coordinator said nothing fires
/// a Torpedo at a healthy run.
const SONAR_TIMEOUT: time::Duration = time::Duration::from_secs(5);

/// How long to wait for the coordinator to acknowledge the scope's own
/// registration.
///
/// A scope that never registered monitors nothing for the rest of the run and
/// has no next round to fall back on, so this fails loudly.
const REGISTRATION_TIMEOUT: time::Duration = time::Duration::from_secs(30);

/// Which of the lost clients warrant a Torpedo.
///
/// Kept apart from the monitor so the rule can be tested on its own: a node
/// known to be non-fatal never contributes, whatever else is going on. The
/// caller must already hold the non-fatal set, since a set it could not read
/// would look like an empty one.
fn fatal_losses(all_lost: &[String], non_fatal: &HashSet<String>) -> Vec<String> {
    all_lost
        .iter()
        .filter(|name| !non_fatal.contains(*name))
        .cloned()
        .collect()
}

#[derive(Debug, Clone)]
pub struct Scope {
    name: String,
    ship: Arc<NetworkShipImpl>,
}

// Save communication channel to the coordinator
struct CoordCommunication {
    coord: (
        tokio::sync::mpsc::Sender<Packet>,
        tokio::sync::broadcast::Receiver<(Packet, Option<SocketAddr>)>,
    ),
}

impl Scope {
    /// Start the Scope. Only once per process due to static OnceCell.
    pub async fn create(config: ScopeConfig) -> anyhow::Result<()> {
        let sea_node_mode = match config.mode {
            Qos::Reliable => mt_sea::Qos::Reliable,
            Qos::TryReliable => mt_sea::Qos::TryReliable,
            Qos::BestEffort => mt_sea::Qos::BestEffort,
        };
        let rm_rules_on_disconnect = sea_node_mode.removes_rules_on_exit();
        let ship = mt_sea::ship::NetworkShipImpl::init(
            ShipKind::Rat(config.name.clone()),
            rm_rules_on_disconnect,
            sea_node_mode,
            mt_sea::NodeOptions::default(),
        )
        .await?;
        debug!("Ship created");

        let ship = Arc::new(ship);
        // A Scope can stay quiet for a long time between samples, and the
        // coordinator drops a client that says nothing.
        ship.spawn_heartbeat();

        let scope = Scope {
            name: config.name,
            ship,
        };

        SCOPE
            .set(Arc::new(Mutex::new(scope)))
            .map_err(|_| anyhow!("This scope already exists"))?;

        PACKET_ID
            .set(Arc::new(Mutex::new(0)))
            .map_err(|_| anyhow!("Packet ID already initialized"))?;

        CLIENTS
            .set(Arc::new(Mutex::new(HashSet::new())))
            .map_err(|_| anyhow!("Clients already initialized"))?;

        BEST_EFFORT_CLIENTS
            .set(Arc::new(Mutex::new(HashSet::new())))
            .map_err(|_| anyhow!("Best-effort clients already initialized"))?;

        Scope::init_connection().await?;
        debug!("Coordinator connection established");

        Scope::connect_to_coord().await?;
        debug!("Registered with coordinator");

        Scope::start_scoping().await?;
        debug!("Scoping complete");

        Ok(())
    }

    async fn init_connection() -> anyhow::Result<()> {
        let scope = Scope::get_scope().await?;
        let scope = scope.lock().await;
        let ship = &scope.ship;

        let (coord_tx, coord_rx) = {
            let client = ship.client.lock().await;
            let client_send_lock = client.coordinator_send.read().unwrap();
            let coord_tx = client_send_lock
                .as_ref()
                .expect("Sender does not exist after creation.")
                .clone();

            let client_recv_lock = client.coordinator_receive.read().unwrap();
            let coord_rx = client_recv_lock
                .as_ref()
                .expect("Receiver does not exist after creation")
                .subscribe();
            (coord_tx, coord_rx)
        };

        let coords = CoordCommunication {
            coord: (coord_tx, coord_rx),
        };

        COORDCOMMUNICATION
            .set(Arc::new(Mutex::new(coords)))
            .map_err(|_| anyhow::anyhow!("COORDCOMMUNICATION already initialized"))?;

        Ok(())
    }

    async fn connect_to_coord() -> anyhow::Result<()> {
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        // Resubscribed before the request goes out, so the answer cannot land
        // in the gap between sending and listening. That is all the ordering
        // this needs, so the answer is awaited inline and nothing outlives the
        // call when it gives up.
        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());

        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::RegisterShipAtVar {
                    ship: Scope::get_scope_name().await?,
                    var: Scope::get_scope_name().await?,
                    kind: net::RatPubRegisterKind::Scope,
                    node_mode: net::Qos::Reliable,
                },
            })
            .await?;
        drop(channels);

        let acknowledged = tokio::time::timeout(REGISTRATION_TIMEOUT, async {
            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => {
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            return true;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return false,
                }
            }
        })
        .await;

        // Fatal, where an unanswered Sonar is not: a scope that never
        // registered monitors nothing, and silence about that helps no one.
        match acknowledged {
            Ok(true) => Ok(()),
            Ok(false) => Err(anyhow!(
                "Coordinator channel closed before the scope was registered"
            )),
            Err(_) => Err(anyhow!(
                "Coordinator did not acknowledge the scope's registration within {REGISTRATION_TIMEOUT:?}"
            )),
        }
    }

    async fn start_scoping() -> anyhow::Result<()> {
        loop {
            tokio::select! {
                _ = ctrl_c() => {
                    debug!("Ctrl-C received. Stopping scoping.");
                    return Ok(())
                }
                res = Scope::threesixty_scoping() => {
                    res?
                }
            }
            let interval = time::Duration::from_millis(1000);
            thread::sleep(interval);
        }
    }

    async fn threesixty_scoping() -> anyhow::Result<()> {
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        // Resubscribed before the Sonar goes out, so the answer cannot land in
        // the gap between sending and listening.
        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());

        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::Sonar,
            })
            .await
            .map_err(|_e| anyhow!("Failed to send Sonar packet"))?;
        // Nothing below needs the channels, and the answer may take seconds.
        drop(channels);

        // Awaited inline: a spawned task outlives the round that gave up on
        // it, holding its subscription and its half of the oneshot until some
        // *later* round's answer wakes it, so every timeout leaves one behind.
        let answer = tokio::time::timeout(SONAR_TIMEOUT, async {
            loop {
                match coord_rx.recv().await {
                    // Only the packet that answers a Sonar ends this wait.
                    // Registration traffic puts an `Acknowledge` on this
                    // channel at any moment, and one carries no client list, so
                    // ending here on it leaves the set empty and
                    // `handle_packet` reads that as every node having died.
                    Ok((packet, _)) => {
                        if let net::PacketKind::ClientsHash {
                            mut reliable,
                            best_effort,
                        } = packet.data
                        {
                            // Exclude the scope itself from the set it monitors
                            if let Ok(name) = Scope::get_scope_name().await {
                                reliable.remove(&name);
                            }
                            debug!(
                                "[SCOPE] Sonar response — reliable: {:?}, best_effort: {:?}",
                                reliable, best_effort
                            );
                            // Legacy field name: it carries every nonfatal
                            // node, BestEffort and TryReliable alike. Names are
                            // only ever added, so a node stays recognisable
                            // after the coordinator drops it from its live set.
                            if let Some(be_set) = BEST_EFFORT_CLIENTS.get() {
                                be_set.lock().await.extend(best_effort);
                            }
                            return Some(reliable);
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return None,
                }
            }
        })
        .await;

        let clients_current = match answer {
            Ok(Some(reliable)) => reliable,
            Ok(None) => {
                debug!("[SCOPE] Coordinator channel closed, skipping this round");
                return Ok(());
            }
            // No answer means this scope does not know who is connected.
            // Skipping the round costs one second, where acting on an
            // unanswered Sonar costs the whole run.
            Err(_) => {
                debug!("[SCOPE] No Sonar answer within {SONAR_TIMEOUT:?}, skipping this round");
                return Ok(());
            }
        };

        Scope::handle_packet(clients_current).await
    }

    async fn handle_packet(clients_current: HashSet<String>) -> anyhow::Result<()> {
        debug!(
            "[SCOPE] Handling packet #{:?}",
            Scope::get_counter_value().await
        );
        let clients = Scope::get_clients().await?;
        let mut clients = clients.lock().await;

        if clients.is_subset(&clients_current) {
            let new_clients: Vec<_> = clients_current.difference(&clients).cloned().collect();
            if new_clients.is_empty() {
                return Ok(());
            }
            info!("[SCOPE] New clients: {:?}", new_clients);
            for client in new_clients {
                clients.insert(client);
            }
        } else {
            let all_lost: Vec<_> = clients.difference(&clients_current).cloned().collect();

            // Filter out nonfatal nodes, since losing BestEffort or
            // TryReliable clients is routine and must not fire a Torpedo.
            //
            // The set is awaited, not sampled: a `try_lock` losing a race would
            // leave every lost node looking fatal, so a moment of contention
            // could Torpedo a run over a departed best-effort viewer.
            let Some(be_set) = BEST_EFFORT_CLIENTS.get() else {
                debug!(
                    "[SCOPE] Non-fatal client set unavailable, skipping Torpedo for: {:?}",
                    all_lost
                );
                return Ok(());
            };
            let lost_reliable = {
                let be = be_set.lock().await;
                fatal_losses(&all_lost, &be)
            };

            for client in &all_lost {
                clients.remove(client);
            }

            if !lost_reliable.is_empty() {
                info!("[SCOPE] Lost clients, firing Torpedo: {:?}", lost_reliable);
                Scope::send_torpedo(lost_reliable).await?;
            } else {
                debug!(
                    "[SCOPE] Lost best-effort clients, skipping Torpedo: {:?}",
                    all_lost
                );
            }
        }
        Ok(())
    }

    async fn send_torpedo(dead_clients: Vec<String>) -> anyhow::Result<()> {
        debug!("[SCOPE] Sending Torpedo for: {:?}", dead_clients);
        let channels = Scope::get_coord_communication().await?;
        let channels = channels.lock().await;

        let (coord_tx, mut coord_rx) = (channels.coord.0.clone(), channels.coord.1.resubscribe());
        let (result_tx, result_rx) = tokio::sync::oneshot::channel();
        let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();

        tokio::spawn(async move {
            let _ = ready_tx.send(());

            loop {
                match coord_rx.recv().await {
                    Ok((packet, _)) => {
                        if let net::PacketKind::Acknowledge = packet.data {
                            let _ = result_tx.send(());
                            return;
                        }
                    }
                    Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(_) => return,
                }
            }
        });

        ready_rx
            .await
            .map_err(|_| anyhow!("Receiver task failed to start"))?;

        info!("[SCOPE] Sending Torpedo packet");
        coord_tx
            .send(Packet {
                header: mt_sea::net::Header::default(),
                data: net::PacketKind::Torpedo(dead_clients),
            })
            .await
            .map_err(|_e| anyhow!("Failed to send Torpedo packet"))?;

        result_rx
            .await
            .map_err(|_e| anyhow!("Failed to receive Torpedo acknowledgement"))?;

        drop(channels);
        Ok(())
    }

    async fn get_scope() -> Result<Arc<Mutex<Scope>>> {
        SCOPE
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No scope initialized"))
    }

    async fn get_coord_communication() -> Result<Arc<Mutex<CoordCommunication>>> {
        COORDCOMMUNICATION
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No coordinator communication channel initialized"))
    }

    async fn get_clients() -> Result<Arc<Mutex<HashSet<String>>>> {
        CLIENTS
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("No clients set initialized"))
    }

    async fn get_scope_name() -> Result<String> {
        let name = Scope::get_scope().await?.lock().await.name.clone();
        Ok(name)
    }

    async fn get_packet_id() -> Result<Arc<Mutex<i32>>> {
        PACKET_ID
            .get()
            .cloned()
            .ok_or_else(|| anyhow!("Packet ID not initialized"))
    }

    async fn get_counter_value() -> Result<i32> {
        let value = Scope::get_packet_id().await?;
        let mut value = value.lock().await;
        let current = *value;
        *value += 1;
        Ok(current)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(names: &[&str]) -> HashSet<String> {
        names.iter().map(|n| n.to_string()).collect()
    }

    fn lost(names: &[&str]) -> Vec<String> {
        names.iter().map(|n| n.to_string()).collect()
    }

    /// The regression: a best-effort viewer dropping off is routine and must
    /// never take the run down with it.
    #[test]
    fn losing_a_non_fatal_client_fires_nothing() {
        let non_fatal = set(&["scope"]);
        assert!(fatal_losses(&lost(&["scope"]), &non_fatal).is_empty());
    }

    /// The publishers named in the Torpedo that started this: losing the viewer
    /// must not implicate them.
    #[test]
    fn a_non_fatal_loss_does_not_implicate_the_publishers() {
        let non_fatal = set(&["scope"]);
        assert!(
            fatal_losses(&lost(&["scope"]), &non_fatal).is_empty(),
            "only the client that actually went away may ever be considered"
        );
    }

    #[test]
    fn losing_a_fatal_client_is_reported() {
        let non_fatal = set(&["scope"]);
        assert_eq!(
            fatal_losses(&lost(&["pelorus_node"]), &non_fatal),
            vec!["pelorus_node".to_string()]
        );
    }

    #[test]
    fn a_mixed_loss_reports_only_the_fatal_ones() {
        let non_fatal = set(&["scope", "viewer"]);
        assert_eq!(
            fatal_losses(&lost(&["scope", "pelorus_node", "viewer"]), &non_fatal),
            vec!["pelorus_node".to_string()]
        );
    }

    /// An unanswered Sonar leaves an empty non-fatal set behind, which means
    /// nothing is known to be non-fatal. The guard above returns before the
    /// caller can reach here; this pins down why that guard exists, since the
    /// bare rule would implicate everything.
    #[test]
    fn an_empty_non_fatal_set_would_implicate_everything() {
        let nothing_known = HashSet::new();
        assert_eq!(
            fatal_losses(&lost(&["scope", "pelorus_node"]), &nothing_known).len(),
            2,
            "which is why an unreadable set must skip the round instead of asking"
        );
    }
}
