//! A Minot network of one's own, for running several full stacks on one
//! machine.
//!
//! Two stacks on the default port and domain find each other, through the
//! coordinator on `7447` or through multicast scouting, and their topics mix.
//! A [`NetworkClaim`] picks a port and domain no other claim on this machine
//! holds. The first stack gets the defaults, so tools left on them still see
//! it; later ones get a free port, with the same number as their domain.
//!
//! Whoever starts a tree of processes claims once and hands the result down
//! as `MINOT_ZENOH_PORT` and `MINOT_DOMAIN_ID` ([`NetworkClaim::env`]). A
//! process that finds either variable already set takes it as chosen by its
//! parent, or by the user, and claims nothing.

use std::fs::File;
use std::io;
use std::net::TcpListener;
use std::path::PathBuf;

use log::debug;

use crate::network::{DEFAULT_ZENOH_PORT, coordinator_port_from_env};

/// How many free ports to try before giving up.
const ATTEMPTS: usize = 64;

/// The port and domain one Minot stack uses, and for a stack that picked its
/// own, the lock that keeps other claims off them until it is dropped.
#[derive(Debug)]
pub struct NetworkClaim {
    port: u16,
    domain_id: u16,
    chosen: bool,
    _lock: Option<File>,
}

impl NetworkClaim {
    /// TCP port of the coordinator, and UDP port of peer scouting.
    pub fn port(&self) -> u16 {
        self.port
    }

    pub fn domain_id(&self) -> u16 {
        self.domain_id
    }

    /// The port and domain were already set in the environment, by the
    /// user or by a parent process, rather than picked here.
    pub fn was_given(&self) -> bool {
        !self.chosen
    }

    /// Whether this is the default network, the one tools use unless told
    /// otherwise.
    pub fn is_default(&self) -> bool {
        self.port == DEFAULT_ZENOH_PORT && self.domain_id == crate::net::get_domain_id()
    }

    /// The environment that puts a child process on this network.
    pub fn env(&self) -> [(&'static str, String); 2] {
        [
            ("MINOT_ZENOH_PORT", self.port.to_string()),
            ("MINOT_DOMAIN_ID", self.domain_id.to_string()),
        ]
    }

    /// Put this process on the network too: its own sessions read the same
    /// variables its children do.
    ///
    /// # Safety
    ///
    /// Changes the process environment, so no other thread may be reading or
    /// writing it. Call it at the start of `main`, before spawning threads.
    pub unsafe fn apply_to_process(&self) {
        for (key, value) in self.env() {
            // SAFETY: the caller guarantees no other thread touches the environment.
            unsafe { std::env::set_var(key, value) };
        }
    }

    /// A line for the user: where this network is, and how to reach it from
    /// another tool when it is not the default one.
    pub fn describe(&self) -> String {
        if self.was_given() {
            format!(
                "Minot network: port {}, domain {} (set in the environment)",
                self.port, self.domain_id
            )
        } else if self.is_default() {
            format!("Minot network: default port {}, domain {}", self.port, self.domain_id)
        } else {
            format!(
                "Minot network: port {} and domain {} are this run's own, the defaults being in use. \
                 To reach it from a tool: MINOT_ZENOH_PORT={} MINOT_DOMAIN_ID={}",
                self.port, self.domain_id, self.port, self.domain_id
            )
        }
    }
}

/// Claim a network for this stack, from the environment as it is.
///
/// If `MINOT_ZENOH_PORT` is set, it is kept, with `MINOT_DOMAIN_ID` as set or
/// the default, and nothing is locked: someone else has chosen. Otherwise the
/// default port is taken when no other claim holds it and nothing listens on
/// it, and a free one when not.
pub fn claim_network() -> io::Result<NetworkClaim> {
    let given_port = coordinator_port_from_env();
    let given_domain = std::env::var("MINOT_DOMAIN_ID")
        .ok()
        .and_then(|value| value.parse::<u16>().ok());
    claim_from(DEFAULT_ZENOH_PORT, given_port, given_domain)
}

pub(crate) fn claim_from(
    preferred: u16,
    given_port: Option<u16>,
    given_domain: Option<u16>,
) -> io::Result<NetworkClaim> {
    if let Some(port) = given_port {
        return Ok(NetworkClaim {
            port,
            domain_id: given_domain.unwrap_or(0),
            chosen: false,
            _lock: None,
        });
    }
    let mut port = preferred;
    for _ in 0..ATTEMPTS {
        if let Some(lock) = try_claim(port) {
            return Ok(NetworkClaim {
                port,
                domain_id: given_domain.unwrap_or(if port == preferred { 0 } else { port }),
                chosen: true,
                _lock: Some(lock),
            });
        }
        debug!("Minot port {port} is taken, looking for a free one");
        port = unused_port()?;
    }
    Err(io::Error::new(
        io::ErrorKind::AddrInUse,
        "no free port for a Minot network",
    ))
}

/// Lock `port` for this process if nothing else holds it.
fn try_claim(port: u16) -> Option<File> {
    let lock = File::create(lock_path(port)).ok()?;
    fs2::FileExt::try_lock_exclusive(&lock).ok()?;
    // Checked after the lock, so two claims racing for it cannot both pass:
    // this also keeps off a coordinator or router that is not a claim.
    port_is_free(port).then_some(lock)
}

fn port_is_free(port: u16) -> bool {
    TcpListener::bind(("0.0.0.0", port)).is_ok() && TcpListener::bind(("127.0.0.1", port)).is_ok()
}

/// A port the system has just handed out. Another process may take it before
/// the caller does, which is why it is locked afterwards.
fn unused_port() -> io::Result<u16> {
    Ok(TcpListener::bind(("127.0.0.1", 0))?.local_addr()?.port())
}

fn lock_path(port: u16) -> PathBuf {
    // SAFETY: `getuid` takes no arguments and has no preconditions.
    let user = unsafe { libc::getuid() };
    std::env::temp_dir().join(format!("minot-net_{user}_{port}.claim"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn free_port() -> u16 {
        unused_port().unwrap()
    }

    #[test]
    fn the_first_claim_gets_the_preferred_port_and_the_next_one_its_own() {
        let preferred = free_port();
        let first = claim_from(preferred, None, None).unwrap();
        assert_eq!((first.port(), first.domain_id()), (preferred, 0));
        let second = claim_from(preferred, None, None).unwrap();
        assert_ne!(second.port(), preferred);
        assert_eq!(second.domain_id(), second.port(), "its own domain too");
        let third = claim_from(preferred, None, None).unwrap();
        assert!(![preferred, second.port()].contains(&third.port()));

        // A released port can be claimed again.
        drop(first);
        let again = claim_from(preferred, None, None).unwrap();
        assert_eq!(again.port(), preferred);
    }

    #[test]
    fn a_port_something_listens_on_is_not_claimed() {
        let listener = TcpListener::bind(("127.0.0.1", 0)).unwrap();
        let busy = listener.local_addr().unwrap().port();
        let claim = claim_from(busy, None, None).unwrap();
        assert_ne!(claim.port(), busy);
    }

    #[test]
    fn a_port_given_by_the_environment_is_kept_and_not_locked() {
        let port = free_port();
        let a = claim_from(DEFAULT_ZENOH_PORT, Some(port), Some(9)).unwrap();
        let b = claim_from(DEFAULT_ZENOH_PORT, Some(port), None).unwrap();
        assert_eq!((a.port(), a.domain_id(), a.was_given()), (port, 9, true));
        assert_eq!((b.port(), b.domain_id()), (port, 0));
    }

    #[test]
    fn a_domain_given_without_a_port_survives_the_choice_of_port() {
        let preferred = free_port();
        let _first = claim_from(preferred, None, Some(5)).unwrap();
        let second = claim_from(preferred, None, Some(5)).unwrap();
        assert_ne!(second.port(), preferred);
        assert_eq!(second.domain_id(), 5);
    }

    #[test]
    fn the_environment_names_the_network_for_children() {
        let claim = claim_from(free_port(), None, None).unwrap();
        let env = claim.env();
        assert_eq!(env[0], ("MINOT_ZENOH_PORT", claim.port().to_string()));
        assert_eq!(env[1], ("MINOT_DOMAIN_ID", "0".to_owned()));
    }
}
