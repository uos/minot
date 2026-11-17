//! Payload transfer counters of the QUIC transport. They are the ones Zenoh
//! feeds as well, see [`mt_sea::client::shm_transfer_stats`], so the transfer
//! report reads the same for both. Without the `shm` feature none are kept.

#[cfg(feature = "shm")]
pub(crate) use mt_sea::client::transfer::*;

#[cfg(not(feature = "shm"))]
mod off {
    pub fn shm_send_attempt() {}
    pub fn shm_send_fallback() {}
    pub fn network_sent(_: usize) {}
    pub fn network_received(_: usize) {}
}

#[cfg(not(feature = "shm"))]
pub(crate) use off::*;

/// Counts a sample Zenoh delivered, through shared memory or the network.
pub(crate) fn zenoh_received(payload: &zenoh::bytes::ZBytes) {
    #[cfg(feature = "shm")]
    if payload.as_shm().is_some() {
        shm_received(payload.len());
    } else {
        network_received(payload.len());
    }
    #[cfg(not(feature = "shm"))]
    let _ = payload;
}
