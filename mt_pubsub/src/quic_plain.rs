//! QUIC without encryption.
//!
//! quinn has no plaintext mode of its own, so this replaces its TLS session
//! with one that only exchanges the transport parameters and leaves every
//! packet as it is. The handshake follows the `quinn-plaintext` crate
//! (MIT OR Apache-2.0, Jerome Gravel-Niquet).
//!
//! Nothing here is confidential or authenticated: anyone on the network can
//! read and forge samples, as with the plain UDP and TCP links of the Zenoh
//! transport. Integrity against bit errors is left to the UDP checksum.

use std::{any::Any, sync::Arc};

use bytes::BytesMut;
use quinn::{
    ConnectError, ConnectionId, Side,
    crypto::{
        self, CryptoError, ExportKeyingMaterialError, HeaderKey, KeyPair, Keys, PacketKey,
        UnsupportedVersion,
    },
};
use quinn_proto::{TransportError, transport_parameters::TransportParameters};

/// Closes every packet. An encrypted node's packets do not end in it, so a
/// plaintext node drops them instead of reading ciphertext as frames.
const TAG: &[u8; 8] = b"MNQPLAIN";

pub(crate) fn server_config() -> quinn::ServerConfig {
    quinn::ServerConfig::with_crypto(Arc::new(Plain))
}

pub(crate) fn client_config() -> quinn::ClientConfig {
    quinn::ClientConfig::new(Arc::new(Plain))
}

struct Plain;

impl HeaderKey for Plain {
    fn decrypt(&self, _: usize, _: &mut [u8]) {}
    fn encrypt(&self, _: usize, _: &mut [u8]) {}
    fn sample_size(&self) -> usize {
        0
    }
}

impl PacketKey for Plain {
    fn encrypt(&self, _: u64, buf: &mut [u8], _: usize) {
        let tag = buf.len() - TAG.len();
        buf[tag..].copy_from_slice(TAG);
    }

    fn decrypt(&self, _: u64, _: &[u8], payload: &mut BytesMut) -> Result<(), CryptoError> {
        let len = payload.len().checked_sub(TAG.len()).ok_or(CryptoError)?;
        if &payload[len..] != TAG {
            return Err(CryptoError);
        }
        payload.truncate(len);
        Ok(())
    }

    fn tag_len(&self) -> usize {
        TAG.len()
    }

    fn confidentiality_limit(&self) -> u64 {
        u64::MAX
    }

    fn integrity_limit(&self) -> u64 {
        u64::MAX
    }
}

fn packet_keys() -> KeyPair<Box<dyn PacketKey>> {
    KeyPair {
        local: Box::new(Plain),
        remote: Box::new(Plain),
    }
}

fn keys() -> Keys {
    Keys {
        header: KeyPair {
            local: Box::new(Plain),
            remote: Box::new(Plain),
        },
        packet: packet_keys(),
    }
}

/// The handshake: each side sends its transport parameters once, and hands
/// out the (empty) handshake and data keys in the order quinn asks for them.
struct Session {
    side: Side,
    params: TransportParameters,
    peer_params: Option<TransportParameters>,
    wrote_params: bool,
    /// Key sets still to hand out: handshake, then data.
    pending_keys: u8,
}

impl Session {
    fn start(side: Side, params: &TransportParameters) -> Box<dyn crypto::Session> {
        Box::new(Self {
            side,
            params: *params,
            peer_params: None,
            wrote_params: false,
            pending_keys: 2,
        })
    }
}

impl crypto::Session for Session {
    fn initial_keys(&self, _: &ConnectionId, _: Side) -> Keys {
        keys()
    }

    fn handshake_data(&self) -> Option<Box<dyn Any>> {
        self.peer_params
            .map(|params| Box::new(params) as Box<dyn Any>)
    }

    fn peer_identity(&self) -> Option<Box<dyn Any>> {
        None
    }

    fn early_crypto(&self) -> Option<(Box<dyn HeaderKey>, Box<dyn PacketKey>)> {
        None
    }

    fn early_data_accepted(&self) -> Option<bool> {
        Some(false)
    }

    fn is_handshaking(&self) -> bool {
        self.peer_params.is_none() || !self.wrote_params && self.pending_keys > 0
    }

    fn read_handshake(&mut self, mut buf: &[u8]) -> Result<bool, TransportError> {
        if self.peer_params.is_none() {
            self.peer_params = Some(TransportParameters::read(self.side, &mut buf)?);
        }
        Ok(true)
    }

    fn transport_parameters(&self) -> Result<Option<TransportParameters>, TransportError> {
        Ok(self.peer_params)
    }

    fn write_handshake(&mut self, buf: &mut Vec<u8>) -> Option<Keys> {
        // The client opens with its parameters; the server answers with its
        // own once it hands out the data keys.
        let due = self.side.is_client() || self.pending_keys == 1;
        if due && !self.wrote_params {
            self.params.write(buf);
            self.wrote_params = true;
        }
        if self.pending_keys == 0 {
            return None;
        }
        self.pending_keys -= 1;
        Some(keys())
    }

    fn next_1rtt_keys(&mut self) -> Option<KeyPair<Box<dyn PacketKey>>> {
        Some(packet_keys())
    }

    fn is_valid_retry(&self, _: &ConnectionId, _: &[u8], _: &[u8]) -> bool {
        true
    }

    fn export_keying_material(
        &self,
        _: &mut [u8],
        _: &[u8],
        _: &[u8],
    ) -> Result<(), ExportKeyingMaterialError> {
        Err(ExportKeyingMaterialError)
    }
}

impl crypto::ClientConfig for Plain {
    fn start_session(
        self: Arc<Self>,
        _: u32,
        _: &str,
        params: &TransportParameters,
    ) -> Result<Box<dyn crypto::Session>, ConnectError> {
        Ok(Session::start(Side::Client, params))
    }
}

impl crypto::ServerConfig for Plain {
    fn initial_keys(&self, _: u32, _: &ConnectionId) -> Result<Keys, UnsupportedVersion> {
        Ok(keys())
    }

    fn retry_tag(&self, _: u32, _: &ConnectionId, _: &[u8]) -> [u8; 16] {
        [0; 16]
    }

    fn start_session(
        self: Arc<Self>,
        _: u32,
        params: &TransportParameters,
    ) -> Box<dyn crypto::Session> {
        Session::start(Side::Server, params)
    }
}
