//! Wire types shared by `marina serve` and the `minot://` registry driver.
//!
//! Both ends are the same binary — one running as a server on the machine that
//! holds the data, one running as a client — so there is no separate protocol
//! crate. What matters is that these types are the *only* thing crossing the
//! link. [`Hello`] reports version mismatches with a useful message.
//!
//! Everything here uses rkyv, matching Minot's transport. The `BagRef` in
//! `crate::model` stays serde-based for config and JSON. [`WireBagRef`] mirrors
//! it on the wire.

use rkyv::{Archive, Deserialize, Serialize};

use crate::model::bag_ref::BagRef;
use crate::registry::driver::{BagInfo, PushMeta};

/// Bumped whenever the meaning of anything below changes.
///
/// A server accepts clients whose major version matches and whose minor version
/// is no newer than its own. [`Hello`] refuses incompatible versions with a
/// message naming both versions, because "connection reset" is a miserable way
/// to learn your marina is out of date.
pub const PROTOCOL_VERSION: (u16, u16) = (2, 3);

/// A [`BagRef`] as it travels.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WireBagRef {
    pub namespace: Option<String>,
    pub name: String,
    pub tags: Vec<String>,
    pub attachment: Option<String>,
}

impl From<&BagRef> for WireBagRef {
    fn from(bag: &BagRef) -> Self {
        Self {
            namespace: bag.namespace.clone(),
            name: bag.name.clone(),
            tags: bag.tags.clone(),
            attachment: bag.attachment.clone(),
        }
    }
}

impl From<WireBagRef> for BagRef {
    fn from(wire: WireBagRef) -> Self {
        Self {
            namespace: wire.namespace,
            name: wire.name,
            tags: wire.tags,
            attachment: wire.attachment,
        }
    }
}

/// A [`BagInfo`] as it travels.
#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct WireBagInfo {
    pub bundle_hash: Option<String>,
    pub original_bytes: u64,
    pub packed_bytes: u64,
    pub pointcloud: Option<String>,
    pub mcap_compression: Option<String>,
    pub pushed_at: Option<u64>,
}

impl From<&BagInfo> for WireBagInfo {
    fn from(info: &BagInfo) -> Self {
        Self {
            bundle_hash: info.bundle_hash.clone(),
            original_bytes: info.original_bytes,
            packed_bytes: info.packed_bytes,
            pointcloud: info.pointcloud.clone(),
            mcap_compression: info.mcap_compression.clone(),
            pushed_at: info.pushed_at,
        }
    }
}

impl From<WireBagInfo> for BagInfo {
    fn from(wire: WireBagInfo) -> Self {
        Self {
            bundle_hash: wire.bundle_hash,
            original_bytes: wire.original_bytes,
            packed_bytes: wire.packed_bytes,
            pointcloud: wire.pointcloud,
            mcap_compression: wire.mcap_compression,
            pushed_at: wire.pushed_at,
        }
    }
}

/// Metadata accompanying an ordinary completed bundle push.
#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct WirePushMeta {
    pub original_bytes: u64,
    pub packed_bytes: u64,
    pub bundle_hash: String,
    pub pointcloud: String,
    pub mcap_compression: String,
    pub pushed_at: u64,
}

impl From<&PushMeta> for WirePushMeta {
    fn from(meta: &PushMeta) -> Self {
        Self {
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            bundle_hash: meta.bundle_hash.clone(),
            pointcloud: meta.pointcloud.clone(),
            mcap_compression: meta.mcap_compression.clone(),
            pushed_at: meta.pushed_at,
        }
    }
}

impl From<WirePushMeta> for PushMeta {
    fn from(meta: WirePushMeta) -> Self {
        Self {
            original_bytes: meta.original_bytes,
            packed_bytes: meta.packed_bytes,
            bundle_hash: meta.bundle_hash,
            pointcloud: meta.pointcloud,
            mcap_compression: meta.mcap_compression,
            pushed_at: meta.pushed_at,
        }
    }
}

/// One file inside a materialised dataset.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WireFile {
    /// Path relative to the dataset root, always with `/` separators.
    pub path: String,
    pub size: u64,
}

/// A completed file as committed by a streaming writer.
#[derive(Archive, Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct WireManifestFile {
    pub path: String,
    pub size: u64,
    pub sha256: String,
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub enum Request {
    /// Sent first. Establishes that both ends speak the same protocol.
    Hello { major: u16, minor: u16 },
    /// Datasets matching a glob-style pattern.
    List { pattern: String },
    /// Metadata for one dataset, without transferring it.
    BagInfo { bag: WireBagRef },
    /// Ask for a bundle. The response names a flow that carries the bytes.
    PullBegin { bag: WireBagRef },
    /// What files a dataset contains, once unpacked.
    ///
    /// The server materialises the dataset if it has not already, so the first
    /// `Stat` of a large dataset can take as long as a pull.
    Stat { bag: WireBagRef },
    /// A byte range from one file in a materialised dataset.
    ///
    /// Ranges travel in the reply. Flows move large resumable objects. A
    /// random-access reader issues *thousands* of small requests, and flow
    /// setup for each would cost far more
    /// than it saves. These are idempotent, so a failed range is simply asked
    /// for again.
    ReadRange {
        bag: WireBagRef,
        path: String,
        offset: u64,
        len: u32,
    },
    /// Check the server and backing registry's write policy without mutating it.
    CheckWrite,
    /// Remove a published dataset from the backing registry.
    Remove { bag: WireBagRef },
    /// Open or resume an ordinary completed-bundle push.
    BeginPush {
        bag: WireBagRef,
        packed_bytes: u64,
        bundle_hash: String,
    },
    /// Append packed bundle bytes at the server's durable watermark.
    PushRange {
        bag: WireBagRef,
        offset: u64,
        data: Vec<u8>,
    },
    /// Atomically publish the completely uploaded packed bundle.
    CommitPush { bag: WireBagRef, meta: WirePushMeta },
    /// Open or resume a local-first streaming upload.
    BeginWrite { bag: WireBagRef },
    /// Idempotently append bytes at the server's current durable watermark.
    WriteRange {
        bag: WireBagRef,
        path: String,
        offset: u64,
        data: Vec<u8>,
    },
    /// Verify all files and atomically publish the completed dataset.
    CommitWrite {
        bag: WireBagRef,
        files: Vec<WireManifestFile>,
    },
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub enum Response {
    Hello {
        major: u16,
        minor: u16,
        /// What the server calls the registry it is exposing, for diagnostics.
        registry: String,
    },
    List(Vec<WireBagRef>),
    BagInfo(Option<WireBagInfo>),
    PullBegin {
        /// Flow the bundle will arrive on. Unique per request, so two concurrent
        /// pulls of the same dataset do not collide.
        flow: String,
        /// Size of the bundle, so the client can verify it received all of it
        /// and verify that the stream completed.
        packed_bytes: u64,
        original_bytes: u64,
    },
    Stat {
        files: Vec<WireFile>,
        /// False when the dataset cannot be read by range — a sqlite3 bag,
        /// which needs a real file on disk. The client falls back to a pull.
        streamable: bool,
        /// Why not, when `streamable` is false. Shown to the user.
        reason: Option<String>,
    },
    /// The first `Stat` found only a packed registry object. Restoration is
    /// running in the background. The client should wait and ask again.
    Materializing {
        message: String,
    },
    ReadRange {
        /// Fewer bytes than asked for means end of file, never an error.
        data: Vec<u8>,
    },
    WriteAllowed,
    Removed,
    PushStatus {
        next_offset: u64,
    },
    PushAck {
        next_offset: u64,
    },
    PushCommitted,
    WriteStatus {
        /// Durable byte watermarks for files already staged on the server.
        files: Vec<WireFile>,
    },
    WriteAck {
        /// Next offset the client should send. Repeating an acknowledged range
        /// is safe and returns the same or a later watermark.
        next_offset: u64,
    },
    WriteCommitted,
}

/// Service topic a server listens on for a given registry name.
///
/// Namespaced by registry so one machine can serve several without collision.
pub fn service_topic(registry: &str) -> String {
    format!("/_marina/{registry}/rpc")
}

/// Check a client's version against this build's.
///
/// Returns the reason for refusal, or `None` if the pairing is fine.
pub fn version_mismatch(client_major: u16, client_minor: u16) -> Option<String> {
    let (major, minor) = PROTOCOL_VERSION;
    if client_major != major {
        return Some(format!(
            "protocol major version mismatch: client speaks {client_major}.{client_minor}, \
             server speaks {major}.{minor}. Both ends need a compatible marina."
        ));
    }
    if client_minor > minor {
        return Some(format!(
            "client is newer than this server: client speaks {client_major}.{client_minor}, \
             server speaks {major}.{minor}. Update the server, or use an older client."
        ));
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::str::FromStr;

    #[test]
    fn a_bag_ref_survives_the_round_trip() {
        let original = BagRef::from_str("stelzo/dlg_cut:ouster:1min").unwrap();
        let wire = WireBagRef::from(&original);
        let returned: BagRef = wire.into();
        assert_eq!(original, returned);
    }

    #[test]
    fn an_attachment_survives_the_round_trip() {
        let original = BagRef::from_str("run:v1[traj.txt]").unwrap();
        let returned: BagRef = WireBagRef::from(&original).into();
        assert_eq!(original, returned);
        assert_eq!(returned.attachment.as_deref(), Some("traj.txt"));
    }

    #[test]
    fn the_same_version_is_accepted() {
        let (major, minor) = PROTOCOL_VERSION;
        assert!(version_mismatch(major, minor).is_none());
    }

    #[test]
    fn an_older_client_is_accepted_but_a_newer_one_is_not() {
        let (major, minor) = PROTOCOL_VERSION;
        assert!(version_mismatch(major, minor.saturating_sub(1)).is_none());
        assert!(
            version_mismatch(major, minor + 1).is_some(),
            "a server must refuse a client that may use features it lacks"
        );
    }

    #[test]
    fn a_major_mismatch_is_refused_in_both_directions() {
        let (major, minor) = PROTOCOL_VERSION;
        assert!(version_mismatch(major + 1, minor).is_some());
        assert!(version_mismatch(major.wrapping_sub(1), minor).is_some());
    }

    #[test]
    fn refusals_name_both_versions() {
        let (major, minor) = PROTOCOL_VERSION;
        let reason = version_mismatch(major + 1, minor).expect("should be refused");
        assert!(
            reason.contains(&format!("{}.{}", major + 1, minor))
                && reason.contains(&format!("{major}.{minor}")),
            "a refusal has to say what both ends speak: {reason}"
        );
    }

    #[test]
    fn service_topics_are_namespaced_per_registry() {
        assert_ne!(service_topic("team"), service_topic("lab"));
    }
}
