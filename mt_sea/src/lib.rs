pub mod client;
pub mod coordinator;
pub mod net;
pub mod network;
pub mod ship;

pub use net::Qos;

// ---------------------------------------------------------------------------
// Timing constants — these values are interdependent.
// When changing one, check all others marked with the same group tag.
// ---------------------------------------------------------------------------

/// [heartbeat] How often nodes send heartbeat messages to keep their connection alive.
pub const HEARTBEAT_INTERVAL_MS: u64 = 400;

/// [heartbeat] Heartbeat is skipped if data was sent within this window.
/// Must be ≤ HEARTBEAT_INTERVAL_MS so a heartbeat is always sent before
/// DISCONNECT_TIMEOUT_MS elapses after the last real data.
pub const HEARTBEAT_SUPPRESS_MS: u64 = HEARTBEAT_INTERVAL_MS / 2;

/// [heartbeat] Coordinator drops a client after this long with no message.
/// Must be > HEARTBEAT_INTERVAL_MS + HEARTBEAT_SUPPRESS_MS to avoid false disconnects.
pub const DISCONNECT_TIMEOUT_MS: u64 = HEARTBEAT_INTERVAL_MS * 2; // 800 ms

/// [registration] How long to wait for a coordinator before giving up on registration.
pub const REGISTRATION_TIMEOUT_MS: u64 = 2000;

/// [peer-heartbeat] Consecutive ping failures before declaring a peer dead.
pub const PEER_DEAD_THRESHOLD: u32 = 3;

/// Policy shared by every node.
///
/// Carried as one value so adding a knob later does not churn every call site.
/// `Default` reproduces the behaviour Minot had before these existed: LAN
/// timing, and reconnect decided by the node's QoS.
#[derive(Debug, Clone, Copy, Default)]
pub struct NodeOptions {
    /// How patient this node and its coordinator are with each other.
    pub timing: Timing,
    /// What to do when the coordinator link drops. `None` derives it from the
    /// node's QoS via [`ReconnectPolicy::for_qos`], which is almost always what
    /// you want — override only to force a `Reliable` node to give up sooner,
    /// or to pin a resilient node to `Never` for a deterministic test.
    pub reconnect: Option<ReconnectPolicy>,
}

impl NodeOptions {
    /// Options for a link that is expected to wobble: WAN timing plus
    /// reconnection.
    pub fn wan() -> Self {
        Self {
            timing: Timing::wan(),
            reconnect: Some(ReconnectPolicy::always()),
        }
    }

    pub fn with_timing(mut self, timing: Timing) -> Self {
        self.timing = timing;
        self
    }

    pub fn with_reconnect(mut self, reconnect: ReconnectPolicy) -> Self {
        self.reconnect = Some(reconnect);
        self
    }

    /// Resolve the reconnect policy for a node running at `qos`.
    pub fn reconnect_policy(&self, qos: Qos) -> ReconnectPolicy {
        self.reconnect
            .unwrap_or_else(|| ReconnectPolicy::for_qos(qos))
    }
}

/// Live state of a node's link to the coordinator.
///
/// A node's connection can come back. The [`ShipKind`] registration, the
/// channels to the coordinator, and the coordinator's own handler for this
/// client are all rebuilt on a reconnect, but the *node* — and every publisher
/// and subscriber the caller is holding — stays alive across it.
///
/// Each successful registration is a **generation**. Long-lived tasks watch the
/// generation. When it changes, the
/// channels they captured are stale and whatever registration they did with the
/// coordinator has to be redone.
#[derive(Debug)]
pub struct ConnectionState {
    generation: std::sync::atomic::AtomicU64,
    connected: std::sync::atomic::AtomicBool,
    link_proven: std::sync::atomic::AtomicBool,
    heartbeating: std::sync::atomic::AtomicBool,
    /// Carries the new generation number to everything that needs to re-register.
    announce: tokio::sync::broadcast::Sender<u64>,
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self::new()
    }
}

impl ConnectionState {
    pub fn new() -> Self {
        let (announce, _) = tokio::sync::broadcast::channel(16);
        Self {
            generation: std::sync::atomic::AtomicU64::new(0),
            connected: std::sync::atomic::AtomicBool::new(true),
            link_proven: std::sync::atomic::AtomicBool::new(false),
            heartbeating: std::sync::atomic::AtomicBool::new(false),
            announce,
        }
    }

    /// Which registration is currently live. Starts at 0 and increments once
    /// per successful re-registration.
    pub fn generation(&self) -> u64 {
        self.generation.load(std::sync::atomic::Ordering::Acquire)
    }

    /// Whether the link is up right now.
    pub fn is_connected(&self) -> bool {
        self.connected.load(std::sync::atomic::Ordering::Acquire)
    }

    /// Subscribe to reconnection announcements. Each message is the generation
    /// that just became live.
    pub fn subscribe(&self) -> tokio::sync::broadcast::Receiver<u64> {
        self.announce.subscribe()
    }

    /// Whether the coordinator has been observed answering this node.
    ///
    /// Registration only proves the coordinator was reachable at that instant.
    /// This becomes true once it echoes a heartbeat, which is also the point at
    /// which the node's own disconnect detector arms — before then, a
    /// coordinator that dies cannot be noticed. Wait on this when you need the
    /// link to be established and ready for traffic.
    pub fn is_link_proven(&self) -> bool {
        self.link_proven.load(std::sync::atomic::Ordering::Acquire)
    }

    /// Whether this node sends heartbeats.
    ///
    /// Only a heartbeating node can expect the coordinator to answer, so only a
    /// heartbeating node may treat silence as a disconnect. A ship that speaks
    /// solely when it has something to say would otherwise be torn down for
    /// being quiet.
    pub fn is_heartbeating(&self) -> bool {
        self.heartbeating.load(std::sync::atomic::Ordering::Acquire)
    }

    pub(crate) fn mark_heartbeating(&self) {
        self.heartbeating
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn mark_link_proven(&self) {
        self.link_proven
            .store(true, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn mark_link_unproven(&self) {
        self.link_proven
            .store(false, std::sync::atomic::Ordering::Release);
    }

    pub(crate) fn mark_disconnected(&self) {
        self.mark_link_unproven();
        self.connected
            .store(false, std::sync::atomic::Ordering::Release);
    }

    /// Publish a completed re-registration. Returns the new generation.
    pub(crate) fn mark_reconnected(&self) -> u64 {
        let generation = self
            .generation
            .fetch_add(1, std::sync::atomic::Ordering::AcqRel)
            + 1;
        self.connected
            .store(true, std::sync::atomic::Ordering::Release);
        // No receivers is normal — nothing is currently holding a subscription.
        let _ = self.announce.send(generation);
        generation
    }
}

/// What a node should do when it loses the coordinator.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum ReconnectPolicy {
    /// Give up. The node's shutdown token is cancelled and it is finished.
    ///
    /// This is the only correct policy for [`Qos::Reliable`], whose whole
    /// contract is that its death is fatal to the run.
    #[default]
    Never,
    /// Keep re-registering, with backoff, until the link comes back or the node
    /// is shut down explicitly. Publishers and subscribers survive.
    Always {
        /// First retry delay.
        initial_backoff_ms: u64,
        /// Ceiling for the exponential backoff.
        max_backoff_ms: u64,
    },
}

impl ReconnectPolicy {
    /// The sensible default for a node that is allowed to come back.
    pub fn always() -> Self {
        Self::Always {
            initial_backoff_ms: 250,
            max_backoff_ms: 10_000,
        }
    }

    pub fn is_enabled(self) -> bool {
        matches!(self, Self::Always { .. })
    }

    /// The default policy for a node with this QoS.
    ///
    /// `Reliable` never reconnects: peers monitor it and its loss torpedoes the
    /// run, so a silent recovery would contradict the guarantee callers rely on.
    /// Everything else reconnects, because nothing depends on its liveness.
    pub fn for_qos(qos: Qos) -> Self {
        if qos.fires_torpedo() {
            Self::Never
        } else {
            Self::always()
        }
    }
}

/// Per-node timing policy.
///
/// The constants above are the defaults, chosen for a LAN-attached replay rig
/// where a node that goes quiet for 800 ms really has died. They are far too
/// tight for a link that is expected to wobble — an SSH tunnel to a robot on
/// cellular, a laptop roaming between access points — where a one-second hiccup
/// is normal and must not be read as a death.
///
/// A node carries its own `Timing` and tells the coordinator about it when it
/// joins, so the two sides agree on how patient to be with each other.
///
/// The fields are interdependent. [`Timing::validate`] checks the relationships
/// that the rest of the system relies on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Timing {
    /// How often this node sends a heartbeat to keep its connection alive.
    pub heartbeat_interval_ms: u64,
    /// A heartbeat is skipped if real data was sent within this window.
    pub heartbeat_suppress_ms: u64,
    /// How long the coordinator waits with no message before declaring this
    /// node gone, and how long this node waits for heartbeat echoes before
    /// declaring the coordinator gone.
    pub disconnect_timeout_ms: u64,
    /// How long to wait for a coordinator before giving up on registration.
    pub registration_timeout_ms: u64,
    /// Consecutive peer ping failures before declaring that peer dead.
    pub peer_dead_threshold: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self {
            heartbeat_interval_ms: HEARTBEAT_INTERVAL_MS,
            heartbeat_suppress_ms: HEARTBEAT_SUPPRESS_MS,
            disconnect_timeout_ms: DISCONNECT_TIMEOUT_MS,
            registration_timeout_ms: REGISTRATION_TIMEOUT_MS,
            peer_dead_threshold: PEER_DEAD_THRESHOLD,
        }
    }
}

impl Timing {
    /// Timing for a link that is expected to wobble: off-LAN, tunnelled,
    /// wireless, or otherwise not under your control.
    ///
    /// Roughly 5× more patient than the default. Dead-node detection takes about
    /// 10 seconds.
    pub fn wan() -> Self {
        Self {
            heartbeat_interval_ms: 2_000,
            heartbeat_suppress_ms: 1_000,
            disconnect_timeout_ms: 10_000,
            registration_timeout_ms: 15_000,
            peer_dead_threshold: 3,
        }
    }

    /// Check the relationships the rest of the system relies on.
    ///
    /// Called when a node is built, so a bad custom `Timing` fails loudly at
    /// startup with a concrete configuration error.
    pub fn validate(&self) -> anyhow::Result<()> {
        if self.heartbeat_interval_ms == 0 {
            anyhow::bail!("Timing::heartbeat_interval_ms must be greater than zero");
        }
        if self.heartbeat_suppress_ms > self.heartbeat_interval_ms {
            anyhow::bail!(
                "Timing::heartbeat_suppress_ms ({}) must be <= heartbeat_interval_ms ({}), \
                 otherwise a heartbeat can be suppressed past the disconnect timeout",
                self.heartbeat_suppress_ms,
                self.heartbeat_interval_ms
            );
        }
        if self.disconnect_timeout_ms <= self.heartbeat_interval_ms + self.heartbeat_suppress_ms {
            anyhow::bail!(
                "Timing::disconnect_timeout_ms ({}) must be > heartbeat_interval_ms + \
                 heartbeat_suppress_ms ({}), otherwise healthy nodes are declared dead",
                self.disconnect_timeout_ms,
                self.heartbeat_interval_ms + self.heartbeat_suppress_ms
            );
        }
        if self.peer_dead_threshold == 0 {
            anyhow::bail!("Timing::peer_dead_threshold must be greater than zero");
        }
        Ok(())
    }

    pub fn heartbeat_interval(&self) -> std::time::Duration {
        std::time::Duration::from_millis(self.heartbeat_interval_ms)
    }

    pub fn heartbeat_suppress(&self) -> std::time::Duration {
        std::time::Duration::from_millis(self.heartbeat_suppress_ms)
    }

    pub fn disconnect_timeout(&self) -> std::time::Duration {
        std::time::Duration::from_millis(self.disconnect_timeout_ms)
    }

    pub fn registration_timeout(&self) -> std::time::Duration {
        std::time::Duration::from_millis(self.registration_timeout_ms)
    }
}

/// [try-reliable] Total budget for one `Qos::TryReliable` delivery.
/// Generous enough to ride out a WiFi roam or a retry burst, bounded so a
/// send can never wedge the caller's loop the way `Qos::Reliable` can.
pub const TRY_RELIABLE_SEND_BUDGET_MS: u64 = 3000;

/// [try-reliable] Per-attempt query timeout inside that budget. Must be well
/// below TRY_RELIABLE_SEND_BUDGET_MS so a stalled attempt leaves room to retry.
pub const TRY_RELIABLE_ATTEMPT_TIMEOUT_MS: u64 = 500;

/// [try-reliable] Pause between failed attempts, so a dead target is not
/// hammered for the whole budget.
pub const TRY_RELIABLE_RETRY_BACKOFF_MS: u64 = 50;

/// [best-effort] Single-attempt query timeout. Best-effort never retries: the
/// next sample is worth more than this one.
pub const BEST_EFFORT_ATTEMPT_TIMEOUT_MS: u64 = 250;

/// [coordinator] Last-resort timeout for coordinator-side per-client handler.
/// This fires for
/// truly isolated or zombie clients that sent no packet for this long.
pub const COORD_CLIENT_IDLE_TIMEOUT_MS: u64 = 30_000;

use mt_net::{ActionPlan, BagMsg, Rules, VariableHuman};

/// An immutable, validated rkyv message that owns its backing byte buffer.
///
/// [`archived`](Self::archived) borrows directly from the stored buffer without
/// deserializing an owned `T`. Keeping the owner and view in one type prevents
/// the archived reference from outliving its bytes.
pub struct ArchivedMessage<T: Archive> {
    bytes: AlignedVec,
    _type: std::marker::PhantomData<fn() -> T>,
}

impl<T> std::fmt::Debug for ArchivedMessage<T>
where
    T: Archive,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("ArchivedMessage")
            .field("len", &self.bytes.len())
            .finish_non_exhaustive()
    }
}

impl<T> ArchivedMessage<T>
where
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>,
{
    pub(crate) fn from_aligned_bytes(bytes: AlignedVec) -> anyhow::Result<Self> {
        rkyv::access::<T::Archived, rkyv::rancor::Error>(&bytes)
            .map_err(|error| anyhow::anyhow!("Could not validate archived message: {error}"))?;
        Ok(Self {
            bytes,
            _type: std::marker::PhantomData,
        })
    }

    /// Access the archived representation without allocating or deserializing `T`.
    pub fn archived(&self) -> &T::Archived {
        // SAFETY: construction validates the buffer as T::Archived, and `bytes` is
        // private and never exposed mutably, so that validation remains valid.
        unsafe { rkyv::access_unchecked::<T::Archived>(&self.bytes) }
    }

    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes
    }

    pub fn len(&self) -> usize {
        self.bytes.len()
    }

    pub fn is_empty(&self) -> bool {
        self.bytes.is_empty()
    }
}

impl<T> ArchivedMessage<T>
where
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
{
    /// Deserialize an owned value when an existing API still requires `T`.
    pub fn deserialize(&self) -> anyhow::Result<T> {
        let mut pool = Pool::new();
        self.archived()
            .deserialize(Strategy::wrap(&mut pool))
            .map_err(|error| anyhow::anyhow!("Could not deserialize archived message: {error}"))
    }
}

/// Initialize logging in the house format, with the transport crates capped
/// however loud `RUST_LOG` asks everything else to be.
pub fn init_logging() {
    mt_log::init_filtered("Minot", "RUST_LOG", "info", mt_log::QUIET_ZENOH);
}

use rkyv::{
    Archive, Deserialize, Serialize,
    api::high::{HighSerializer, HighValidator},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    ser::allocator::ArenaHandle,
    util::AlignedVec,
};

#[derive(Debug, Clone, PartialEq, Archive, Serialize, Deserialize, Hash, Eq, PartialOrd, Ord)]
pub enum ShipKind {
    Rat(String),
    Wind(String),
}

pub type ShipName = i128;

#[derive(Debug, Clone, Serialize, Deserialize, Archive, PartialEq, Eq)]
pub struct NetworkShipAddress {
    ip: [u8; 4],
    pub port: u16,
    ship: ShipName,
    pub kind: ShipKind,
    pub node_mode: net::Qos,
}

#[derive(Debug, Archive, Clone, Default, Serialize, Deserialize)]
pub enum Action {
    #[default]
    Sail,
    Shoot {
        target: Vec<NetworkShipAddress>,
        id: u32,
    },
    Catch {
        source: NetworkShipAddress,
        id: u32,
    },
}

#[derive(Clone, Debug)]
pub struct Variable {
    pub ship: ShipName,
    pub strategy: Option<Action>,
}

pub fn get_strategies(
    haystack: &Rules,
    rat_ship: &str,
    variable: String,
    indirect_parent_rat: Option<&str>,
) -> Vec<ActionPlan> {
    match haystack.raw().get(&variable) {
        None => vec![ActionPlan::default()],
        Some(plans) => {
            // directly because rule was set
            let directly = plans
                .iter()
                .filter(|plan| plan.ship == rat_ship)
                .filter_map(|el| el.strategy.clone())
                .collect::<Vec<_>>();

            // as other part of a rule
            let mut indirect = plans
                .iter()
                .filter(|plan| indirect_parent_rat.is_none_or(|parent_rat| plan.ship == parent_rat))
                .filter_map(|plan| match plan.strategy.as_ref()? {
                    ActionPlan::Sail => None,
                    ActionPlan::Shoot { target, id } => target
                        .iter()
                        .find(|shoot_target| *shoot_target == rat_ship)
                        .map(|_| ActionPlan::Catch {
                            source: plan.ship.clone(),
                            id: *id,
                        }),
                    ActionPlan::Catch { source, id } => {
                        if source == rat_ship {
                            Some(ActionPlan::Shoot {
                                target: vec![source.clone()],
                                id: *id,
                            })
                        } else {
                            None
                        }
                    }
                })
                .collect::<Vec<_>>();

            indirect.extend(directly);
            indirect
        }
    }
}

#[async_trait::async_trait]
pub trait Ship: Send + Sync + 'static {
    /// Indicate a trigger point and ask the link pilot what to do with the variable.
    async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<(Action, bool)>;

    // async fn wait_for_action(&self) -> anyhow::Result<crate::Action>;

    async fn wait_for_wind(&self) -> anyhow::Result<Vec<WindData>>;

    fn get_cannon(&self) -> &impl Cannon;
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone, Copy, Default)]
pub enum VariableType {
    #[default]
    StaticOnly, // statically supported but no dynamic conversion implemented
    U8,
    I32,
    F32,
    F64,
}

impl From<u8> for VariableType {
    fn from(value: u8) -> Self {
        match value {
            1 => Self::U8,
            2 => Self::I32,
            3 => Self::F32,
            4 => Self::F64,
            _ => Self::default(),
        }
    }
}

impl From<VariableType> for u8 {
    fn from(value: VariableType) -> Self {
        match value {
            VariableType::StaticOnly => 0,
            VariableType::U8 => 1,
            VariableType::I32 => 2,
            VariableType::F32 => 3,
            VariableType::F64 => 4,
        }
    }
}

use rkyv::rancor::Error as RkyvError;

// Trait for types that can be Sent (Serialized).
// Requires Sized, Send, Sync, 'static, and the specific rkyv Serialize bound.
pub trait Sendable: Sized + Send + Sync + 'static
where
    Self: for<'b> Serialize<HighSerializer<AlignedVec, ArenaHandle<'b>, RkyvError>>,
    Self: Archive<
        Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
                      + Deserialize<Self, Strategy<Pool, rkyv::rancor::Error>>,
    >,
{
}
// Blanket implementation for Sendable. Any type meeting the bounds is Sendable.
impl<T> Sendable for T
where
    T: Sized + Send + Sync + 'static,
    T: for<'b> Serialize<HighSerializer<AlignedVec, ArenaHandle<'b>, RkyvError>>,
    T: Archive<
        Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
                      + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    >,
{
}

#[async_trait::async_trait]
pub trait Cannon: Send + Sync + 'static {
    // Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.

    /// Dump the data to the target.
    async fn shoot<'b, T: Sendable>(
        &self,
        targets: &'b [crate::NetworkShipAddress],
        id: u32,
        data: &T,
        variable_type: VariableType,
        variable_name: &str,
    ) -> anyhow::Result<()>;

    /// Catch the dumped data from the source.
    /// The returning Vec can contain previously missed entities of T from existing sync connections.
    /// The first item of T is the newest, followed by incremental older ones.
    async fn catch<T: Sendable>(&self, id: u32) -> anyhow::Result<Vec<T>>;

    /// Catch validated archived messages without deserializing owned values.
    async fn catch_archived<T: Sendable>(&self, id: u32)
    -> anyhow::Result<Vec<ArchivedMessage<T>>>;

    async fn catch_dyn(&self, id: u32) -> anyhow::Result<Vec<(String, VariableType, String)>>;
}

#[derive(Clone, Debug, Default, Copy, Archive, Serialize, Deserialize, PartialEq)]
pub struct TimeMsg {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Archive, Serialize, Deserialize)]
pub struct Header {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

pub type WindData = BagMsg;

#[async_trait::async_trait]
pub trait Coordinator: Send + Sync + 'static {
    async fn rat_action_request_queue(
        &self,
        ship: String,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>>;

    async fn blow_wind(&self, ship: String, data: Vec<WindData>) -> anyhow::Result<()>;

    async fn rat_action_send(
        &self,
        ship: String,
        variable: String,
        action: ActionPlan,
        lock_until_ack: bool,
        best_effort: bool,
    ) -> anyhow::Result<()>;

    /// Push updated routes for `variable` to all ships currently involved in it.
    /// Called after topology changes (register / disconnect / PeerDead).
    async fn push_routes_for_var(&self, variable: &str, rules: &Rules) -> anyhow::Result<()>;
}
