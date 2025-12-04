// #![feature(async_drop)]
// #![feature(impl_trait_in_assoc_type)]
pub mod client;
pub mod coordinator;
pub mod net;
pub mod ship;

use ::net::{ActionPlan, BagMsg, Rules, VariableHuman};
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
    port: u16,
    ship: ShipName,
    pub kind: ShipKind,
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
                .filter_map(|plan| {
                    if let Some(partner) = indirect_parent_rat {
                        if plan.ship == partner {
                            Some(plan)
                        } else {
                            None
                        }
                    } else {
                        Some(plan)
                    }
                })
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
{
}
// Blanket implementation for Sendable. Any type meeting the bounds is Sendable.
impl<T> Sendable for T
where
    T: Sized + Send + Sync + 'static,
    T: for<'b> Serialize<HighSerializer<AlignedVec, ArenaHandle<'b>, RkyvError>>,
{
}

#[async_trait::async_trait]
pub trait Cannon: Send + Sync + 'static {
    /// Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.

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
    async fn catch<T>(&self, id: u32) -> anyhow::Result<Vec<T>>
    where
        T: Send,
        T: Archive,
        T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
            + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>;

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
        action: ActionPlan,
        lock_until_ack: bool,
    ) -> anyhow::Result<()>;
}
