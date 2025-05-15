// #![feature(async_drop)]
// #![feature(impl_trait_in_assoc_type)]
pub mod client;
pub mod coordinator;
pub mod net;
pub mod ship;

pub use bagread::SensorTypeMapped;
use nalgebra::{UnitQuaternion, Vector3};
use rkyv::{
    Archive, Deserialize, Serialize, deserialize,
    rancor::{Error, Fallible},
};
use rlc::{ActionPlan, VariableHuman};

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
    },
    Catch {
        source: NetworkShipAddress,
    },
}

// #[derive(Debug, Clone, Default, Serialize, Deserialize)]
// pub enum ActionPlan {
//     #[default]
//     Sail,
//     Shoot {
//         target: Vec<String>,
//     },
//     Catch {
//         source: String,
//     },
// }

// #[derive(Clone, Debug, Deserialize, Serialize)]
// pub struct VariableHuman {
//     pub ship: String,
//     pub strategy: Option<ActionPlan>,
// }

#[derive(Clone, Debug)]
pub struct Variable {
    pub ship: ShipName,
    pub strategy: Option<Action>,
}

pub fn get_strategies(
    haystack: &rlc::Rules,
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
                    ActionPlan::Shoot { target } => target
                        .iter()
                        .find(|shoot_target| *shoot_target == rat_ship)
                        .map(|_| ActionPlan::Catch {
                            source: plan.ship.clone(),
                        }),
                    ActionPlan::Catch { source } => {
                        if source == rat_ship {
                            Some(ActionPlan::Shoot {
                                target: vec![source.clone()],
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

#[async_trait::async_trait]
pub trait Cannon: Send + Sync + 'static {
    /// Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.

    /// Dump the data to the target.
    async fn shoot<S: Fallible>(
        &self,
        targets: &Vec<crate::NetworkShipAddress>,
        data: impl Serialize<S> + Archive,
        variable_type: VariableType,
        variable_name: &str,
    ) -> anyhow::Result<()>;

    /// Catch the dumped data from the source.
    async fn catch<'de, T, S: Fallible>(
        &self,
        target: &crate::NetworkShipAddress,
    ) -> anyhow::Result<T>
    where
        T: Deserialize<T, S>;

    async fn catch_dyn(
        &self,
        target: &crate::NetworkShipAddress,
    ) -> anyhow::Result<(String, VariableType, String)>;
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

#[derive(Clone, Default, Debug, PartialEq, Archive, Serialize, Deserialize)]
pub struct ImuMsg {
    pub header: Header,
    pub timestamp_sec: TimeMsg,
    pub orientation: UnitQuaternion<f64>,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: Vector3<f64>,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: Vector3<f64>,
    pub linear_acceleration_covariance: [f64; 9],
}

pub type WindData = bagread::BagMsg;

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
        action: rlc::ActionPlan,
        lock_until_ack: bool,
    ) -> anyhow::Result<()>;
}
