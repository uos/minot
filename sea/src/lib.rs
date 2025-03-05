// #![feature(async_drop)]
// #![feature(impl_trait_in_assoc_type)]
pub mod client;
pub mod coordinator;
pub mod net;
pub mod ship;

use std::collections::HashMap;

use nalgebra::{UnitQuaternion, Vector3};
use net::SeaSendableBuffer;
use ros_pointcloud2::PointCloud2Msg;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize, Hash, Eq, PartialOrd, Ord)]
pub enum ShipKind {
    Rat(String),
    Wind(String),
    God, // TODO there are better names out there
}

pub type ShipName = i128;

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NetworkShipAddress {
    ip: [u8; 4],
    port: u16,
    ship: ShipName,
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub enum ActionPlan {
    #[default]
    Sail,
    Shoot {
        target: Vec<String>,
    },
    Catch {
        source: String,
    },
}

#[derive(Clone, Debug)]
pub struct VariableHuman {
    pub ship: String,
    pub strategy: Option<ActionPlan>,
}

#[derive(Clone, Debug)]
pub struct Variable {
    pub ship: ShipName,
    pub strategy: Option<Action>,
}

pub type VarPair = (Variable, Variable);
pub type HumanVarPair = (VariableHuman, VariableHuman);

pub fn get_strategy(
    haystack: &HashMap<String, HumanVarPair>,
    rat_ship: &str,
    variable: String,
) -> ActionPlan {
    match haystack.get(&variable) {
        None => ActionPlan::default(),
        Some((left_var, right_var)) => match (left_var.ship.as_str(), right_var.ship.as_str()) {
            (ship_l, ship_r) if ship_l == rat_ship && ship_r == rat_ship => {
                panic!("Rule with 2 of the same ships. Should have been checked before");
            }
            (ship, _) if ship == rat_ship => match left_var.strategy.as_ref() {
                Some(action) => action.clone(),
                None => ActionPlan::default(),
            },
            (_, ship) if ship == rat_ship => match right_var.strategy.as_ref() {
                Some(action) => action.clone(),
                None => ActionPlan::default(),
            },
            _ => ActionPlan::default(),
        },
    }
}

#[async_trait::async_trait]
pub trait Ship: Send + Sync + 'static {
    /// Indicate a trigger point and ask the link pilot what to do with the variable.
    async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<Action>;

    async fn wait_for_action(&self) -> anyhow::Result<crate::Action>;

    async fn wait_for_wind(&self) -> anyhow::Result<WindData>;

    fn get_cannon(&self) -> &impl Cannon;
}

#[async_trait::async_trait]
pub trait Cannon: Send + Sync + 'static {
    /// Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.

    /// Dump the data to the target.
    async fn shoot(
        &self,
        targets: &Vec<crate::NetworkShipAddress>,
        data: impl net::SeaSendableBuffer,
    ) -> anyhow::Result<()>;

    /// Catch the dumped data from the source.
    async fn catch<T>(&self, target: &crate::NetworkShipAddress) -> anyhow::Result<T>
    where
        T: SeaSendableBuffer;
}

#[derive(Clone, Debug, Default, Copy, Serialize, Deserialize, PartialEq)]
pub struct TimeMsg {
    pub sec: i32,
    pub nanosec: u32,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Header {
    pub seq: u32,
    pub stamp: TimeMsg,
    pub frame_id: String,
}

#[derive(Clone, Default, Debug, PartialEq, Serialize, Deserialize)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WindData {
    Pointcloud(PointCloud2Msg),
    Imu(ImuMsg),
}

#[async_trait::async_trait]
pub trait Coordinator: Send + Sync + 'static {
    async fn rat_action_request_queue(
        &self,
        ship: String,
    ) -> anyhow::Result<tokio::sync::broadcast::Receiver<String>>;

    async fn blow_wind(&self, ship: String, data: WindData) -> anyhow::Result<()>;

    async fn rat_action_send(&self, ship: String, action: crate::ActionPlan) -> anyhow::Result<()>;
}
