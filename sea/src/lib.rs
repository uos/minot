pub mod cannon;
pub mod coordinator;
mod net;
pub mod ship;

use nalgebra::{UnitQuaternion, Vector3};
use ros_pointcloud2::PointCloud2Msg;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
pub enum ShipKind {
    Rat(String),
    Wind(String),
    God, // TODO there are better names out there
}

pub type ShipName = i128;

#[derive(Debug, Copy, Clone, Default)]
pub enum Action {
    #[default]
    Sail,
    Shoot {
        target: ShipName,
    },
    Catch {
        source: ShipName,
    },
}

#[async_trait::async_trait]
pub trait Ship: Send + Sync + 'static {
    /// Register the client to the network and return the assigned member id (unique to the network).
    /// It is needed for every communication on the network.
    //async fn init(&self, node_name: &str) -> Self;

    async fn water(&mut self, kind: ShipKind) -> anyhow::Result<ShipName>;

    /// Indicate a trigger point and ask the link pilot what to do with the variable.
    async fn ask_for_action(
        &self,
        kind: crate::ShipKind,
        variable_name: &str,
    ) -> anyhow::Result<Action>;

    async fn wait_for_action(&self, kind: crate::ShipKind) -> anyhow::Result<crate::Action> {
        todo!()
    }

    fn get_name(&self) -> ShipName;

    fn get_cannon(&self) -> &Box<dyn Cannon>;
}

#[async_trait::async_trait]
pub trait Cannon: Send + Sync + 'static {
    /// Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.

    /// Dump the data to the target.
    async fn shoot(&self, target: crate::ShipName, data: &[u8]);

    /// Catch the dumped data from the source.
    async fn catch(&self, target: crate::ShipName) -> Vec<u8>;
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
        ship: ShipName,
    ) -> anyhow::Result<tokio::sync::mpsc::Receiver<String>>;

    async fn blow_wind(&self, ship: crate::ShipName, data: WindData) -> anyhow::Result<()>;

    async fn rat_action_send(&self, ship: ShipName, action: crate::Action) -> anyhow::Result<()>;
}
