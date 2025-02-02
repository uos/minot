pub mod client;

#[cfg(test)]
pub mod medium;

pub mod cannon;
pub mod constants;
pub mod coordinator;
pub mod datapack_handler;
pub mod packet;
pub mod router;
pub mod ship;
pub mod traits;

use nalgebra::{UnitQuaternion, Vector3};
use ros_pointcloud2::PointCloud2Msg;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
pub enum ShipKind {
    Rat(String),
    Wind(String),
    God, // TODO there are better names out there
}

pub type ShipName = u128;

#[derive(Debug, Copy, Clone)]
pub enum Action {
    Sail,
    Shoot { target: ShipName },
    Catch { source: ShipName },
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

#[cfg(test)]
mod tests {
    use ipnet::IpNet;
    use std::net::{IpAddr, Ipv4Addr};

    use crate::{
        datapack_handler::{
            get_data_from_datapacket, get_datapackets_from_datachunk, SequenceCollector,
            SequenceSenderHelper,
        },
        traits::{ClientTrait, RouterTrait},
    };

    use log::info;

    use super::*;

    #[test]
    fn packet_serialization() {
        // ConnectRequest
        let datapacket = packet::DataPacket::new(100, 102, packet::DataPacketType::ConnectRequest);
        let serialized: Vec<u8> = datapacket.clone().into();
        let deserialized = packet::DataPacket::from(&serialized);
        assert_eq!(deserialized.source, datapacket.source);
        assert_eq!(deserialized.payload, datapacket.payload);
        assert_eq!(deserialized.destination, datapacket.destination);

        // ConnectResponse
        let datapacket = packet::DataPacket::new(
            100,
            102,
            packet::DataPacketType::ConnectResponse {
                ip: [192, 168, 0, 1],
                prefix_len: 24,
                device_id: 101,
            },
        );
        let serialized: Vec<u8> = datapacket.clone().into();
        let deserialized = packet::DataPacket::from(&serialized);
        assert_eq!(deserialized.source, datapacket.source);
        assert_eq!(deserialized.payload, datapacket.payload);
        assert_eq!(deserialized.destination, datapacket.destination);

        // Ack
        let datapacket = packet::DataPacket::new(100, 102, packet::DataPacketType::Ack);
        let serialized: Vec<u8> = datapacket.clone().into();
        let deserialized = packet::DataPacket::from(&serialized);
        assert_eq!(deserialized.source, datapacket.source);
        assert_eq!(deserialized.payload, datapacket.payload);
        assert_eq!(deserialized.destination, datapacket.destination);

        // TODO more payload types
    }

    #[tokio::test]
    async fn generate_network_ip() {
        let mut ipv4_network = router::RouterIpNetwork::new([192, 168, 0, 0], [255, 255, 255, 0]);
        let generated_client_ip = ipv4_network.register();
        assert!(generated_client_ip.is_ok());
        assert_eq!(generated_client_ip.unwrap(), [192, 168, 0, 1]);

        let generated_client_ip = ipv4_network.register();
        assert!(generated_client_ip.is_ok());
        assert_eq!(generated_client_ip.unwrap(), [192, 168, 0, 2]);
    }

    #[tokio::test]
    async fn router_ip_configuration() {
        let network: IpNet = "192.168.0.0/24".parse().unwrap();

        assert_eq!(network.prefix_len(), 24);
        assert_eq!(network.addr(), IpAddr::V4(Ipv4Addr::new(192, 168, 0, 0)));
        assert_eq!(
            network.netmask(),
            IpAddr::V4(Ipv4Addr::new(255, 255, 255, 0))
        );
        let router = router::Router::new(network).await;
        let ipnet = router.get_ip_net().await;
        assert_eq!(ipnet, network);
    }

    #[tokio::test]
    async fn client_router_basics() {
        let (medium, _) = tokio::sync::broadcast::channel(10);

        // large buffer to avoid blocking because we never read from it for this unit tests. They are for integration with the daemon.
        let (radio_reconfigure_snd, mut radio_reconfigure_rcv) = tokio::sync::mpsc::channel(10);
        let (vnet_reconfigure_snd, mut vnet_reconfigure_rcv) = tokio::sync::mpsc::channel(10);
        tokio::spawn(async move {
            loop {
                tokio::select! {
                    Some((device_id, network_id)) = radio_reconfigure_rcv.recv() => {
                        info!("\\\\|| configure radio: device_id {:?} network_id {:?}", device_id, network_id);
                    }
                    Some((network, default_route)) = vnet_reconfigure_rcv.recv() => {
                        info!("\\\\|| configure vnet: network {:?} default_route {:?}", network, default_route);
                    }
                }
            }
        });

        info!("=================================   CREATING   =================================");
        // Create router/server
        let network: IpNet = "192.168.0.0/24".parse().unwrap();
        let router = router::Router::new(network).await;
        info!(
            "#---# Router created with device_id {:?} #---#",
            router.get_device_id().await
        );

        let router_device_id = router.get_device_id().await;
        assert_eq!(router_device_id, crate::constants::NET_ID);

        // Register and run router
        let router_vlan_snd = tokio::sync::broadcast::channel(1).0;
        let mut router_vlan = medium::Vlan::new(router_vlan_snd.clone());
        let router_vlan_rcv = router_vlan.register_client();
        router
            .run_init(
                router_vlan_snd,
                router_vlan_rcv,
                medium.clone(),
                medium.subscribe(),
                radio_reconfigure_snd.clone(),
                vnet_reconfigure_snd.clone(),
            )
            .await;
        router_vlan.run_loop();

        // Create client
        let client_1 = client::Client::new();
        let tmp_device_id = client_1.get_device_id().await;
        assert!(tmp_device_id >= 250);
        info!(
            "#---# Client_1 created with temporal device_id {:?} #---#",
            tmp_device_id
        );

        // Register and run client
        let client_1_vlan_snd = tokio::sync::broadcast::channel(1).0;
        let mut client_1_vlan = medium::Vlan::new(client_1_vlan_snd.clone());
        let client_1_vlan_rcv = client_1_vlan.register_client();
        client_1
            .run_init(
                client_1_vlan_snd,
                client_1_vlan_rcv,
                medium.clone(),
                medium.subscribe(),
                radio_reconfigure_snd,
                vnet_reconfigure_snd,
            )
            .await;
        client_1_vlan.run_loop();

        // Connect client to network
        client_1.connect_to_network().await;
        let device_id = client_1.get_device_id().await;
        assert!(device_id < 250);
        info!(
            "#---# Client_1 connected with new device_id {:?} #---#",
            device_id
        );

        info!("=================================   CONNECTING   =================================");
        info!(
            "client_1_id {:?} #---# router_id {:?}",
            client_1.get_device_id().await,
            router.get_device_id().await
        );

        info!("================================   PING   ================================");
        let dur = router.ping(client_1.get_device_id().await).await.unwrap();
        info!("router -> client: {:?}", dur);

        let dur = client_1.ping(router.get_device_id().await).await.unwrap();
        info!("client -> router: {:?}", dur);

        info!(
            "================================   DISCONNECTING   ================================"
        );
        client_1.disconnect_from_network().await;
        info!("#---# Client_1 disconnected. #---#");
    }

    #[tokio::test]
    async fn partition_test() {
        env_logger::builder()
            .filter_level(log::LevelFilter::Info)
            .init();

        // Make data vector
        let len = 250;
        let mut zero_vec: Vec<u8> = vec![0; len];

        zero_vec[0] = 25;

        for n in 3..250 {
            if n % 3 == 0 {
                zero_vec[n] = n as u8;
            }
        }

        zero_vec[31] = 37;

        // clone vector
        let zero_vec_copy = zero_vec.clone();

        // Partition and put back together
        let datapackets = get_datapackets_from_datachunk(1, 2, zero_vec);
        let mut seq_hel = SequenceSenderHelper::new(datapackets).await;
        let mut next_packet = seq_hel.get_next_packet().await;
        let first_packet = next_packet.unwrap();
        let seq_col = SequenceCollector::new(first_packet.clone()).await;
        next_packet = seq_hel.get_next_packet().await;
        let sec_packet = next_packet.clone().unwrap();

        info!(
            "Firstpacket, data: {:?}",
            get_data_from_datapacket(first_packet.clone())
        );
        info!(
            "Sec packet, data: {:?}",
            get_data_from_datapacket(sec_packet.clone())
        );

        while next_packet.is_some() {
            seq_col.add_packet(next_packet.unwrap()).await;
            next_packet = seq_hel.get_next_packet().await;
        }

        // Receiving must be complete her
        let rec_compl = seq_col.receive_complete.lock().await.unwrap();
        if !rec_compl {
            panic!("Receiving was not complete despite all packets having been added");
        }
        // Get data back out
        let merged_data = seq_col.get_data_of_seq().await;

        // Compare
        // Check compare function
        if !match_bool(zero_vec_copy.clone(), zero_vec_copy.clone()) {
            panic!("Match function broken");
        }

        info!("Zero Vec: {:?}", zero_vec_copy.clone());
        info!("Merged: {:?}", merged_data.clone());

        if !match_bool(zero_vec_copy, merged_data) {
            panic!("Original and partitioned+merged not the same");
        }
    }

    pub fn match_num(a: Vec<u8>, b: Vec<u8>) -> usize {
        a.iter().zip(&b).filter(|&(a, b)| a == b).count()
    }

    pub fn match_bool(a: Vec<u8>, b: Vec<u8>) -> bool {
        if a.len() != b.len() {
            return false;
        }
        if a.len() != match_num(a, b) {
            return false;
        }
        true
    }
}
