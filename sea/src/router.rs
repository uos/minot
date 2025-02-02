use std::collections::HashSet;
use std::sync::Arc;

use etherparse::SlicedPacket;
use rand::Rng;

use crate::client::PacketHandled;
use ipnet::{IpNet, Ipv4Net};
use log::{error, info};
use std::net::{IpAddr, Ipv4Addr};
use std::time::Duration;
use tokio::sync::{broadcast, mpsc, Mutex};

use crate::packet::DeviceID;
use crate::packet::{DataPacket, DataPacketType};
use crate::traits::{ClientTrait, RadioModuleTrait, VirtualEthernetTrait};
use async_trait::async_trait;
use bimap::BiHashMap;

#[derive(Debug, Clone)]
pub struct RouterIpNetwork {
    ip: [u8; 4],
    mask: [u8; 4],

    registered: HashSet<Ipv4Addr>,
}

#[allow(dead_code)]
#[derive(Debug, Clone)]
pub enum RouterIpNetworkError {
    InvalidIp,
    InvalidMask,
    TooManyDevices,
}

impl RouterIpNetwork {
    pub fn new(ip: [u8; 4], mask: [u8; 4]) -> RouterIpNetwork {
        RouterIpNetwork {
            ip,
            mask,
            registered: HashSet::new(),
        }
    }

    pub fn prefix_len(&self) -> u8 {
        self.mask
            .iter()
            .fold(0, |acc, x| acc + x.count_ones() as u8)
    }

    pub fn register(&mut self) -> Result<[u8; 4], RouterIpNetworkError> {
        let network_ip = Ipv4Addr::new(self.ip[0], self.ip[1], self.ip[2], self.ip[3]);
        let prefix_len = self.prefix_len();

        let net = match Ipv4Net::new(network_ip, prefix_len) {
            Ok(net) => net,
            Err(_) => return Err(RouterIpNetworkError::InvalidMask),
        };

        for addr in net.hosts() {
            if !self.registered.contains(&addr) {
                self.registered.insert(addr);
                return Ok(addr.octets());
            }
        }

        Err(RouterIpNetworkError::TooManyDevices)
    }

    pub fn unregister_from_network(&mut self, ip: [u8; 4]) {
        self.registered
            .remove(&Ipv4Addr::new(ip[0], ip[1], ip[2], ip[3]));
    }
}

#[derive(Debug, Clone)]
pub enum RouterRequest {
    ConnectRequest(DeviceID),
    DisconnectRequest(DeviceID),
    PassThroughRequest(DeviceID, Vec<u8>), // target device id, data
}

pub struct Router {
    client: Arc<Mutex<crate::client::Client>>,
    ip_v4_subnet: Arc<Mutex<RouterIpNetwork>>,
    router_queue_sender: broadcast::Sender<RouterRequest>,
    ip_device_id: Arc<Mutex<BiHashMap<[u8; 4], DeviceID>>>,
}

#[derive(Debug, Clone, Copy)]
pub enum FrameProcessError {
    Read,
    Write,
    NoIpHeader,
    UnknownProtocol,
    ProtocolNotSupported,
    NoPacket,
    MediumBusy,
    NoAck,
}

#[async_trait]
impl crate::traits::RouterTrait for Router {
    async fn new(network: IpNet) -> Self {
        let (router_queue, _) = broadcast::channel(100);

        let client_ip = match network.addr() {
            IpAddr::V4(ip) => ip.octets(),
            IpAddr::V6(_) => panic!("ipv6 not supported"),
        };

        let client_mask = match network.netmask() {
            IpAddr::V4(ip) => ip.octets(),
            IpAddr::V6(_) => panic!("ipv6 not supported"),
        };

        let mut client = crate::client::Client::new();
        client.set_as_router_client(router_queue.clone()).await;
        let client = Arc::new(Mutex::new(client));
        let ip_v4_subnet = Arc::new(Mutex::new(RouterIpNetwork::new(client_ip, client_mask)));

        Router {
            client,
            ip_v4_subnet,
            router_queue_sender: router_queue,
            ip_device_id: Arc::new(Mutex::new(BiHashMap::new())),
        }
    }

    async fn run(
        &self,
        vnet: Arc<impl VirtualEthernetTrait>,
        vnet_send: broadcast::Sender<Vec<u8>>,
        vnet_rcv: broadcast::Receiver<Vec<u8>>,
        radio: Arc<impl RadioModuleTrait>,
        radio_send: broadcast::Sender<Vec<u8>>,
        radio_rcv: broadcast::Receiver<Vec<u8>>,
    ) -> () {
        let (radio_reconfigure_sender, mut radio_reconfigure_receiver) =
            mpsc::channel::<(u8, u8)>(10);
        let (vnet_reconfigure_sender, mut vnet_reconfigure_receiver) =
            mpsc::channel::<(Option<IpNet>, bool)>(10);
        self.run_init(
            vnet_send,
            vnet_rcv,
            radio_send,
            radio_rcv,
            radio_reconfigure_sender,
            vnet_reconfigure_sender,
        )
        .await;
        loop {
            tokio::select! {
                reconfigure_msg = radio_reconfigure_receiver.recv() => {
                    if let Some((chip_address, network_id)) = reconfigure_msg {
                        radio.reconfigure(chip_address, network_id).await.expect("Failed to reconfigure in daemon");
                    }
                }
                reconfigure_msg = vnet_reconfigure_receiver.recv() => {
                    if let Some((ipv4, is_router)) = reconfigure_msg {
                        vnet.reconfigure(ipv4, is_router).await.expect("Failed to reconfigure in daemon");
                    }
                }
            }
        }
    }

    async fn ping(&self, peer_address: u8) -> Result<Duration, ()> {
        self.client.lock().await.ping(peer_address).await
    }

    async fn get_peers(&self) -> Result<Vec<(u8, Ipv4Addr)>, ()> {
        self.ip_device_id
            .lock()
            .await
            .iter()
            .map(|(k, v)| Ok((*v, Ipv4Addr::new(k[0], k[1], k[2], k[3]))))
            .collect()
    }
}

impl Router {
    pub async fn run_init(
        &self,
        vnet_send: broadcast::Sender<Vec<u8>>,
        vnet_rcv: broadcast::Receiver<Vec<u8>>,
        radio_send: broadcast::Sender<Vec<u8>>,
        radio_rcv: broadcast::Receiver<Vec<u8>>,
        radio_reconfigure_sender: mpsc::Sender<(u8, u8)>,
        vnet_reconfigure_sender: mpsc::Sender<(Option<ipnet::IpNet>, bool)>,
    ) -> () {
        let (ip, prefix_len) = {
            let ip_v4_subnet = self.ip_v4_subnet.lock().await;
            let prefix_len = ip_v4_subnet.prefix_len();
            (ip_v4_subnet.ip, prefix_len)
        };
        let conf_ip: IpNet = format!("{}.{}.{}.{}/{}", ip[0], ip[1], ip[2], ip[3], prefix_len)
            .parse()
            .expect("failed to parse ip");
        vnet_reconfigure_sender
            .send((Some(conf_ip), false))
            .await
            .expect("failed to send vnet reconfigure in router at startup");
        let outgoing = {
            let client = self.client.lock().await;
            client
                .run_init(
                    vnet_send.clone(),
                    vnet_rcv,
                    radio_send,
                    radio_rcv,
                    radio_reconfigure_sender,
                    vnet_reconfigure_sender,
                )
                .await;
            client.outgoing.clone()
        };

        let network_management_jobs = self.router_queue_sender.subscribe();
        let ip_v4_subnet = self.ip_v4_subnet.clone();
        let device_ids = self.ip_device_id.clone();

        tokio::spawn(async move {
            Self::job_queue_worker(
                network_management_jobs,
                ip_v4_subnet,
                device_ids,
                outgoing,
                vnet_send.clone(),
            )
            .await;
        });
    }

    pub async fn get_ip_net(&self) -> IpNet {
        let subnet = self.ip_v4_subnet.lock().await;
        let prefix_len = subnet.prefix_len();
        let ip = subnet.ip;
        format!("{}.{}.{}.{}/{}", ip[0], ip[1], ip[2], ip[3], prefix_len)
            .parse()
            .expect("failed to parse ip")
    }

    async fn generate_device_id(device_ids: &Mutex<BiHashMap<[u8; 4], DeviceID>>) -> DeviceID {
        let all_current_device_ids = {
            device_ids
                .lock()
                .await
                .iter()
                .map(|(_, v)| *v)
                .collect::<Vec<DeviceID>>()
        };
        let mut rng = rand::thread_rng();
        let mut device_id = rng.gen_range(
            crate::constants::NETWORK_DEVICE_ID_MIN..crate::constants::NETWORK_DEVICE_ID_MAX,
        );
        while device_id == crate::constants::NET_ID || all_current_device_ids.contains(&device_id) {
            device_id = rng.gen_range(
                crate::constants::NETWORK_DEVICE_ID_MIN..crate::constants::NETWORK_DEVICE_ID_MAX,
            );
        }
        device_id
    }

    async fn add_device(
        ip_v4_subnet: &Mutex<RouterIpNetwork>,
        device_ids: &Mutex<BiHashMap<[u8; 4], DeviceID>>,
    ) -> Result<([u8; 4], DeviceID), RouterIpNetworkError> {
        let ip = ip_v4_subnet.lock().await.register()?;
        let device_id = Self::generate_device_id(device_ids).await;
        device_ids.lock().await.insert(ip, device_id);
        Ok((ip, device_id))
    }

    pub async fn get_network_device_id(
        ip_v4_subnet: &Mutex<RouterIpNetwork>,
        ip: &[u8; 4],
        device_ids: &Mutex<BiHashMap<[u8; 4], DeviceID>>,
    ) -> Option<DeviceID> {
        match ip_v4_subnet
            .lock()
            .await
            .registered
            .contains(&Ipv4Addr::new(ip[0], ip[1], ip[2], ip[3]))
        {
            true => {
                let device_ids = device_ids.lock().await;
                Some(device_ids.get_by_left(ip).unwrap().clone())
            }
            false => None,
        }
    }

    pub async fn ethernet_frame_device_id(
        ip_v4_subnet: &Mutex<RouterIpNetwork>,
        data: &Vec<u8>,
        device_ids: &Mutex<BiHashMap<[u8; 4], DeviceID>>,
    ) -> Result<Option<DeviceID>, FrameProcessError> {
        match SlicedPacket::from_ethernet(data.as_slice()) {
            Err(_) => return Err(FrameProcessError::Read),
            Ok(value) => {
                let ip_header = value.ip.expect("no ip header in packet");
                let destination_ip = match ip_header {
                    etherparse::InternetSlice::Ipv4(header, _) => header.destination(),
                    etherparse::InternetSlice::Ipv6(_, _) => todo!(),
                };
                Ok(Self::get_network_device_id(ip_v4_subnet, &destination_ip, device_ids).await)
            }
        }
    }

    pub async fn remove_device(
        device_id: DeviceID,
        ip_v4_subnet: &Mutex<RouterIpNetwork>,
        device_ids: &Mutex<BiHashMap<[u8; 4], DeviceID>>,
    ) {
        let ip = {
            device_ids
                .lock()
                .await
                .remove_by_right(&device_id)
                .expect("device_id was not registered")
        }
        .0;
        ip_v4_subnet.lock().await.unregister_from_network(ip);
    }

    pub async fn get_device_id(&self) -> DeviceID {
        self.client.lock().await.get_device_id().await
    }

    async fn handle_routing_packet(
        job: RouterRequest,
        ip_v4_subnet: Arc<Mutex<RouterIpNetwork>>,
        device_ids: Arc<Mutex<BiHashMap<[u8; 4], DeviceID>>>,
        client_outgoing: broadcast::Sender<DataPacket>,
        vnet_send: broadcast::Sender<Vec<u8>>,
    ) -> PacketHandled {
        match job {
            RouterRequest::ConnectRequest(device_id) => {
                let (reg_ip, reg_device_id) =
                    Self::add_device(ip_v4_subnet.as_ref(), device_ids.as_ref())
                        .await
                        .expect("failed to add device");

                let prefix_len = { ip_v4_subnet.lock().await.prefix_len() };

                client_outgoing
                    .send(DataPacket::new(
                        crate::constants::NET_ID,
                        device_id,
                        DataPacketType::ConnectResponse {
                            ip: reg_ip,
                            prefix_len,
                            device_id: reg_device_id,
                        },
                    ))
                    .expect("reschedule broken");
                info!(
                    "[SRV] added device with ip {:?}, answer {:?}",
                    reg_ip, device_id
                );
            }
            RouterRequest::DisconnectRequest(device_id) => {
                Self::remove_device(device_id, ip_v4_subnet.as_ref(), device_ids.as_ref()).await;
                client_outgoing
                    .send(DataPacket::new(
                        crate::constants::NET_ID,
                        device_id,
                        DataPacketType::Ack,
                    ))
                    .expect("reschedule broken");
            }
            RouterRequest::PassThroughRequest(_, data) => {
                let destination_device_id = Self::ethernet_frame_device_id(
                    ip_v4_subnet.as_ref(),
                    &data,
                    device_ids.as_ref(),
                )
                .await;
                match destination_device_id {
                    Ok(Some(destination_device_id)) => {
                        if destination_device_id == crate::constants::NET_ID {
                            match vnet_send.send(data) {
                                Ok(_) => {}
                                Err(_) => {
                                    error!("failed to send packet to vlan")
                                }
                            }
                        } else {
                            client_outgoing
                                .send(DataPacket::new(
                                    crate::constants::NET_ID,
                                    destination_device_id,
                                    DataPacketType::Raw(data),
                                ))
                                .expect("reschedule broken");
                        }
                    }
                    Ok(None) => {
                        error!("no device found for packet");
                    }
                    Err(_) => {
                        error!("failed to read packet");
                    }
                }
            }
        }
        PacketHandled::Succ
    }

    pub async fn job_queue_worker(
        mut network_management_jobs: broadcast::Receiver<RouterRequest>,
        ip_v4_subnet: Arc<Mutex<RouterIpNetwork>>,
        ip_device_ids: Arc<Mutex<BiHashMap<[u8; 4], DeviceID>>>,
        client_outgoing: broadcast::Sender<DataPacket>,
        vnet_send: broadcast::Sender<Vec<u8>>,
    ) {
        loop {
            match network_management_jobs.recv().await {
                Err(_) => {
                    info!("router job queue worker cancelled");
                    break;
                }
                Ok(job) => {
                    match Self::handle_routing_packet(
                        job,
                        ip_v4_subnet.clone(),
                        ip_device_ids.clone(),
                        client_outgoing.clone(),
                        vnet_send.clone(),
                    )
                    .await
                    {
                        PacketHandled::Succ | PacketHandled::Cont => {}
                        PacketHandled::Stop => {
                            info!("router job queue worker cancelled");
                            break;
                        }
                    }
                }
            }
        }
    }
}
