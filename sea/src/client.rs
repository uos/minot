use async_trait::async_trait;
use log::debug;
use log::{error, info, warn};
use rand::Rng;
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{Duration, Instant};
use tokio::sync::mpsc;
use tokio::sync::Mutex;

use crate::datapack_handler::{
    get_datapackets_from_datachunk, get_seq_num_from_datapacket, SequenceCollector,
    SequenceSenderHelper,
};
use crate::packet::{DataPacket, DataPacketType, DeviceID};
use crate::traits::{RadioModuleTrait, VirtualEthernetTrait};

pub enum ReceivePacket {
    Medium(Vec<u8>),
    Vlan(Vec<u8>),
}

// An address has the possibilities of 0..255, excluding NET_ID. All values above 250 are for temporary device IDs. TODO handle multiple clients with the same temporary device ID
fn generate_tmp_device_id() -> DeviceID {
    let mut rng = rand::thread_rng();
    let mut tmp_device_id =
        rng.gen_range(crate::constants::TMP_DEVICE_ID_MIN..crate::constants::TMP_DEVICE_ID_MAX);
    while tmp_device_id == crate::constants::NET_ID {
        tmp_device_id =
            rng.gen_range(crate::constants::TMP_DEVICE_ID_MIN..crate::constants::TMP_DEVICE_ID_MAX);
    }
    tmp_device_id
}

#[derive(Debug)]
pub struct Client {
    ipv4: Arc<Mutex<Option<[u8; 4]>>>,
    static_device_id: Arc<Mutex<DeviceID>>,

    router_device_id: DeviceID,

    pub outgoing: broadcast::Sender<DataPacket>, // for rescheduling
    router_queue: Option<broadcast::Sender<crate::router::RouterRequest>>,
    pong_received: Arc<Mutex<Option<bool>>>,
    // For each destinationbyte one sequence sender helper if there is an active sequence being sent
    sending_collection: Arc<Mutex<Option<HashMap<u8, SequenceSenderHelper>>>>,
    // For each sourcebyte one sequence collector if there is an active sequence being received
    receiving_collection: Arc<Mutex<Option<HashMap<u8, SequenceCollector>>>>,
}

use tokio::sync::broadcast;

#[async_trait]
impl crate::traits::ClientTrait for Client {
    fn new() -> Self {
        let (outgoing, _) = broadcast::channel(16);
        Client {
            pong_received: Arc::new(Mutex::new(Some(false))),
            ipv4: Arc::new(Mutex::new(None)),
            static_device_id: Arc::new(Mutex::new(generate_tmp_device_id())),
            router_device_id: crate::constants::NET_ID,
            outgoing,
            router_queue: None,
            sending_collection: Arc::new(Mutex::new(Some(HashMap::new()))),
            receiving_collection: Arc::new(Mutex::new(Some(HashMap::new()))),
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
            mpsc::channel::<(Option<ipnet::IpNet>, bool)>(10);
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

    async fn connect(&self) -> Result<String, ()> {
        {
            if self.ipv4.lock().await.is_some() {
                return Err(());
            }
        }

        self.connect_to_network().await;
        Ok("todo".to_string())
    }

    async fn disconnect(&self) -> Result<(), ()> {
        self.disconnect_from_network().await;
        Ok(())
    }

    async fn ping(&self, peer_address: u8) -> Result<Duration, ()> {
        let start = Instant::now();

        let ping_packet = DataPacket::new(
            self.get_device_id().await,
            peer_address,
            DataPacketType::Ping,
        );

        match self.outgoing.send(ping_packet) {
            Ok(_) => {
                loop {
                    if self.pong_received.lock().await.unwrap() == false {
                        tokio::time::sleep(Duration::from_micros(10)).await;
                    } else {
                        // If we are here a Pong was received
                        // TODO Timeout ??
                        break;
                    }
                }
            }
            Err(_) => {
                error!("Failed to enqueue Ping packet");
            }
        }

        // Send Duration
        Ok(start.elapsed())
    }
}

#[derive(Debug)]
pub enum PacketHandled {
    Succ,
    Cont,
    Stop,
}

impl Client {
    pub async fn set_as_router_client(
        &mut self,
        router_queue: broadcast::Sender<crate::router::RouterRequest>,
    ) {
        let mut static_device_id = self.static_device_id.lock().await;
        *static_device_id = self.router_device_id;

        self.router_queue = Some(router_queue);
    }

    pub async fn get_device_id(&self) -> DeviceID {
        *self.static_device_id.lock().await
    }

    pub async fn get_ipv4(&self) -> Option<[u8; 4]> {
        *self.ipv4.lock().await
    }

    pub fn enqueue_packet(&self, packet: DataPacket) -> Result<(), ()> {
        match self.outgoing.send(packet) {
            Ok(_) => Ok(()),
            Err(_) => Err(()),
        }
    }

    pub async fn run_init(
        &self,
        vnet_send: broadcast::Sender<Vec<u8>>,
        vnet_rcv: broadcast::Receiver<Vec<u8>>,
        radio_send: broadcast::Sender<Vec<u8>>,
        radio_rcv: broadcast::Receiver<Vec<u8>>,
        radio_reconfigure_sender: mpsc::Sender<(u8, u8)>,
        vnet_reconfigure_sender: mpsc::Sender<(Option<ipnet::IpNet>, bool)>,
    ) -> () {
        radio_reconfigure_sender
            .send((self.get_device_id().await, crate::constants::NET_ID))
            .await
            .expect("Failed to send reconfigure message to radio");
        let to_ack = Arc::new(Mutex::new(Vec::new()));

        let (receiving_sender, receiving_receiver) = mpsc::channel::<ReceivePacket>(100);

        let receiving_worker_router_queue = self.router_queue.clone();

        let rcv_minion_vnet_snd = vnet_send.clone();
        let rcv_minion_ackq = to_ack.clone();
        let rcv_minion_ipv4 = self.ipv4.clone();
        let rcv_minion_device_id = self.static_device_id.clone();
        let rcv_minion_outgoing_sender = self.outgoing.clone();
        let rcv_minion_pong_received = self.pong_received.clone();
        let rcv_minion_receiving_collection = self.receiving_collection.clone();
        let rcv_minion_sending_collection = self.sending_collection.clone();

        tokio::task::spawn(async move {
            Self::receiving_worker(
                rcv_minion_vnet_snd,
                rcv_minion_ackq,
                rcv_minion_ipv4,
                receiving_receiver,
                receiving_worker_router_queue,
                rcv_minion_device_id,
                rcv_minion_outgoing_sender,
                rcv_minion_pong_received,
                rcv_minion_sending_collection,
                rcv_minion_receiving_collection,
                vnet_reconfigure_sender,
                radio_reconfigure_sender,
            )
            .await;
        });

        let snd_minion_outgoing = self.outgoing.clone();
        let snd_minion_outgoing_receiver = self.outgoing.subscribe();
        let snd_minion_to_ack = to_ack.clone();
        let snd_minion_ipv4 = self.ipv4.clone();
        let snd_minion_medium = radio_send.clone();
        let snd_minion_pong_received = self.pong_received.clone();
        let snd_minion_sending_collection = self.sending_collection.clone();
        tokio::task::spawn(async move {
            Self::sending_worker(
                snd_minion_outgoing_receiver,
                snd_minion_outgoing,
                snd_minion_medium,
                snd_minion_to_ack,
                snd_minion_ipv4,
                snd_minion_pong_received,
                snd_minion_sending_collection,
            )
            .await;
        });

        let mprod_minion_sender = receiving_sender.clone();

        tokio::task::spawn(async move {
            Self::receiver_medium_producer(radio_rcv, mprod_minion_sender).await;
        });

        let vprod_minion_sender = receiving_sender.clone();
        let vprod_minion_ipv4 = self.ipv4.clone();
        tokio::task::spawn(async move {
            Self::receiver_vlan_producer(vnet_rcv, vprod_minion_sender, vprod_minion_ipv4).await;
        });
    }

    pub async fn connect_to_network(&self) {
        let tmp_device_id = { self.static_device_id.lock().await.clone() };

        match self.outgoing.send(DataPacket::new(
            tmp_device_id,
            self.router_device_id,
            DataPacketType::ConnectRequest,
        )) {
            Ok(_) => {
                // wait for connection
                loop {
                    if self.ipv4.lock().await.is_none() {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    } else {
                        break;
                    }
                }
            }
            Err(e) => {
                error!("Failed to enqueue connect packet: {:?}", e);
            }
        }
    }

    pub async fn disconnect_from_network(&self) {
        let device_id = { self.static_device_id.lock().await.clone() };
        match self.outgoing.send(DataPacket::new(
            device_id,
            self.router_device_id,
            DataPacketType::DisconnectRequest,
        )) {
            Ok(_) => {
                // wait for disconnect
                loop {
                    if self.ipv4.lock().await.is_some() {
                        tokio::time::sleep(Duration::from_millis(5)).await;
                    } else {
                        break;
                    }
                }
            }
            Err(e) => {
                error!("Failed to enqueue disconnect packet: {:?}", e);
            }
        }
    }

    async fn wait_for_ack(destination: DeviceID, to_ack: &Mutex<Vec<DeviceID>>) -> bool {
        {
            to_ack.lock().await.push(destination);
        }

        let check_interval = Duration::from_millis(2);
        let timeout_duration = Duration::from_millis(crate::constants::CLIENT_TIMEOUT);
        let mut timeout = Duration::from_millis(0);
        while timeout < timeout_duration {
            tokio::time::sleep(check_interval).await;
            timeout += check_interval;

            let received = { !to_ack.lock().await.contains(&destination) };

            if received {
                return true;
            }
        }

        false
    }

    async fn reschedule_with_random_wait_timeout(
        packet: DataPacket,
        queue: &mut broadcast::Sender<DataPacket>,
    ) {
        const MAX_REQUEUE_TIME: u64 = crate::constants::CLIENT_TIMEOUT + 200; // ms

        match queue.send(packet) {
            Ok(_) => {}
            Err(_) => {
                error!("Failed to requeue packet");
            }
        }

        // TODO rand from to

        let timeout = Duration::from_millis(
            rand::random::<u64>() % (MAX_REQUEUE_TIME - crate::constants::CLIENT_TIMEOUT)
                + crate::constants::CLIENT_TIMEOUT,
        );
        tokio::time::sleep(timeout).await;
    }

    async fn wait_ack_else_reschedule(
        packet: DataPacket,
        queue: &mut broadcast::Sender<DataPacket>,
        to_ack: &Mutex<Vec<DeviceID>>,
    ) {
        let succ: bool = Self::wait_for_ack(packet.destination, to_ack).await;
        if !succ {
            info!("packet sent, but no ack received, assuming busy medium, rescheduling");
            Self::reschedule_with_random_wait_timeout(packet, queue).await;
        }
    }

    // --- worker functions ---

    pub async fn receiver_medium_producer(
        mut radio_rcv: broadcast::Receiver<Vec<u8>>,
        receive_queue: mpsc::Sender<ReceivePacket>,
    ) {
        loop {
            match radio_rcv.recv().await {
                Ok(data) => match receive_queue.send(ReceivePacket::Medium(data)).await {
                    Ok(_) => {}
                    Err(_) => {
                        error!("Failed to enqueue packet");
                    }
                },
                Err(err) => match err {
                    broadcast::error::RecvError::Closed => {
                        debug!("medium producer rcv channel closed down");
                        break;
                    }
                    broadcast::error::RecvError::Lagged(missed) => {
                        error!(
                            "medium producer rcv channel lags behind, missed {:?} packets",
                            missed
                        );
                        continue;
                    }
                },
            }
        }
    }

    pub async fn receiver_vlan_producer(
        mut vnet_rcv: broadcast::Receiver<Vec<u8>>,
        receive_queue: mpsc::Sender<ReceivePacket>,
        ipv4: Arc<Mutex<Option<[u8; 4]>>>,
    ) {
        loop {
            match vnet_rcv.recv().await {
                Ok(data) => {
                    // TODO chunking, encrypt data

                    // Add to sending queue

                    let connected_to_network = { ipv4.lock().await.is_some() };
                    if !connected_to_network {
                        warn!("received packet from vnet, but not connected to network, ignoring");
                        continue;
                    }

                    match receive_queue.send(ReceivePacket::Vlan(data)).await {
                        Ok(_) => {}
                        Err(e) => {
                            error!("Failed to enqueue packet: {:?}", e);
                        }
                    }
                }
                Err(err) => match err {
                    broadcast::error::RecvError::Closed => {
                        debug!("vlan producer rcv channel closed down");
                        break;
                    }
                    broadcast::error::RecvError::Lagged(missed) => {
                        error!(
                            "vlan producer rcv channel lags behind, missed {:?} packets",
                            missed
                        );
                        continue;
                    }
                },
            }
        }
    }

    async fn handle_rcv(
        receive_type: ReceivePacket,
        g_device_id: Arc<Mutex<DeviceID>>,
        router_queue: Option<broadcast::Sender<crate::router::RouterRequest>>,
        vnet_send: broadcast::Sender<Vec<u8>>,
        to_ack: Arc<Mutex<Vec<DeviceID>>>,
        ipv4: Arc<Mutex<Option<[u8; 4]>>>,
        sending_queue: broadcast::Sender<DataPacket>,
        pong_received: Arc<Mutex<Option<bool>>>,
        sending_collection: Arc<Mutex<Option<HashMap<u8, SequenceSenderHelper>>>>,
        receiving_collection: Arc<Mutex<Option<HashMap<u8, SequenceCollector>>>>,
        net_reconfigure_sender: mpsc::Sender<(Option<ipnet::IpNet>, bool)>, // ip, receive all traffic
        radio_reconfigure_sender: mpsc::Sender<(u8, u8)>, // device_id, router device_id
    ) -> PacketHandled {
        let unp_device_id = { g_device_id.lock().await.clone() };

        match receive_type {
            ReceivePacket::Medium(data) => {
                let packet = DataPacket::from(&data);

                if packet.destination != unp_device_id {
                    info!(
                        "[RCV@{:?}] ignoring packet with destination: {:?}",
                        unp_device_id, packet.destination
                    );
                    return PacketHandled::Cont;
                }

                info!("[RCV@{:?}] got packet! {:?}", unp_device_id, packet);
                match packet.payload {
                    DataPacketType::ConnectRequest => match router_queue.clone() {
                        Some(router_queue) => {
                            match router_queue
                                .send(crate::router::RouterRequest::ConnectRequest(packet.source))
                            {
                                Ok(_) => {
                                    sending_queue
                                        .send(DataPacket::new(
                                            unp_device_id,
                                            packet.source,
                                            DataPacketType::Ack,
                                        ))
                                        .expect("reschedule broken");

                                    info!(
                                        "[RCV@{:?}] is connect request, forwarding to router",
                                        unp_device_id
                                    );
                                }
                                Err(_) => {
                                    warn!("router queue closed, ignoring packet");
                                }
                            }
                        }
                        None => {
                            warn!(
                                "[RCV@{:?}] connect request received from client, but router queue is not set.", unp_device_id
                            );
                        }
                    },
                    DataPacketType::DisconnectRequest => {
                        match router_queue.clone() {
                            Some(router_queue) => {
                                match router_queue.send(
                                    crate::router::RouterRequest::DisconnectRequest(packet.source),
                                ) {
                                    Ok(_) => {}
                                    Err(_) => {
                                        warn!("router queue closed, ignoring packet");
                                    }
                                }
                            }
                            None => {
                                // ignore disconnect request
                                warn!("disconnect request received in client, ignoring");
                            }
                        }
                    }
                    DataPacketType::Raw(data) => {
                        match router_queue.clone() {
                            Some(router_queue) => {
                                match router_queue.send(
                                    crate::router::RouterRequest::PassThroughRequest(
                                        // send to vlan
                                        unp_device_id,
                                        data,
                                    ),
                                ) {
                                    Ok(_) => {}
                                    Err(_) => {
                                        warn!("router queue closed, ignoring packet");
                                    }
                                }
                            }
                            None => match vnet_send.send(data) {
                                Ok(_) => {}
                                Err(_) => {
                                    warn!("vnet closed, ignoring packet");
                                }
                            },
                        };
                    }
                    DataPacketType::Ack => {
                        let mut to_ack = to_ack.lock().await;
                        let pos = to_ack.iter().position(|x| *x == packet.source);
                        match pos {
                            Some(pos) => {
                                to_ack.remove(pos);
                            }
                            None => {
                                warn!("ack received from client, but no packet to ack, ignoring");
                            }
                        }
                    }
                    DataPacketType::Nack(_seq_num) => {
                        info!("nack received from client");
                        // Get the sequence Number and source
                        // Alread present

                        // Get Corresponding SequenceSenderHelper
                        let mut sender_helper_arcmut = sending_collection.lock().await;
                        let sender_helper_asmut = sender_helper_arcmut.as_mut().unwrap();
                        let sender_helper = sender_helper_asmut.get_mut(&packet.source).unwrap();
                        (*sender_helper).process_nack(packet.clone()).await; // Takes seq num into account

                        // Get out the right package to resend
                        let mut resend_pack = (*sender_helper).get_next_packet().await;

                        while resend_pack.is_some() {
                            // Send the Package again
                            match sending_queue.send(resend_pack.unwrap()) {
                                Ok(_) => {
                                    info!(
                                        "[RCV@{:?}] Resent packet to {:?}",
                                        unp_device_id, packet.source
                                    );
                                }
                                Err(_) => {
                                    error!("Failed to enqueue packet");
                                }
                            }
                            resend_pack = (*sender_helper).get_next_packet().await;
                        }
                    }
                    DataPacketType::ConnectResponse {
                        ip,
                        prefix_len,
                        device_id,
                    } => match router_queue.clone() {
                        Some(_) => {
                            warn!("connect response received inside router, ignoring");
                        }
                        None => {
                            {
                                let mut ipv4 = ipv4.lock().await;
                                *ipv4 = Some(ip);
                                let mut static_device_id = g_device_id.lock().await;
                                *static_device_id = device_id;
                            }

                            let conf_ip: ipnet::IpNet =
                                format!("{}.{}.{}.{}/{}", ip[0], ip[1], ip[2], ip[3], prefix_len)
                                    .parse()
                                    .expect("failed to parse ip");
                            net_reconfigure_sender
                                .send((Some(conf_ip), false))
                                .await
                                .expect("failed to send net reconfigure");
                            radio_reconfigure_sender
                                .send((device_id, packet.source))
                                .await
                                .expect("failed to send radio reconfigure");

                            sending_queue
                                .send(DataPacket::new(
                                    unp_device_id,
                                    packet.source,
                                    DataPacketType::Ack,
                                ))
                                .expect("reschedule broken");
                        }
                    },
                    DataPacketType::Ping => {
                        info!(
                            "[RCV@{:?}] Received ping packet from {:?}",
                            unp_device_id, packet.source
                        );
                        let p2: DataPacket =
                            DataPacket::new(unp_device_id, packet.source, DataPacketType::Pong);

                        match sending_queue.send(p2) {
                            Ok(_) => {
                                info!(
                                    "[RCV@{:?}] Sent pong packet to {:?}",
                                    unp_device_id, packet.source
                                );
                            }
                            Err(_) => {
                                error!("Failed to enqueue packet");
                            }
                        }
                    }
                    DataPacketType::Pong => {
                        info!(
                            "[RCV@{:?}] Received pong packet from {:?}",
                            unp_device_id, packet.source
                        );
                        // Set Pong received to True
                        let mut pong_received = pong_received.lock().await;
                        *pong_received = Some(true);
                    }
                    DataPacketType::RejoinRequest => {
                        let p2: DataPacket = DataPacket::new(
                            unp_device_id,
                            packet.source,
                            DataPacketType::ConnectRequest,
                        );

                        match sending_queue.send(p2) {
                            Ok(_) => {}
                            Err(_) => {
                                error!("Failed to enqueue packet");
                            }
                        }
                    }
                    DataPacketType::Data(_) => {
                        // Get sequence number
                        let seq_num = get_seq_num_from_datapacket(packet.clone());

                        if seq_num == 1 {
                            // If first packet, add sequencepacketcollector to client with it and send ack
                            let seq_col = SequenceCollector::new(packet.clone()).await;
                            let mut recv_col_arcmut = receiving_collection.lock().await;
                            let recv_col_asmut = recv_col_arcmut.as_mut().unwrap();
                            (*recv_col_asmut).insert(packet.source, seq_col);

                            // Ack
                            let ack_packet: DataPacket =
                                DataPacket::new(unp_device_id, packet.source, DataPacketType::Ack);
                            match sending_queue.send(ack_packet) {
                                Ok(_) => {}
                                Err(_) => {
                                    error!("Failed to enqueue packet");
                                }
                            }
                        } else {
                            // If nth packet, add to sequence packetcollector
                            let mut recv_col_arcmut = receiving_collection.lock().await;
                            let recv_col_asmut = recv_col_arcmut.as_mut().unwrap();
                            let seq_col = recv_col_asmut.get_mut(&packet.source).unwrap();
                            (*seq_col).add_packet(packet.clone()).await;

                            // Check if it is expected number
                            // Increase expected num
                            let expected_num_arcmut = (*seq_col).expected_pack_num.lock().await;
                            let expected_num = expected_num_arcmut.unwrap();
                            if seq_num != expected_num {
                                // If not the expected Packet send nack
                                let nack_packet: DataPacket = DataPacket::new(
                                    unp_device_id,
                                    packet.source,
                                    DataPacketType::Nack(expected_num),
                                );
                                match sending_queue.send(nack_packet) {
                                    Ok(_) => {}
                                    Err(_) => {
                                        error!("Failed to enqueue packet");
                                    }
                                }
                            }
                        }

                        // Get the sequence collector
                        let mut recv_col_arcmut = receiving_collection.lock().await;
                        let recv_col_asmut = recv_col_arcmut.as_mut().unwrap();
                        let seq_col = recv_col_asmut.get_mut(&packet.source).unwrap();

                        // See if data are completely received
                        let recv_complete = (*seq_col).receive_complete.lock().await.unwrap();
                        if recv_complete {
                            // Ack
                            let ack_packet: DataPacket =
                                DataPacket::new(unp_device_id, packet.source, DataPacketType::Ack);
                            match sending_queue.send(ack_packet) {
                                Ok(_) => {}
                                Err(_) => {
                                    error!("Failed to enqueue packet");
                                }
                            }

                            let full_data = (*seq_col).get_data_of_seq().await;

                            // Send data on
                            match router_queue.clone() {
                                Some(router_queue) => {
                                    match router_queue.send(
                                        crate::router::RouterRequest::PassThroughRequest(
                                            // send to vlan
                                            unp_device_id,
                                            full_data,
                                        ),
                                    ) {
                                        Ok(_) => {}
                                        Err(_) => {
                                            warn!("router queue closed, ignoring packet");
                                        }
                                    }
                                }
                                None => match vnet_send.send(full_data) {
                                    Ok(_) => {}
                                    Err(_) => {
                                        warn!("vnet closed, ignoring packet");
                                    }
                                },
                            };

                            // Dispose of Sequence Collector
                            (*recv_col_asmut).remove(&packet.source);
                        }
                    }
                }
            }
            ReceivePacket::Vlan(data) => match router_queue.clone() {
                None => {
                    let send_packet = DataPacket::new(
                        unp_device_id,
                        crate::constants::NET_ID,
                        DataPacketType::Raw(data),
                    );
                    match sending_queue.send(send_packet) {
                        Ok(_) => {}
                        Err(_) => {
                            warn!("sending queue closed, ignoring packet");
                        }
                    };
                }
                Some(router_queue) => {
                    match router_queue.send(crate::router::RouterRequest::PassThroughRequest(
                        crate::constants::NET_ID,
                        data,
                    )) {
                        Ok(_) => {}
                        Err(_) => {
                            warn!("router queue closed, ignoring packet");
                        }
                    }
                }
            },
        };

        PacketHandled::Cont
    }

    /// Handling packets coming from the network
    pub async fn receiving_worker(
        vnet_send: broadcast::Sender<Vec<u8>>,
        to_ack: Arc<Mutex<Vec<DeviceID>>>,
        ipv4: Arc<Mutex<Option<[u8; 4]>>>,
        mut receive_queue: mpsc::Receiver<ReceivePacket>,
        router_queue: Option<broadcast::Sender<crate::router::RouterRequest>>,
        device_id: Arc<Mutex<DeviceID>>,
        sending_queue: broadcast::Sender<DataPacket>,
        pong_received: Arc<Mutex<Option<bool>>>,
        sending_collection: Arc<Mutex<Option<HashMap<u8, SequenceSenderHelper>>>>,
        receiving_collection: Arc<Mutex<Option<HashMap<u8, SequenceCollector>>>>,
        net_reconfigure_sender: mpsc::Sender<(Option<ipnet::IpNet>, bool)>,
        radio_reconfigure_sender: mpsc::Sender<(u8, u8)>,
    ) {
        loop {
            match receive_queue.recv().await {
                Some(received) => {
                    match Self::handle_rcv(
                        received,
                        device_id.clone(),
                        router_queue.clone(),
                        vnet_send.clone(),
                        to_ack.clone(),
                        ipv4.clone(),
                        sending_queue.clone(),
                        pong_received.clone(),
                        sending_collection.clone(),
                        receiving_collection.clone(),
                        net_reconfigure_sender.clone(),
                        radio_reconfigure_sender.clone(),
                    )
                    .await
                    {
                        PacketHandled::Cont | PacketHandled::Succ => {}
                        PacketHandled::Stop => {
                            break;
                        }
                    }
                }
                None => {
                    info!("receiving worker got None");
                    break;
                }
            }
        }
    }

    async fn handle_sent(
        packet: DataPacket,
        medium_send: &broadcast::Sender<Vec<u8>>,
        ipv4: Arc<Mutex<Option<[u8; 4]>>>,
        to_ack: Arc<Mutex<Vec<DeviceID>>>,
        mut outgoing_sender: broadcast::Sender<DataPacket>,
        pong_received: Arc<Mutex<Option<bool>>>,
        sending_collection: Arc<Mutex<Option<HashMap<u8, SequenceSenderHelper>>>>,
    ) -> PacketHandled {
        let buffer: Vec<u8> = packet.clone().into();
        let device_id = packet.source;
        let destination = packet.clone().destination;

        // If it is raw packet, then transform it into one or multiple Datapackets depending on legth

        match packet.clone().payload {
            DataPacketType::Raw(data) => {
                // Split Data into smaller packets
                let datapackets =
                    get_datapackets_from_datachunk(packet.source, packet.destination, data);

                // Add Senderhelper for this sequence
                let mut sen_hel = SequenceSenderHelper::new(datapackets.clone()).await;
                let mut sen_col_arcmut = sending_collection.lock().await;
                let sen_col_asmut = sen_col_arcmut.as_mut().unwrap();

                // Add first Datapacket
                let first_pack = sen_hel.get_next_packet().await;

                // Check if currently a sequence is already being sent to the Destination Address
                // If so, wait and requeue the raw packet
                if (*sen_col_asmut).get(&destination).is_some() {
                    tokio::time::sleep(Duration::from_micros(1000)).await;
                    match outgoing_sender.send(packet.clone()) {
                        Ok(_) => {
                            info!(
                                "[RCV@{:?}] Sent data packet to {:?}",
                                device_id, packet.destination
                            );
                        }
                        Err(_) => {
                            error!("Failed to enqueue packet");
                        }
                    }
                    return PacketHandled::Succ;
                }

                // Otherwise add new Packethandler
                (*sen_col_asmut).insert(packet.destination, sen_hel);

                match outgoing_sender.send(first_pack.clone().unwrap()) {
                    Ok(_) => {
                        info!(
                            "[RCV@{:?}] Sent data packet to {:?}",
                            device_id, packet.destination
                        );
                    }
                    Err(_) => {
                        error!("Failed to enqueue packet");
                    }
                }
                return PacketHandled::Succ;
            }
            _ => {
                // Nothing
            }
        }

        // Actually send given packet
        match medium_send.send(buffer) {
            Ok(_) => {
                match packet.payload {
                    DataPacketType::Nack(_seq_num) => {
                        // Nothing to do I believe
                    }
                    DataPacketType::Ack => {} // no ack for ack
                    DataPacketType::ConnectRequest => {
                        info!(
                            "[SND@{:?}] connect request to router {:?}",
                            device_id, packet.destination
                        );

                        // -- router acks request --
                        Self::wait_ack_else_reschedule(
                            packet,
                            &mut outgoing_sender,
                            to_ack.as_ref(),
                        )
                        .await;
                    }
                    DataPacketType::ConnectResponse { .. } => {
                        info!(
                            "[SND@{:?}] connect response to {:?}",
                            device_id, packet.destination
                        );
                        Self::wait_ack_else_reschedule(
                            packet,
                            &mut outgoing_sender,
                            to_ack.as_ref(),
                        )
                        .await;
                    }
                    DataPacketType::DisconnectRequest => {
                        Self::wait_ack_else_reschedule(
                            packet,
                            &mut outgoing_sender,
                            to_ack.as_ref(),
                        )
                        .await;

                        // delete own ip address
                        {
                            ipv4.lock().await.take();
                        }
                    }
                    DataPacketType::Raw(_) => {
                        Self::wait_ack_else_reschedule(
                            packet,
                            &mut outgoing_sender,
                            to_ack.as_ref(),
                        )
                        .await;
                    }
                    DataPacketType::Ping => {
                        info!(
                            "[SND@{:?}] Ping packet to {:?}",
                            device_id, packet.destination
                        );
                        pong_received.lock().await.take(); // reset pong received
                    }
                    DataPacketType::Pong => {
                        info!(
                            "[SND@{:?}] Pong packet to {:?}",
                            device_id, packet.destination
                        );
                    }
                    DataPacketType::RejoinRequest => {
                        info!(
                            "[SND@{:?}] Rejoin request to {:?}",
                            device_id, packet.destination
                        );
                    }
                    DataPacketType::Data(_) => {
                        // Wait for Ack if first or last packet and set corresponding values
                        let seq_num = get_seq_num_from_datapacket(packet.clone());

                        let mut sender_helper_arcmut = sending_collection.lock().await;
                        let sender_helper_asmut = sender_helper_arcmut.as_mut().unwrap();
                        let sender_helper =
                            sender_helper_asmut.get_mut(&packet.destination).unwrap();
                        //(*sender_helper).process_nack(packet.clone()).await; // Takes seq num into account

                        let total_num = (*sender_helper).num_packets_to_send;

                        if seq_num == 1 || seq_num == total_num {
                            Self::wait_ack_else_reschedule(
                                packet,
                                &mut outgoing_sender,
                                to_ack.as_ref(),
                            )
                            .await;

                            if seq_num == 1 {
                                (*sender_helper).set_first_acked(true).await;
                                // Add Remaining Packets

                                // Get Corresponding SequenceSenderHelper
                                let mut sender_helper_arcmut = sending_collection.lock().await;
                                let sender_helper_asmut = sender_helper_arcmut.as_mut().unwrap();
                                let sender_helper =
                                    sender_helper_asmut.get_mut(&device_id).unwrap();

                                // Get out the right package to resend
                                let mut resend_pack = (*sender_helper).get_next_packet().await;

                                while resend_pack.is_some() {
                                    // Send the Package again
                                    match outgoing_sender.send(resend_pack.unwrap()) {
                                        Ok(_) => {
                                            info!(
                                                "[RCV@{:?}] Resent packet to {:?}",
                                                device_id, destination
                                            );
                                        }
                                        Err(_) => {
                                            error!("Failed to enqueue packet");
                                        }
                                    }
                                    resend_pack = (*sender_helper).get_next_packet().await;
                                }
                            } else {
                                (*sender_helper).set_last_acked(true).await;

                                // Dispose Sender Helper
                                (*sender_helper_asmut).remove(&destination);
                            }
                        }
                    }
                }
                PacketHandled::Succ
            }
            Err(_) => {
                warn!("tried to send in client, but no active receivers");
                return PacketHandled::Succ;
            }
        }
    }

    /// Handling the outgoing packets into the network
    pub async fn sending_worker(
        mut outgoing: broadcast::Receiver<DataPacket>,
        outgoing_sender: broadcast::Sender<DataPacket>,
        medium_send: broadcast::Sender<Vec<u8>>,
        to_ack: Arc<Mutex<Vec<DeviceID>>>,
        ipv4: Arc<Mutex<Option<[u8; 4]>>>,
        pong_received: Arc<Mutex<Option<bool>>>,
        sending_collection: Arc<Mutex<Option<HashMap<u8, SequenceSenderHelper>>>>,
    ) {
        loop {
            match outgoing.recv().await {
                Ok(packet) => {
                    match Self::handle_sent(
                        packet,
                        &medium_send,
                        ipv4.clone(),
                        to_ack.clone(),
                        outgoing_sender.clone(),
                        pong_received.clone(),
                        sending_collection.clone(),
                    )
                    .await
                    {
                        PacketHandled::Cont | PacketHandled::Succ => {}
                        PacketHandled::Stop => {
                            break;
                        }
                    }
                }
                Err(e) => match e {
                    broadcast::error::RecvError::Closed => {
                        debug!("medium producer rcv channel closed down");
                        break;
                    }
                    broadcast::error::RecvError::Lagged(missed) => {
                        error!(
                            "medium producer rcv channel lags behind, missed {:?} packets",
                            missed
                        );
                        continue;
                    }
                },
            }
        }
    }
}
