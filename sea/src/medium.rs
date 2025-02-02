use std::collections::VecDeque;
use std::time::Duration;

use async_trait::async_trait;
use log::warn;

pub struct VnetDummy {}

#[async_trait]
impl crate::traits::VirtualEthernetTrait for VnetDummy {
    async fn reconfigure(
        &self,
        _net: Option<ipnet::IpNet>,
        _default_route: bool,
    ) -> Result<(), ()> {
        Ok(())
    }

    async fn get_network(&self) -> Option<ipnet::IpNet> {
        None
    }

    async fn get_default_route(&self) -> bool {
        false
    }
}

pub struct RadioDummy {}

#[async_trait]
impl crate::traits::RadioModuleTrait for RadioDummy {
    async fn reconfigure(&self, _chip_address: u8, _network_id: u8) -> Result<(), ()> {
        Ok(())
    }

    async fn get_chip_address(&self) -> u8 {
        0
    }

    async fn get_network_id(&self) -> u8 {
        0
    }
}

#[derive(Debug)]
pub struct Vlan {
    packets_send_queue: VecDeque<Vec<u8>>,
    sender: tokio::sync::broadcast::Sender<Vec<u8>>,
    receiver: tokio::sync::broadcast::Sender<Vec<u8>>,
    publisher_handle: Option<tokio::task::JoinHandle<()>>,
}

impl Vlan {
    pub fn new(get_from: tokio::sync::broadcast::Sender<Vec<u8>>) -> Self {
        Self {
            packets_send_queue: VecDeque::new(),
            sender: tokio::sync::broadcast::channel(1).0,
            receiver: get_from,
            publisher_handle: None,
        }
    }

    pub fn add_packet(&mut self, packet: Vec<u8>) {
        self.packets_send_queue.push_back(packet);
    }

    /// get a channel to send to the vlan
    pub fn register_client(&self) -> tokio::sync::broadcast::Receiver<Vec<u8>> {
        self.sender.subscribe()
    }

    pub fn run_loop(&mut self) {
        let mut general_rcv = self.receiver.subscribe();
        self.publisher_handle = Some(tokio::spawn(async move {
            loop {
                match general_rcv.recv().await {
                    Ok(_packet) => {
                        tokio::time::sleep(Duration::from_millis(1)).await;
                        // from the client into the void of the internet
                    }
                    Err(e) => {
                        warn!("Error while receiving packet: {:?}", e);
                    }
                };
            }
        }));
    }
}
