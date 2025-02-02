use std::{cmp, collections::HashMap, sync::Arc};
use tokio::sync::Mutex;

use crate::packet::{DataPacket, DataPacketType, DeviceID};

// Paketlänenwerte
pub const MAX_PACK_LENGTH: u8 = 66;
pub const COMPLETE_HEADER_LENGTH: u8 = 1 + 1 + 1 + 1;
pub const PAYLOAD_SIZE: u8 = MAX_PACK_LENGTH - COMPLETE_HEADER_LENGTH;
pub const SEQNUM_SIZE: u8 = 1;
pub const PAYLOAD_SIZE_FIRST_SEQ: u8 = PAYLOAD_SIZE - 2 * SEQNUM_SIZE;
pub const PAYLOAD_SIZE_NTH_SEQ: u8 = PAYLOAD_SIZE - 1 * SEQNUM_SIZE;

// Get the max num of all sequence packets from the first packet
pub fn get_max_num_from_datapacket(packet: DataPacket) -> u8 {
    match packet.payload {
        DataPacketType::Data(data) => {
            // Check if we have first packet
            let seq_num = data[0];
            if seq_num != 1 {
                panic!(
                    "Given Datapacket is not the first one but has num {:?}",
                    seq_num
                );
            }
            let total_pack_num = data[usize::from(0 + SEQNUM_SIZE)];
            return total_pack_num;
        }
        _ => {
            panic!("Given Datapacket is of wrong type");
        }
    }
}

// Get the sequence number
pub fn get_seq_num_from_datapacket(packet: DataPacket) -> u8 {
    match packet.payload {
        DataPacketType::Data(data) => {
            // Check if we have first packet
            let seq_num = data[0];
            return seq_num;
        }
        _ => {
            panic!("Given Datapacket is of wrong type");
        }
    }
}

pub async fn get_seq_num_from_nack(nack_packet: DataPacket) -> u8 {
    match nack_packet.payload {
        DataPacketType::Nack(seq_num) => {
            return seq_num;
        }
        _ => {
            panic!("Given Datapacket is of wrong type");
        }
    }
}

// Get the data part from a given data datapacket
pub fn get_data_from_datapacket(packet: DataPacket) -> Vec<u8> {
    match packet.payload {
        DataPacketType::Data(data) => {
            // Check if we have first packet
            let seq_num = data[0];
            if seq_num == 1 {
                // Max number is in packet,  PAYLOAD_SIZE_FIRST_SEQ many bytes
                return data[usize::from(0 + 2 * SEQNUM_SIZE)..].to_vec();
            } else {
                // Only jump over sec number, PAYLOAD_SIZE_NTH_SEQ many bytes
                return data[usize::from(0 + SEQNUM_SIZE)..].to_vec();
            }
        }
        _ => {
            panic!("Given Datapacket is of wrong type");
        }
    }
}

// Given a datachunk, partition it and make multiple packets from it
pub fn get_datapackets_from_datachunk(
    source: DeviceID,
    destination: DeviceID,
    data: Vec<u8>,
) -> Vec<DataPacket> {
    // Collection of returned packets
    let mut packets = Vec::new();
    // If data short enough it can be put into one packet
    if data.len() <= usize::from(PAYLOAD_SIZE_FIRST_SEQ) {
        // If data short enough it can be put into one packet
        let data_packet = build_datapacket_data(source, destination, 1, 1, data);
        packets.push(data_packet);
    } else {
        // If data does not fit into one package partition into multiple packets
        // Calculate number of total packets
        let remaining_len: usize = data.len() - usize::from(PAYLOAD_SIZE_FIRST_SEQ);
        let num_packets = 1 + ceil_div(remaining_len, usize::from(PAYLOAD_SIZE_NTH_SEQ));

        // Make first Datapacket
        let first_slice = data[0..usize::from(PAYLOAD_SIZE_FIRST_SEQ - 1)].to_vec();
        let first_data_packet =
            build_datapacket_data(source, destination, 1, num_packets as u8, first_slice);
        packets.push(first_data_packet);

        //Make further Datapackets
        let mut cut_index = usize::from(PAYLOAD_SIZE_FIRST_SEQ) - 1;
        let mut seq_num = 2;

        while cut_index < data.len() {
            // Slice Date
            let end_index = cmp::min(cut_index + usize::from(PAYLOAD_SIZE_NTH_SEQ), data.len());
            let cur_slice = data[cut_index..end_index].to_vec();
            // Add Packet
            let nth_data_packet =
                build_datapacket_data(source, destination, seq_num, num_packets as u8, cur_slice);
            packets.push(nth_data_packet);
            // Increase indices
            seq_num = seq_num + 1;
            cut_index = end_index;
        }
    }
    return packets;
}

// Divide two integers and round up
pub fn ceil_div(a: usize, b: usize) -> usize {
    (a + b - 1) / b
}

pub fn build_datapacket_data(
    source: DeviceID,
    destination: DeviceID,
    seq_num: u8,
    max_num: u8,
    data: Vec<u8>,
) -> DataPacket {
    // Check if input is correct
    if (seq_num == 1 && data.len() > usize::from(PAYLOAD_SIZE_FIRST_SEQ))
        || (seq_num > 1 && data.len() > usize::from(PAYLOAD_SIZE_NTH_SEQ))
    {
        panic!(
            "Data too long for seq num {:?} with {:?} bytes",
            seq_num,
            data.len()
        );
    }

    // Build payload
    let mut payload = Vec::new();
    // Add Seqnums
    payload.push(seq_num);
    if seq_num == 1 {
        payload.push(max_num);
    }

    // Add Data
    payload.extend_from_slice(&data);

    // Build Packet
    let data_packet = DataPacket::new(source, destination, DataPacketType::Data(payload));
    // Return
    data_packet
}

// Class for handling a sequence of datapckets
pub struct DatapacketHandler {
    pub source: DeviceID,   // Read only
    pub total_pack_num: u8, // Read only
    pub receive_complete: Arc<Mutex<Option<bool>>>,
    pub next_expected_num: Arc<Mutex<Option<u8>>>, // Number of the next expected packet
    pub data: Arc<Mutex<Option<Vec<u8>>>>,
}

impl DatapacketHandler {
    pub async fn new(first_seq_packet: DataPacket) -> Self {
        // Implement use of first packet
        // Check packettype
        match first_seq_packet.clone().payload {
            DataPacketType::Data(_data) => {
                // Packet is okay
            }
            _ => {
                panic!("Given the Datapackethandler a packet of a Type that is not Data");
            }
        }

        // Read Max number
        let total_pack_num = get_max_num_from_datapacket(first_seq_packet.clone());

        // Set Current number
        let next_expected_num = Arc::new(Mutex::new(Some(2)));

        // Set received
        let receive_complete = Arc::new(Mutex::new(Some(false)));
        if total_pack_num == 1 {
            let mut receive_complete_arcmut = receive_complete.lock().await;
            *receive_complete_arcmut = Some(true);
        }

        // Add Data
        let data = Arc::new(Mutex::new(Some(Vec::new())));
        {
            let mut data_arcmut = data.lock().await;
            let data_first_pack = get_data_from_datapacket(first_seq_packet.clone());
            (*data_arcmut)
                .as_mut()
                .unwrap()
                .extend_from_slice(&data_first_pack);
        }

        let source = first_seq_packet.clone().source;

        DatapacketHandler {
            source,
            total_pack_num,
            receive_complete,
            next_expected_num,
            data,
        }
    }

    // Use this function to add the packets until the packet is created
    pub async fn add_packet(&mut self, new_packet: DataPacket) {
        // Check if the given packet is the expected packet
        let new_pack_num = get_seq_num_from_datapacket(new_packet.clone());
        let mut own_num_arcmut = self.next_expected_num.lock().await;
        let own_expected_num = own_num_arcmut.unwrap();
        if new_pack_num != own_expected_num {
            panic!("Packet given to the packet handler wasnt the expected packet");
        }
        // Check source
        if new_packet.source != self.source {
            panic!("Given packet and packethandler have different sources");
        }
        // Packet okay
        // Get Data From this Packet
        let new_data = get_data_from_datapacket(new_packet);

        // Lock Data, increase number and append new data
        let mut data_arcmut = self.data.lock().await;
        *own_num_arcmut = Some(own_expected_num + 1);
        (*data_arcmut)
            .as_mut()
            .unwrap()
            .extend_from_slice(&new_data);
        // If enough packets received stop
        if own_expected_num == self.total_pack_num {
            let mut rec_comp_arcmut = self.receive_complete.lock().await;
            *rec_comp_arcmut = Some(true);
        }
    }
}

// Class for handling an open datapacket connection
#[derive(Debug)]
pub struct SequenceCollector {
    pub source: DeviceID,       // Read only
    pub num_packets_needed: u8, // Read only
    pub receive_complete: Arc<Mutex<Option<bool>>>,
    pub num_packs_collected: Arc<Mutex<Option<u8>>>,
    pub expected_pack_num: Arc<Mutex<Option<u8>>>,
    pub packet_collection: Arc<Mutex<Option<HashMap<u8, DataPacket>>>>,
}

impl SequenceCollector {
    pub async fn new(first_data_packet: DataPacket) -> Self {
        let source = first_data_packet.source;
        let num_packets_needed = get_max_num_from_datapacket(first_data_packet.clone());
        let seq_num = get_seq_num_from_datapacket(first_data_packet.clone());
        if seq_num != 1 {
            panic!("Given Datapacket is not first one");
        }

        let receive_complete = Arc::new(Mutex::new(Some(false)));
        if num_packets_needed == 1 {
            let mut receive_complete_arcmut = receive_complete.lock().await;
            *receive_complete_arcmut = Some(true);
        }

        let num_packs_collected = Arc::new(Mutex::new(Some(1)));
        let expected_pack_num = Arc::new(Mutex::new(Some(2)));

        let packet_collection = Arc::new(Mutex::new(Some(HashMap::new())));
        let first_index: u8 = 1;
        {
            let mut pack_coll_arcmut = packet_collection.lock().await;
            (*pack_coll_arcmut)
                .as_mut()
                .unwrap()
                .insert(first_index, first_data_packet);
        }

        SequenceCollector {
            source,
            num_packets_needed,
            receive_complete,
            num_packs_collected,
            expected_pack_num,
            packet_collection,
        }
    }

    // Add a packet
    pub async fn add_packet(&self, datapacket_with_data: DataPacket) {
        // get packet number
        let seq_num = get_seq_num_from_datapacket(datapacket_with_data.clone());

        // Increase expected num
        let mut expected_num_arcmut = self.expected_pack_num.lock().await;
        let expected_num = expected_num_arcmut.unwrap();
        if seq_num == expected_num {
            *expected_num_arcmut = Some(expected_num + 1);
        }

        let mut pack_coll_arcmut = self.packet_collection.lock().await;
        // Add if Packet not already Contained
        let packet_already_contained = (*pack_coll_arcmut).as_mut().unwrap().contains_key(&seq_num);
        if !packet_already_contained {
            (*pack_coll_arcmut)
                .as_mut()
                .unwrap()
                .insert(seq_num, datapacket_with_data);
            let mut num_collected_arcmut = self.num_packs_collected.lock().await;
            let curr_num = num_collected_arcmut.unwrap();
            *num_collected_arcmut = Some(curr_num + 1);

            // if enough packets collected, set recv all to true
            if self.num_packets_needed == curr_num + 1 {
                let mut receive_complete_arcmut = self.receive_complete.lock().await;
                *receive_complete_arcmut = Some(true);
            }
        }
    }

    // Have all Packets been received
    pub async fn all_packets_received(&self) -> bool {
        let received = self.receive_complete.lock().await.unwrap();
        received
    }

    pub async fn get_data_of_seq(&self) -> Vec<u8> {
        if !self.all_packets_received().await {
            panic!("Not all Packets have been received yet")
        }
        let mut pack_coll_arcmut = self.packet_collection.lock().await;

        // Get first packet and take it out
        let first_index: u8 = 1;
        let first_packet = (*pack_coll_arcmut)
            .as_mut()
            .unwrap()
            .get(&first_index)
            .unwrap();

        let mut handler: DatapacketHandler = DatapacketHandler::new(first_packet.clone()).await;

        //(*pack_coll_arcmut).as_mut().unwrap().remove(&first_index);

        for n in first_index + 1..self.num_packets_needed + 1 {
            let cur_packet = (*pack_coll_arcmut).as_mut().unwrap().get(&n).unwrap();
            handler.add_packet(cur_packet.clone()).await;
        }

        return handler.data.lock().await.as_ref().unwrap().clone();
    }
}

#[derive(Debug)]
pub struct SequenceSenderHelper {
    pub destination: DeviceID,           // Read only
    pub num_packets_to_send: u8,         // Read only
    pub counter: Arc<Mutex<Option<u8>>>, // Count up the packets sent, go back down if nack received, has number of next packet to send
    pub first_acked: Arc<Mutex<Option<bool>>>,
    pub last_acked: Arc<Mutex<Option<bool>>>,
    pub packet_collection: Arc<Mutex<Option<HashMap<u8, DataPacket>>>>,
}

pub async fn add_packet_to_collection(
    packet_collection: Arc<Mutex<Option<HashMap<u8, DataPacket>>>>,
    packet: DataPacket,
) {
    let pack_num = get_seq_num_from_datapacket(packet.clone());
    let mut pack_coll_arcmut = packet_collection.lock().await;
    (*pack_coll_arcmut)
        .as_mut()
        .unwrap()
        .insert(pack_num, packet.clone());
}

impl SequenceSenderHelper {
    pub async fn new(packets_to_send: Vec<DataPacket>) -> Self {
        //let source = packets_to_send.get(0).unwrap().source;
        let destination = packets_to_send.get(0).unwrap().destination;

        let num_packets_to_send = u8::try_from(packets_to_send.len()).unwrap();

        let counter = Arc::new(Mutex::new(Some(1)));

        let first_acked = Arc::new(Mutex::new(Some(false)));

        let last_acked = Arc::new(Mutex::new(Some(false)));

        let packet_collection = Arc::new(Mutex::new(Some(HashMap::new())));
        // Add packets
        for n in 0..packets_to_send.len() {
            //add_packet_to_collection(packet_collection, packets_to_send.get(n).unwrap().clone()).await;

            let packet = packets_to_send.get(n).unwrap().clone();
            let pack_num = get_seq_num_from_datapacket(packet.clone());
            let mut pack_coll_arcmut = packet_collection.lock().await;
            (*pack_coll_arcmut)
                .as_mut()
                .unwrap()
                .insert(pack_num, packet.clone());
        }

        SequenceSenderHelper {
            destination,
            num_packets_to_send,
            counter,
            first_acked,
            last_acked,
            packet_collection,
        }
    }

    pub async fn get_datapacket(&self, index: u8) -> DataPacket {
        let mut pack_coll_arcmut = self.packet_collection.lock().await;
        let cur_packet = (*pack_coll_arcmut).as_mut().unwrap().get(&index).unwrap();
        cur_packet.clone()
    }

    pub async fn get_counter(&self) -> u8 {
        return self.counter.lock().await.unwrap();
    }

    pub async fn set_counter(&mut self, new_val: u8) {
        let mut counter_arcmut = self.counter.lock().await;
        *counter_arcmut = Some(new_val);
    }

    pub async fn inc_counter(&mut self) {
        let cur_val = self.get_counter().await;
        self.set_counter(cur_val + 1).await;
    }

    pub async fn get_first_acked(&self) -> bool {
        return self.first_acked.lock().await.unwrap();
    }

    pub async fn set_first_acked(&self, new_val: bool) {
        let mut first_acked_arcmut = self.first_acked.lock().await;
        *first_acked_arcmut = Some(new_val);
    }

    pub async fn get_last_acked(&self) -> bool {
        return self.last_acked.lock().await.unwrap();
    }

    pub async fn set_last_acked(&self, new_val: bool) {
        let mut last_acked_arcmut = self.last_acked.lock().await;
        *last_acked_arcmut = Some(new_val);
    }

    pub async fn get_next_packet(&mut self) -> Option<DataPacket> {
        let next_num = self.get_counter().await;

        if next_num == self.num_packets_to_send + 1 {
            return None;
        }

        self.set_counter(next_num + 1).await;

        Some(self.get_datapacket(next_num).await)
    }

    pub async fn process_nack(&mut self, nack_packet: DataPacket) {
        // Read values and reset counter
        let seq_num = get_seq_num_from_nack(nack_packet).await;
        self.set_counter(seq_num).await;
    }
}
