pub type DeviceID = u8;

#[derive(Debug, Clone)]
pub struct DataPacket {
    pub source: DeviceID,
    pub destination: DeviceID,
    pub payload: DataPacketType, // variable length
    pub protocol_version: u8,
}

impl DataPacket {
    pub fn new(source: DeviceID, destination: DeviceID, payload: DataPacketType) -> Self {
        Self {
            source,
            destination,
            payload,
            protocol_version: crate::constants::PROTOCOL_VERSION,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DataPacketType {
    Nack(u8), // Seq Num of the Nacked Packet
    Ack,
    ConnectRequest,
    ConnectResponse {
        ip: [u8; 4], // set by router, last 2 bytes are device_id
        prefix_len: u8,
        device_id: DeviceID,
    },
    DisconnectRequest,
    Raw(Vec<u8>), // ethernet frame data
    Ping,
    Pong,
    RejoinRequest,
    Data(Vec<u8>), // Data Packet to be sent
}

fn _build_byte_packet(packet: DataPacket, type_: u8, data: Vec<u8>) -> Vec<u8> {
    let predefined = _create_predifined_preamble(packet.destination);
    let header = _create_payload_header(type_, packet.source);

    let mut bytes = Vec::new();
    bytes.extend(&predefined);
    bytes.extend(header);

    bytes.extend_from_slice(&data);
    // override length byte with the real length
    bytes[0] = bytes.len() as u8;

    return bytes;
}

fn _create_predifined_preamble(destination: u8) -> [u8; 2] {
    // set dummy value for the length byte
    let preamble: [u8; 2] = [2, destination];
    return preamble;
}

fn _create_payload_header(packet_type: u8, source: DeviceID) -> [u8; 2] {
    let protocoll_version = crate::constants::PROTOCOL_VERSION;

    let protocoll_version_u4 = protocoll_version & 0b00001111;
    let packet_type_u4 = packet_type & 0b00001111;

    let version_and_type = (protocoll_version_u4 << 4) | packet_type_u4;

    let header: [u8; 2] = [version_and_type, source];
    return header;
}

// Deconstruct byte packet

fn _get_version_and_type(version_and_type: u8) -> [u8; 2] {
    let protocoll_version = (version_and_type & 0b11110000) >> 4;
    let packet_type = version_and_type & 0b00001111;

    return [protocoll_version, packet_type];
}

impl Into<Vec<u8>> for DataPacket {
    fn into(self) -> Vec<u8> {
        match &self.payload {
            DataPacketType::Nack(seq_num) => {
                let seq_num_loc = *seq_num;
                let mut data = Vec::new();
                data.push(seq_num_loc);
                _build_byte_packet(self, 0, data)
            }
            DataPacketType::Ack => _build_byte_packet(self, 1, Vec::new()),
            DataPacketType::ConnectRequest => _build_byte_packet(self, 2, Vec::new()),
            DataPacketType::ConnectResponse {
                ip,
                device_id,
                prefix_len,
            } => {
                let mut data = ip.to_vec();
                data.push(*device_id);
                data.push(*prefix_len);
                _build_byte_packet(self, 3, data)
            }
            DataPacketType::DisconnectRequest => _build_byte_packet(self, 4, Vec::new()),
            DataPacketType::Raw(data) => {
                // bytes.push(5);
                // bytes.extend_from_slice(data);
                let d = data.to_vec();
                _build_byte_packet(self, 5, d)
            }
            DataPacketType::RejoinRequest => _build_byte_packet(self, 6, Vec::new()),
            DataPacketType::Data(_data) => {
                _build_byte_packet(self, 7, Vec::new())
                // TODO:
            }
            DataPacketType::Ping => _build_byte_packet(self, 8, Vec::new()),
            DataPacketType::Pong => _build_byte_packet(self, 9, Vec::new()),
        }
    }
}

impl From<&Vec<u8>> for DataPacket {
    fn from(bytes: &Vec<u8>) -> Self {
        // hardware preamble
        let _length = bytes[0];
        let destination = bytes[1];
        // packet header
        let [protocoll_version, packet_type] = _get_version_and_type(bytes[2]);
        let source = bytes[3];

        let payload = match packet_type {
            0 => DataPacketType::Nack(bytes[4]),
            1 => DataPacketType::Ack,
            2 => DataPacketType::ConnectRequest,
            3 => {
                let mut ip = [0; 4];
                ip.copy_from_slice(&bytes[4..8]);
                let device_id = bytes[8];
                let prefix_len = bytes[9];
                DataPacketType::ConnectResponse {
                    ip,
                    device_id,
                    prefix_len,
                }
            }
            4 => DataPacketType::DisconnectRequest,
            5 => DataPacketType::Raw(bytes[4..].to_vec()),
            6 => DataPacketType::RejoinRequest,
            7 => DataPacketType::Data(bytes[4..].to_vec()),
            8 => DataPacketType::Ping,
            9 => DataPacketType::Pong,
            _ => panic!("Unknown packet type"),
        };
        Self {
            source,
            destination,
            payload,
            protocol_version: protocoll_version,
        }
    }
}
