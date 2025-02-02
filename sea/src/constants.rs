use crate::packet::DeviceID;

pub const PROTOCOL_VERSION: u8 = 255 & 0b00001111; // u4 is needed
pub const NET_ID: DeviceID = 100;
pub const CLIENT_TIMEOUT: u64 = 40; // ms, depends on how long it takes to send a packet

pub const NETWORK_DEVICE_ID_MIN: DeviceID = 1;
pub const NETWORK_DEVICE_ID_MAX: DeviceID = 250;

pub const TMP_DEVICE_ID_MIN: DeviceID = 250;
pub const TMP_DEVICE_ID_MAX: DeviceID = 255;
