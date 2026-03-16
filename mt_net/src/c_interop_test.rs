use mt_net::Imu;
use rkyv::{api::high::to_bytes_in, rancor::Error, util::AlignedVec};

#[repr(C)]
#[derive(Debug)]
pub struct RkyvString {
    pub ptr_offset: i32,
    pub length: u32,
}

#[repr(C)]
#[derive(Debug)]
pub struct CHeader {
    pub seq: u32,
    pub stamp_sec: i32,
    pub stamp_nanosec: u32,
    pub frame_id: RkyvString,
}

#[repr(C)]
#[derive(Debug)]
pub struct CImu {
    pub header: CHeader,
    pub orientation_x: f64,
    pub orientation_y: f64,
    pub orientation_z: f64,
    pub orientation_w: f64,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity_x: f64,
    pub angular_velocity_y: f64,
    pub angular_velocity_z: f64,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration_x: f64,
    pub linear_acceleration_y: f64,
    pub linear_acceleration_z: f64,
    pub linear_acceleration_covariance: [f64; 9],
}

fn main() {
    let mut imu = Imu::default();
    imu.header.frame_id = "test_frame_123".to_string();
    imu.linear_acceleration.x = 9.81;

    let mut buf = AlignedVec::new();
    let bytes = to_bytes_in::<Error, _>(&imu, &mut buf).unwrap();

    let c_imu = unsafe { &*(bytes.as_ptr() as *const CImu) };
    
    println!("C Imu Address: {:p}", c_imu);
    println!("C Imu Linear Accel X: {}", c_imu.linear_acceleration_x);
    
    let frame_id_ptr = unsafe {
        (c_imu as *const _ as *const u8)
            .offset(c_imu.header.frame_id.ptr_offset as isize)
    };
    
    let frame_id_slice = unsafe {
        std::slice::from_raw_parts(frame_id_ptr, c_imu.header.frame_id.length as usize)
    };
    
    let frame_id_str = std::str::from_utf8(frame_id_slice).unwrap();
    println!("C Imu Frame ID: {}", frame_id_str);
}
