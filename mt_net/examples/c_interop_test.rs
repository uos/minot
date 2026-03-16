use rkyv::rancor::Error;
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::Imu;

#[derive(Clone, Copy)]
#[repr(C)]
pub struct RkyvOutOfLine {
    pub len: u32,
    pub offset: i32,
}

#[derive(Clone, Copy)]
#[repr(C)]
pub union RkyvString {
    pub out_of_line: RkyvOutOfLine,
    pub inline: [u8; 8],
}

impl RkyvString {
    pub fn is_inline(&self) -> bool {
        unsafe { self.inline[0] & 0xc0 != 0x80 }
    }

    pub fn len(&self) -> usize {
        if self.is_inline() {
            unsafe { self.inline.iter().position(|b| *b == 0xff).unwrap_or(8) }
        } else {
            let len = unsafe { self.out_of_line.len };
            let len = (len & 0b0011_1111) | ((len & !0xff) >> 2);
            len as usize
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn as_str(&self) -> &str {
        if self.is_inline() {
            unsafe {
                let slice = std::slice::from_raw_parts(self.inline.as_ptr(), self.len());
                std::str::from_utf8(slice).unwrap()
            }
        } else {
            unsafe {
                let base_ptr = self as *const _ as *const u8;
                let offset = self.out_of_line.offset as isize;
                let string_ptr = base_ptr.offset(offset);
                let slice = std::slice::from_raw_parts(string_ptr, self.len());
                std::str::from_utf8(slice).unwrap()
            }
        }
    }
}

#[repr(C)]
#[derive(Debug)]
pub struct CTime {
    pub sec: i32,
    pub nanosec: u32,
}

#[repr(C, align(8))]
pub struct CHeader {
    pub stamp: CTime,
    pub frame_id: RkyvString,
}

#[repr(C)]
#[derive(Debug)]
pub struct CQuaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[repr(C)]
#[derive(Debug)]
pub struct CVector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[repr(C)]
pub struct CImu {
    pub header: CHeader,
    pub orientation: CQuaternion,
    pub orientation_covariance: [f64; 9],
    pub angular_velocity: CVector3,
    pub angular_velocity_covariance: [f64; 9],
    pub linear_acceleration: CVector3,
    pub linear_acceleration_covariance: [f64; 9],
}

fn main() {
    let mut imu = Imu::default();
    imu.header.frame_id = "test_frame_123".to_string();
    imu.linear_acceleration.x = 9.81;

    let bytes = rkyv::api::high::to_bytes::<Error>(&imu).unwrap();

    let arch_imu = unsafe { rkyv::access_unchecked::<rkyv::Archived<Imu>>(&bytes) };
    let c_imu = unsafe { &*(arch_imu as *const _ as *const CImu) };

    println!("C Imu Linear Accel X: {}", c_imu.linear_acceleration.x);
    println!("C Imu Frame ID: {}", c_imu.header.frame_id.as_str());
}
