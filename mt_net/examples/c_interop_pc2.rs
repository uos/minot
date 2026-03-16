use rkyv::rancor::Error;
use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::{PointCloud2, PointField};

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

    // For vectors, rkyv treats them as out of line slices natively if len > 8. But wait! Vec<u8> uses 8 bytes inline optimization in rkyv just like strings do if the vec is short! Let's handle it.
    pub fn as_slice<T>(&self) -> &[T] {
        if self.is_inline() {
            unsafe {
                // If it is inline, the elements are directly inside the `inline` array.
                // Note: This only works for small u8 vectors safely without alignment issues
                let len = self.inline.iter().position(|b| *b == 0xff).unwrap_or(8);
                let ptr = self.inline.as_ptr() as *const T;
                std::slice::from_raw_parts(ptr, len / std::mem::size_of::<T>())
            }
        } else {
            unsafe {
                let base_ptr = self as *const _ as *const u8;
                let offset = self.out_of_line.offset as isize;
                let arr_ptr = base_ptr.offset(offset) as *const T;

                let len = self.out_of_line.len;
                let len = (len & 0b0011_1111) | ((len & !0xff) >> 2);
                let elements = len as usize;

                std::slice::from_raw_parts(arr_ptr, elements)
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
pub struct CPointField {
    pub name: RkyvString,
    pub offset: u32,
    pub datatype: u8,
    pub count: u32,
}

#[repr(C)]
pub struct CPointCloud2 {
    pub header: CHeader,
    pub height: u32,
    pub width: u32,
    pub fields: RkyvString,
    pub is_bigendian: bool,
    pub point_step: u32,
    pub row_step: u32,
    pub data: RkyvString,
    pub is_dense: bool,
}

fn main() {
    let mut pc = PointCloud2::default();
    pc.header.frame_id = "lidar_link".to_string();
    pc.height = 1;
    pc.width = 100;

    let field = PointField {
        name: "x".to_string(),
        offset: 0,
        datatype: 7, // FLOAT32
        count: 1,
    };
    pc.fields.push(field);

    // add 8 dummy data bytes (will hit the inline optimization!)
    pc.data = vec![10, 20, 30, 40, 50, 60, 70, 80];

    pc.is_dense = true;

    let bytes = rkyv::api::high::to_bytes::<Error>(&pc).unwrap();

    let arch_pc = unsafe { rkyv::access_unchecked::<rkyv::Archived<PointCloud2>>(&bytes) };
    let c_pc = unsafe { &*(arch_pc as *const _ as *const CPointCloud2) };

    println!("C PointCloud2 Width: {}", c_pc.width);
    println!("C PointCloud2 Height: {}", c_pc.height);
    println!("C PointCloud2 Frame ID: {}", c_pc.header.frame_id.as_str());
    println!("C PointCloud2 is_dense: {}", c_pc.is_dense);

    let data_slice: &[u8] = c_pc.data.as_slice::<u8>();
    println!("C PointCloud2 data: {:?}", data_slice);

    let fields_slice: &[CPointField] = c_pc.fields.as_slice::<CPointField>();
    println!(
        "C PointCloud2 fields[0].name: {}",
        fields_slice[0].name.as_str()
    );
    println!("C PointCloud2 fields[0].offset: {}", fields_slice[0].offset);
}
