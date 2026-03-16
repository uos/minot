use mt_net::Imu;
use rkyv::string::ArchivedString;
use ros2_interfaces_jazzy_rkyv::std_msgs::msg::Header;

fn main() {
    println!("ArchivedString size: {}, align: {}", std::mem::size_of::<ArchivedString>(), std::mem::align_of::<ArchivedString>());
    println!("ArchivedImu size: {}, align: {}", std::mem::size_of::<rkyv::Archived<Imu>>(), std::mem::align_of::<rkyv::Archived<Imu>>());
    println!("ArchivedHeader size: {}, align: {}", std::mem::size_of::<rkyv::Archived<Header>>(), std::mem::align_of::<rkyv::Archived<Header>>());
}
