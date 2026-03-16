use ros2_interfaces_jazzy_rkyv::sensor_msgs::msg::PointField;
fn main() {
    type ArchPF = rkyv::Archived<PointField>;
    println!(
        "ArchPF size: {}, align: {}",
        std::mem::size_of::<ArchPF>(),
        std::mem::align_of::<ArchPF>()
    );
    println!("ArchPF name offset: {}", std::mem::offset_of!(ArchPF, name));
    println!(
        "ArchPF offset offset: {}",
        std::mem::offset_of!(ArchPF, offset)
    );
    println!(
        "ArchPF datatype offset: {}",
        std::mem::offset_of!(ArchPF, datatype)
    );
    println!(
        "ArchPF count offset: {}",
        std::mem::offset_of!(ArchPF, count)
    );
}
