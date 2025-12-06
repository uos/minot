fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=ROS_DISTRO");

    let ros_sourced = std::env::var("AMENT_PREFIX_PATH").is_ok() || std::env::var("ROS_DISTRO").is_ok();

    if ros_sourced {
        println!("cargo:rustc-cfg=ros2_sourced");
        if std::env::var("CARGO_FEATURE_EMBED_ROS2_C").is_err() {
            println!("cargo:warning=ROS 2 environment detected.");
            println!("cargo:warning=You might want to install with '--features embed-ros2-c' to enable ROS 2 support using rclc.");
        }
    }
}
