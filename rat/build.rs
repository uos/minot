fn main() {
    let crate_type = std::env::var("CARGO_CRATE_TYPE").unwrap();
    if crate_type == "dylib" || crate_type == "staticlib" {
        println!("cargo:rustc-cfg=build_c_interop");
    }
}
