use anyhow::anyhow;
pub use sea;
#[cfg(feature = "ratpub")]
pub mod ratpub;
#[cfg(feature = "ros1-native")]
pub mod ros1;
#[cfg(feature = "ros2-native")]
pub mod ros2;
#[cfg(feature = "ros2-c")]
pub mod ros2_r2r;

pub fn get_env_or_default(key: &str, default: &str) -> anyhow::Result<String> {
    match std::env::var(key) {
        Ok(name) => Ok(name),
        Err(e) => match e {
            std::env::VarError::NotPresent => Ok(default.to_owned()),
            std::env::VarError::NotUnicode(_os_string) => Err(anyhow!(
                "Could not fetch env variable because it is not unicode"
            )),
        },
    }
}
