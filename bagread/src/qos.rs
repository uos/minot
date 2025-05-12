use anyhow::anyhow;
use serde::{Deserialize, Serialize};
use std::str::FromStr;

#[derive(Debug, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum RmwQosPolicyKind {
    Invalid = 1 << 0,
    Durability = 1 << 1,
    Deadline = 1 << 2,
    Liveliness = 1 << 3,
    Reliability = 1 << 4,
    History = 1 << 5,
    Lifespan = 1 << 6,
    Depth = 1 << 7,
    LivelinessLeaseDuration = 1 << 8,
    AvoidRosNamespaceConventions = 1 << 9,
}

impl FromStr for RmwQosPolicyKind {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as a string first
        match s {
            "durability" => Ok(RmwQosPolicyKind::Durability),
            "deadline" => Ok(RmwQosPolicyKind::Deadline),
            "liveliness" => Ok(RmwQosPolicyKind::Liveliness),
            "reliability" => Ok(RmwQosPolicyKind::Reliability),
            "history" => Ok(RmwQosPolicyKind::History),
            "lifespan" => Ok(RmwQosPolicyKind::Lifespan),
            "depth" => Ok(RmwQosPolicyKind::Depth),
            "liveliness_lease_duration" => Ok(RmwQosPolicyKind::LivelinessLeaseDuration),
            "avoid_ros_namespace_conventions" => Ok(RmwQosPolicyKind::AvoidRosNamespaceConventions),
            "invalid" => Ok(RmwQosPolicyKind::Invalid),
            _ => {
                // If string parsing fails, try parsing as an integer
                if let Ok(value) = s.parse::<i32>() {
                    match value {
                        1 => Ok(RmwQosPolicyKind::Invalid),
                        2 => Ok(RmwQosPolicyKind::Durability),
                        4 => Ok(RmwQosPolicyKind::Deadline),
                        8 => Ok(RmwQosPolicyKind::Liveliness),
                        16 => Ok(RmwQosPolicyKind::Reliability),
                        32 => Ok(RmwQosPolicyKind::History),
                        64 => Ok(RmwQosPolicyKind::Lifespan),
                        128 => Ok(RmwQosPolicyKind::Depth),
                        256 => Ok(RmwQosPolicyKind::LivelinessLeaseDuration),
                        512 => Ok(RmwQosPolicyKind::AvoidRosNamespaceConventions),
                        _ => Err(anyhow!("Unknown number for Policy kind")),
                    }
                } else {
                    Err(anyhow!("Unknown encoding format for Policy kind"))
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum RmwQosReliabilityPolicy {
    /// Implementation specific default
    SystemDefault = 0,
    /// Guarantee that samples are delivered, may retry multiple times.
    Reliable = 1,
    /// Attempt to deliver samples, but some may be lost if the network is not robust
    BestEffort = 2,
    /// Reliability policy has not yet been set
    Unknown = 3,
    /// Will match the majority of endpoints and use a reliable policy if possible
    BestAvailable = 4,
}

impl FromStr for RmwQosReliabilityPolicy {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as a string first
        match s {
            "system_default" => Ok(RmwQosReliabilityPolicy::SystemDefault),
            "reliable" => Ok(RmwQosReliabilityPolicy::Reliable),
            "best_effort" => Ok(RmwQosReliabilityPolicy::BestEffort),
            "best_available" => Ok(RmwQosReliabilityPolicy::BestAvailable),
            "unknown" => Ok(RmwQosReliabilityPolicy::Unknown),
            _ => {
                // If string parsing fails, try parsing as an integer
                if let Ok(value) = s.parse::<i32>() {
                    match value {
                        0 => Ok(RmwQosReliabilityPolicy::SystemDefault),
                        1 => Ok(RmwQosReliabilityPolicy::Reliable),
                        2 => Ok(RmwQosReliabilityPolicy::BestEffort),
                        3 => Ok(RmwQosReliabilityPolicy::Unknown),
                        4 => Ok(RmwQosReliabilityPolicy::BestAvailable),
                        _ => Err(anyhow!("Unknown number for Reliability kind")),
                    }
                } else {
                    Err(anyhow!("Unknown encoding format for Reliability kind"))
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum RmwQosHistoryPolicy {
    /// Implementation default for history policy
    SystemDefault = 0,
    /// Only store up to a maximum number of samples, dropping oldest once max is exceeded
    KeepLast = 1,
    /// Store all samples, subject to resource limits
    KeepAll = 2,
    /// History policy has not yet been set
    Unknown = 3,
}

impl FromStr for RmwQosHistoryPolicy {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as a string first
        match s {
            "system_default" => Ok(RmwQosHistoryPolicy::SystemDefault),
            "keep_last" => Ok(RmwQosHistoryPolicy::KeepLast),
            "keep_all" => Ok(RmwQosHistoryPolicy::KeepAll),
            "unknown" => Ok(RmwQosHistoryPolicy::Unknown),
            _ => {
                // If string parsing fails, try parsing as an integer
                if let Ok(value) = s.parse::<i32>() {
                    match value {
                        0 => Ok(RmwQosHistoryPolicy::SystemDefault),
                        1 => Ok(RmwQosHistoryPolicy::KeepLast),
                        2 => Ok(RmwQosHistoryPolicy::KeepAll),
                        3 => Ok(RmwQosHistoryPolicy::Unknown),
                        _ => Err(anyhow!("Unknown number for History kind")),
                    }
                } else {
                    Err(anyhow!("Unknown encoding format for History  kind"))
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum RmwQosDurabilityPolicy {
    /// Implementation specific default
    SystemDefault = 0,
    /// The rmw publisher is responsible for persisting samples for “late-joining” subscribers
    TransientLocal = 1,
    /// Samples are not persistent
    Volatile = 2,
    /// Durability policy has not yet been set
    Unknown = 3,
    /// Will match the majority of endpoints and use a transient local policy if possible
    BestAvailable = 4,
}

impl FromStr for RmwQosDurabilityPolicy {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as a string first
        match s {
            "system_default" => Ok(RmwQosDurabilityPolicy::SystemDefault),
            "transient_local" => Ok(RmwQosDurabilityPolicy::TransientLocal),
            "volatile" => Ok(RmwQosDurabilityPolicy::Volatile),
            "best_available" => Ok(RmwQosDurabilityPolicy::BestAvailable),
            "unknown" => Ok(RmwQosDurabilityPolicy::Unknown),
            _ => {
                // If string parsing fails, try parsing as an integer
                if let Ok(value) = s.parse::<i32>() {
                    match value {
                        0 => Ok(RmwQosDurabilityPolicy::SystemDefault),
                        1 => Ok(RmwQosDurabilityPolicy::TransientLocal),
                        2 => Ok(RmwQosDurabilityPolicy::Volatile),
                        3 => Ok(RmwQosDurabilityPolicy::Unknown),
                        4 => Ok(RmwQosDurabilityPolicy::BestAvailable),
                        _ => Err(anyhow!("Unknown number for Durability kind")),
                    }
                } else {
                    Err(anyhow!("Unknown encoding format for Durability kind"))
                }
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq, Clone, Copy, Serialize, Deserialize)]
pub enum RmwQosLivelinessPolicy {
    /// Implementation specific default
    SystemDefault = 0,
    /// The signal that establishes a Topic is alive comes from the ROS rmw layer.
    Automatic = 1,
    /// Explicitly asserting node liveliness is required in this case.
    // #[deprecated(
    //     note = "RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_NODE is deprecated. Use ManualByTopic if manually asserted liveliness is needed."
    // )]
    ManualByNode = 2,
    /// The signal that establishes a Topic is alive is at the Topic level.
    ManualByTopic = 3,
    /// Liveliness policy has not yet been set
    Unknown = 4,
    /// Will match the majority of endpoints and use a manual by topic policy if possible
    BestAvailable = 5,
}

impl FromStr for RmwQosLivelinessPolicy {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        // Try parsing as a string first
        match s {
            "system_default" => Ok(RmwQosLivelinessPolicy::SystemDefault),
            "automatic" => Ok(RmwQosLivelinessPolicy::Automatic),
            // #[allow(deprecated)] // Allow parsing into deprecated variant
            "manual_by_node" => Ok(RmwQosLivelinessPolicy::ManualByNode),
            "manual_by_topic" => Ok(RmwQosLivelinessPolicy::ManualByTopic),
            "best_available" => Ok(RmwQosLivelinessPolicy::BestAvailable),
            "unknown" => Ok(RmwQosLivelinessPolicy::Unknown),
            _ => {
                // If string parsing fails, try parsing as an integer
                if let Ok(value) = s.parse::<i32>() {
                    match value {
                        0 => Ok(RmwQosLivelinessPolicy::SystemDefault),
                        1 => Ok(RmwQosLivelinessPolicy::Automatic),
                        2 => Ok(RmwQosLivelinessPolicy::ManualByNode),
                        3 => Ok(RmwQosLivelinessPolicy::ManualByTopic),
                        4 => Ok(RmwQosLivelinessPolicy::Unknown),
                        5 => Ok(RmwQosLivelinessPolicy::BestAvailable),
                        _ => Err(anyhow!("Unknown number for Liveliness kind")),
                    }
                } else {
                    Err(anyhow!("Unknown encoding format for Liveliness kind"))
                }
            }
        }
    }
}
