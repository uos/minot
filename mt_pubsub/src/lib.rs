//! Typed pub/sub over Zenoh peers. Coordinator modes remain available for legacy routes.
mod coordinated;
mod peer;
mod quic;
mod quic_plain;
mod quic_shm;
mod quic_stream;
/// Shared memory use of the QUIC peer transport: samples (sent, received).
pub use quic_shm::shm_samples as peer_shm_samples;
pub use quic_stream::{BudgetScope, QuicStats, SamplePolicy};

pub use coordinated::{CoordMode, NodeConfig, PeerTransport};
#[cfg(feature = "image")]
pub use mt_net::image::{self, ImageMessageExt, ToImage};
pub use mt_sea::claim::{NetworkClaim, claim_network};
pub use mt_sea::{
    ArchivedMessage, Durability, NodeOptions, Qos, ReconnectPolicy, Reliability, Timing,
};

use mt_sea::{Sendable, Ship};
use std::sync::Arc;
use tokio_util::sync::CancellationToken;

#[derive(Debug, Clone)]
pub struct Node {
    backend: NodeBackend,
}

#[derive(Debug, Clone)]
enum NodeBackend {
    Peer(Arc<peer::PeerNode>),
    Quic(Arc<quic::QuicNode>),
    Coordinated(coordinated::Node),
}

impl Node {
    pub async fn create(config: NodeConfig) -> anyhow::Result<Self> {
        if matches!(config.coordinator_mode(), CoordMode::Peer) {
            if config.transport() == PeerTransport::Quic {
                return Ok(Self {
                    backend: NodeBackend::Quic(quic::QuicNode::create(&config).await?),
                });
            }
            Ok(Self {
                backend: NodeBackend::Peer(peer::PeerNode::create(&config).await?),
            })
        } else {
            Ok(Self {
                backend: NodeBackend::Coordinated(coordinated::Node::create(config).await?),
            })
        }
    }
    pub async fn create_publisher<T: Sendable>(
        &self,
        topic: String,
        qos: Qos,
    ) -> anyhow::Result<Publisher<T>> {
        match &self.backend {
            NodeBackend::Peer(node) => Ok(Publisher {
                backend: PublisherBackend::Peer(
                    peer::PeerPublisher::create(Arc::clone(node), topic, qos).await?,
                ),
            }),
            NodeBackend::Quic(node) => Ok(Publisher {
                backend: PublisherBackend::Quic(
                    quic::QuicPublisher::create(Arc::clone(node), topic, qos).await?,
                ),
            }),
            NodeBackend::Coordinated(node) => Ok(Publisher {
                backend: PublisherBackend::Coordinated(node.create_publisher(topic, qos).await?),
            }),
        }
    }
    pub async fn create_subscriber<T: Sendable>(
        &self,
        topic: String,
        queue_size: usize,
        qos: Qos,
    ) -> anyhow::Result<Subscriber<T>> {
        match &self.backend {
            NodeBackend::Peer(node) => Ok(Subscriber {
                backend: SubscriberBackend::Peer(
                    peer::PeerSubscriber::create(Arc::clone(node), topic, queue_size, qos).await?,
                ),
            }),
            NodeBackend::Quic(node) => Ok(Subscriber {
                backend: SubscriberBackend::Quic(
                    quic::QuicSubscriber::create(Arc::clone(node), topic, queue_size, qos).await?,
                ),
            }),
            NodeBackend::Coordinated(node) => Ok(Subscriber {
                backend: SubscriberBackend::Coordinated(
                    node.create_subscriber(topic, queue_size, qos).await?,
                ),
            }),
        }
    }
    /// Counters of the QUIC peer transport; `None` on any other transport.
    pub fn quic_stats(&self) -> Option<QuicStats> {
        match &self.backend {
            NodeBackend::Quic(node) => Some(node.stats()),
            _ => None,
        }
    }
    pub fn mode(&self) -> Qos {
        match &self.backend {
            NodeBackend::Peer(node) => node.mode,
            NodeBackend::Quic(node) => node.mode,
            NodeBackend::Coordinated(node) => node.mode(),
        }
    }
    pub fn shutdown_token(&self) -> CancellationToken {
        match &self.backend {
            NodeBackend::Peer(node) => node.shutdown.clone(),
            NodeBackend::Quic(node) => node.shutdown.clone(),
            NodeBackend::Coordinated(node) => node.shutdown_token(),
        }
    }
    /// Peer mode reports session lifetime; coordinated mode reports coordinator generations.
    pub fn connection(&self) -> Arc<mt_sea::ConnectionState> {
        match &self.backend {
            NodeBackend::Peer(node) => Arc::clone(&node.connection),
            NodeBackend::Quic(node) => Arc::clone(&node.connection),
            NodeBackend::Coordinated(node) => node.connection(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct Publisher<T: Sendable> {
    backend: PublisherBackend<T>,
}

#[derive(Debug, Clone)]
enum PublisherBackend<T: Sendable> {
    Peer(peer::PeerPublisher<T>),
    Quic(quic::QuicPublisher<T>),
    Coordinated(coordinated::Publisher<T>),
}
impl<T: Sendable> Publisher<T> {
    pub async fn publish(&self, data: &T) -> anyhow::Result<()> {
        match &self.backend {
            PublisherBackend::Peer(publisher) => publisher.publish(data).await,
            PublisherBackend::Quic(publisher) => publisher.publish(data).await,
            PublisherBackend::Coordinated(publisher) => publisher.publish(data).await,
        }
    }
    /// Wait until discovery has observed at least one subscriber. Samples sent
    /// before discovery completes may have no recipients, just as with DDS.
    pub async fn wait_for_subscribers(&self, timeout: std::time::Duration) -> anyhow::Result<()> {
        self.wait_for_subscriber_count(1, timeout).await
    }

    /// Number of receiving endpoints currently observed through discovery.
    pub async fn subscriber_count(&self) -> anyhow::Result<usize> {
        match &self.backend {
            PublisherBackend::Peer(publisher) => publisher.subscriber_count().await,
            PublisherBackend::Quic(publisher) => Ok(publisher.subscriber_count()),
            PublisherBackend::Coordinated(publisher) => {
                match publisher.ship.ask_for_action(&publisher.topic).await? {
                    (mt_sea::Action::Shoot { target, .. }, _) => Ok(target.len()),
                    _ => Ok(0),
                }
            }
        }
    }

    /// Wait for a known startup group before sending its first samples.
    pub async fn wait_for_subscriber_count(
        &self,
        minimum: usize,
        timeout: std::time::Duration,
    ) -> anyhow::Result<()> {
        anyhow::ensure!(minimum > 0, "minimum subscriber count must be positive");
        tokio::time::timeout(timeout, async {
            loop {
                if self.subscriber_count().await? >= minimum {
                    return Ok(());
                }
                tokio::time::sleep(std::time::Duration::from_millis(20)).await;
            }
        })
        .await
        .map_err(|_| anyhow::anyhow!("timed out waiting for {minimum} subscribers"))?
    }
}

#[derive(Debug)]
pub struct Subscriber<T: Sendable> {
    backend: SubscriberBackend<T>,
}

#[derive(Debug)]
enum SubscriberBackend<T: Sendable> {
    Peer(peer::PeerSubscriber<T>),
    Quic(quic::QuicSubscriber<T>),
    Coordinated(coordinated::Subscriber<T>),
}
impl<T: Sendable> Subscriber<T> {
    pub async fn unsubscribe(self) -> anyhow::Result<()> {
        match self.backend {
            SubscriberBackend::Peer(subscriber) => {
                drop(subscriber);
                Ok(())
            }
            SubscriberBackend::Quic(subscriber) => {
                drop(subscriber);
                Ok(())
            }
            SubscriberBackend::Coordinated(subscriber) => subscriber.unsubscribe().await,
        }
    }
    pub fn try_next(&mut self) -> Option<T> {
        match &mut self.backend {
            SubscriberBackend::Peer(subscriber) => subscriber.try_next(),
            SubscriberBackend::Quic(subscriber) => subscriber.try_next(),
            SubscriberBackend::Coordinated(subscriber) => subscriber.try_next(),
        }
    }
    pub async fn next(&mut self) -> Option<T> {
        match &mut self.backend {
            SubscriberBackend::Peer(subscriber) => {
                subscriber.next_archived().await.map(|message| {
                    message
                        .deserialize()
                        .expect("validated message must deserialize")
                })
            }
            SubscriberBackend::Quic(subscriber) => {
                subscriber.next_archived().await.map(|message| {
                    message
                        .deserialize()
                        .expect("validated message must deserialize")
                })
            }
            SubscriberBackend::Coordinated(subscriber) => subscriber.next().await,
        }
    }
    pub async fn next_archived(&mut self) -> Option<ArchivedMessage<T>> {
        match &mut self.backend {
            SubscriberBackend::Peer(subscriber) => subscriber.next_archived().await,
            SubscriberBackend::Quic(subscriber) => subscriber.next_archived().await,
            SubscriberBackend::Coordinated(subscriber) => subscriber.next_archived().await,
        }
    }
}
