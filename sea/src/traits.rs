/// =======================================================================
/// # Traits for the virtual network and radio module
///
/// This file contains the traits for the virtual network and radio module.
/// The goal is to find a common base to develop for and to mock against all crates.
///
/// THIS MODULE IS SHARED BETWEEN ALL CRATES AND SHALL NOT BE CHANGED WITHOUT NOTICE!
///
/// # Notes
/// * All methods are required to operate with only an unmutable &self reference, so they can be used inside the drivers context.
/// =======================================================================
use ipnet;

#[derive(Debug, Clone)]
pub enum GenericError {
    /// This indicates an internal problem, which should lead to a safe shutdown of the driver.
    /// Those errors can occur if e.g. the driver is (safely) shutting down or if the driver
    /// encounters an inconsistent state and tries to recover by resetting the hardware.
    BrokenPipe,
}

#[async_trait::async_trait]
pub trait VirtualEthernetTrait: Sized + Send + Sync + 'static {
    /// This function must be called to assign the virtual network interface an IP with subnet.
    /// If you do not call this, you won't be able to send or receive any packets on this interface,
    /// as no default configuration is provided. Due to restrictions of the underlying implementation,
    /// you can only assign one IP with subnet at a time and are also not allowed to modify the randomized
    /// MAC address of the virtual network interface.
    ///
    /// # Arguments
    ///
    /// * `net` - The IP with subnet to assign to the virtual network interface. If you call this with None, the virtual network interface will be unconfigured.
    /// * `default_route` - If true, the virtual network interface will be configured as the default route (i.e. receiving all system traffic).
    ///
    /// # Returns
    ///
    /// This function returns an error if the configuration could not be applied (feel free to panic).
    ///
    async fn reconfigure(&self, net: Option<ipnet::IpNet>, default_route: bool) -> Result<(), ()>;

    /// This function returns the current network configuration.
    async fn get_network(&self) -> Option<ipnet::IpNet>;

    /// This function returns the current default route configuration.
    async fn get_default_route(&self) -> bool;
}

#[async_trait::async_trait]
pub trait RadioModuleTrait: Sized + Send + Sync + 'static {
    /// This function must be called to assign the radio module a network ID.
    ///
    /// If you do not call this, a default value of 0 will be used for the network id.
    /// Any send() or receive() call will transmit on this network ID. You have to call
    /// this function before calling send() or receive() for the first time to set your
    /// own chip address!
    ///
    /// # Arguments
    ///
    /// * `chip_address` - The chip address to assign to the radio module. You will only receive() packets for this chip address.
    /// * `network_id` - The network ID to assign to the radio module.
    ///
    /// # Returns
    ///
    /// This function returns an error if the configuration could not be applied (feel free to panic).
    ///
    async fn reconfigure(&self, chip_address: u8, network_id: u8) -> Result<(), ()>;

    /// This function returns the current chip address.
    async fn get_chip_address(&self) -> u8;

    /// This function returns the current network ID.
    async fn get_network_id(&self) -> u8;
}

#[async_trait::async_trait]
pub trait RouterTrait: Sized + Send {
    /// Create a new logic instance for a server with the given network.
    async fn new(network: ipnet::IpNet) -> Self;

    /// Run the logic - if you ever return, the daemon will shut down.
    async fn run(
        &self,
        vnet: std::sync::Arc<impl VirtualEthernetTrait>,
        vnet_send: tokio::sync::broadcast::Sender<Vec<u8>>,
        vnet_rcv: tokio::sync::broadcast::Receiver<Vec<u8>>,
        radio: std::sync::Arc<impl RadioModuleTrait>,
        radio_send: tokio::sync::broadcast::Sender<Vec<u8>>,
        radio_rcv: tokio::sync::broadcast::Receiver<Vec<u8>>,
    ) -> ();

    /// Ping a peer.
    ///
    /// # Arguments
    ///
    /// * `peer_address` - The peer address to ping.
    ///
    /// # Returns
    ///
    /// This function returns the ping time in milliseconds or an error if the ping timed out.
    ///
    async fn ping(&self, peer_address: u8) -> Result<std::time::Duration, ()>;

    /// Get a list of connected peers.
    ///
    /// # Returns
    ///
    /// This function returns a list of connected peer addresses with their IPv4 address.
    ///
    async fn get_peers(&self) -> Result<Vec<(u8, std::net::Ipv4Addr)>, ()>;
}

#[async_trait::async_trait]
pub trait ClientTrait: Sized + Send + Sync + 'static {
    /// Create a new logic instance for a client.
    fn new() -> Self;

    /// Run the logic - if you ever return, the daemon will shut down.
    async fn run(
        &self,
        vnet: std::sync::Arc<impl VirtualEthernetTrait>,
        vnet_send: tokio::sync::broadcast::Sender<Vec<u8>>,
        vnet_rcv: tokio::sync::broadcast::Receiver<Vec<u8>>,
        radio: std::sync::Arc<impl RadioModuleTrait>,
        radio_send: tokio::sync::broadcast::Sender<Vec<u8>>,
        radio_rcv: tokio::sync::broadcast::Receiver<Vec<u8>>,
    ) -> ();

    /// Connect this client to the well-known router chip address of the network.
    ///
    /// # Returns
    ///
    /// This function returns the used shared secret for the encryption.
    /// If the connection failed (or is already connected), this function returns an error.
    ///
    async fn connect(&self) -> Result<String, ()>;

    /// Disconnect this client (gracefully).
    ///
    /// # Returns
    ///
    /// This function returns an error if the client was not connected.
    ///
    async fn disconnect(&self) -> Result<(), ()>;

    /// Ping a peer.
    ///
    /// # Arguments
    ///
    /// * `peer_address` - The peer chip address to ping.
    ///
    /// # Returns
    ///
    /// This function returns the ping time or an error if the ping timed out.
    ///
    async fn ping(&self, peer_address: u8) -> Result<std::time::Duration, ()>;
}
