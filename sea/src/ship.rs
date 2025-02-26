use log::{error, info};

use crate::cannon::CannonImpl;

pub struct NetworkShipImpl {
    cannon: Box<dyn crate::Cannon>,
}

impl crate::Ship for NetworkShipImpl {
    async fn water(&mut self, kind: crate::ShipKind) -> anyhow::Result<crate::ShipName> {
        todo!()
    }

    async fn wait_for_action(&self, kind: crate::ShipKind) -> anyhow::Result<crate::Action> {
        todo!()
    }

    async fn ask_for_action(
        &self,
        kind: crate::ShipKind,
        variable_name: &str,
    ) -> anyhow::Result<crate::Action> {
        todo!()
    }

    fn get_name(&self) -> crate::ShipName {
        todo!()
    }

    fn get_cannon(&self) -> &Box<dyn crate::Cannon> {
        &self.cannon
    }
}

impl NetworkShipImpl {
    async fn spawn_recursive_rejoin_task(
        disconnect_handle: tokio::sync::oneshot::Receiver<()>,
        mut client: crate::net::Client,
    ) {
        match disconnect_handle.await {
            Err(e) => {
                error!("Error receiving disconnect signla: {e}");
            }
            Ok(_) => match client.register().await {
                Err(e) => {
                    error!("Could not register after dropped connection: {e}");
                }
                Ok(recv) => {
                    let future = Box::pin(Self::spawn_recursive_rejoin_task(recv, client));
                    future.await;
                }
            },
        }
    }

    // TODO add external ip
    pub async fn init(controller: bool) -> anyhow::Result<Self> {
        let mut client = crate::net::Client::init(controller, None).await;
        let my_tcp_port_for_1on1 = client.other_client_entrance;

        info!("Registering for network...");
        let disconnect_handle = client.register().await?;
        info!("Registered :)");
        // TODO should also have receving channel when server sends us something without us requesting it

        // handle reconnecting to the network
        tokio::spawn(async move {
            Self::spawn_recursive_rejoin_task(disconnect_handle, client).await;
        });

        Ok(Self {
            cannon: Box::new(CannonImpl::new()),
        })
    }
}

impl Drop for NetworkShipImpl {
    fn drop(&mut self) {
        todo!("disconnect from network")
    }
}
