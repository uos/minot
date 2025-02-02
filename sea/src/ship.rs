use async_trait::async_trait;

use crate::cannon::CannonImpl;

pub struct NetworkShipImpl {
    cannon: Box<dyn crate::Cannon>,
}

#[async_trait]
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
    pub fn new() -> Self {
        Self {
            cannon: Box::new(CannonImpl::new()),
        }
    }
}

impl Drop for NetworkShipImpl {
    fn drop(&mut self) {
        todo!("disconnect from network")
    }
}
