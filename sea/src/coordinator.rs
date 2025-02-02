use async_trait::async_trait;

pub struct CoordinatorImpl {}

#[async_trait]
impl crate::Coordinator for CoordinatorImpl {
    async fn rat_action_request_queue(
        &self,
        ship: crate::ShipName,
    ) -> anyhow::Result<tokio::sync::mpsc::Receiver<String>> {
        todo!()
    }
    async fn wind_set_sail(
        &self,
        ship: crate::ShipName,
        data: crate::WindData,
    ) -> anyhow::Result<()> {
        todo!()
    }
}

impl CoordinatorImpl {
    pub fn new() -> Self {
        Self {}
    }
}
