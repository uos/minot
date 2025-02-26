pub struct CannonImpl {}

impl crate::Cannon for CannonImpl {
    /// Initialize a 1:1 connection to the target. Ports are shared using the sea network internally.
    /// Dump the data to the target.
    async fn shoot(&self, target: &Vec<crate::ShipName>, data: &[u8]) {
        todo!()
    }

    /// Catch the dumped data from the source.
    async fn catch(&self, target: crate::ShipName) -> Vec<u8> {
        todo!()
    }
}

impl CannonImpl {
    pub fn new() -> Self {
        Self {}
    }
}
