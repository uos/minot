pub mod app;

pub async fn run() -> anyhow::Result<()> {
    app::run().await
}
