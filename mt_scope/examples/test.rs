use mt_scope::{Scope, ScopeConfig};

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    println!(
        "{:#?}",
        Scope::create(ScopeConfig {
            name: "quickscope".into(),
            mode: Default::default()
        })
        .await
    );
    Ok(())
}
