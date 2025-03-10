use std::sync::{Arc, RwLock};

use log::{debug, error, info};
use sea::{ship::NetworkShipImpl, Action, Cannon, Ship, ShipKind};

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    let var_data: Arc<RwLock<Vec<(String, String)>>> = Arc::new(RwLock::new(Vec::new()));

    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat("#compare".to_string()), None).await?;

    let (ndata_tx, mut ndata_rx) = tokio::sync::mpsc::channel(10);
    tokio::spawn(async move {
        loop {
            match comparer.wait_for_action().await {
                Ok(action) => match action {
                    Action::Sail => todo!(),
                    Action::Shoot { target } => {
                        error!("Received shoot. Comparer should never shoot.");
                    }
                    Action::Catch { source } => {
                        match comparer.get_cannon().catch_dyn(&source).await {
                            Ok((strrep, var_type)) => {
                                match ndata_tx.send((source, strrep, var_type)).await {
                                    Ok(_) => {}
                                    Err(e) => {
                                        error!("Error sending received variable: {e}");
                                    }
                                }
                            }
                            Err(e) => {
                                error!("Could not catch dynamic typed var: {e}");
                            }
                        }
                    }
                },
                Err(e) => {
                    error!("Error receiving action for comparer: {e}");
                }
            }
        }
    });

    loop {
        while let Some((source, sttrep, var_type)) = ndata_rx.recv().await {}
    }

    Ok(())
}
