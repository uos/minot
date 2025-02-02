use log::info;
use sea::{Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<sea::WindData>> {
    let mut ship = sea::ship::NetworkShipImpl::new();
    let kind = ShipKind::Wind(name.to_string());
    let ship_name = ship.water(kind.clone()).await?;
    info!("Wind initialized with ship {}", ship_name);

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();

    tokio::spawn(async move {
        loop {
            match ship.wait_for_action(kind.clone()).await {
                Ok(sea::Action::Catch { source }) => {
                    let data = ship.get_cannon().catch(source).await;
                    let data = bincode::deserialize::<sea::WindData>(&data).unwrap();
                    tx.send(data).unwrap();
                }
                Err(e) => {
                    info!("Error: {}", e);
                    continue;
                }
                _ => {}
            }
        }
    });

    Ok(rx)
}
