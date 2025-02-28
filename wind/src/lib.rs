use anyhow::Error;
use log::{error, info};
use sea::{net::SeaSendableBuffer, Cannon, Ship, ShipKind};
use tokio::sync::mpsc::UnboundedReceiver;

pub async fn wind(name: &str) -> anyhow::Result<UnboundedReceiver<sea::WindData>> {
    let kind = ShipKind::Wind(name.to_string());
    let ship = sea::ship::NetworkShipImpl::init(kind.clone(), None).await?;
    info!("Wind initialized with ship {:?}", kind);

    let (tx, rx) = tokio::sync::mpsc::unbounded_channel();

    tokio::spawn(async move {
        loop {
            match ship.wait_for_action(kind.clone()).await {
                Ok(sea::Action::Catch { source }) => {
                    let cannon = ship.get_cannon();
                    let data = cannon.catch(&source).await;
                    if let Some(received_wind) =
                        data.as_any().downcast_ref::<nalgebra::DMatrix<u8>>()
                    {
                        let data =
                            bincode::deserialize::<sea::WindData>(&received_wind.data.as_slice())
                                .unwrap();
                    } else {
                        error!("Could not downcast to u8 matrix");
                    }
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
