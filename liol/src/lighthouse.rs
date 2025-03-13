use std::sync::{Arc, RwLock};

use log::{debug, error, info};
use sea::{coordinator::COMPARE_NODE_NAME, Action, Cannon, Ship, ShipKind};

use anyhow::anyhow;
use std::io;

pub mod app;
pub mod coord;
pub mod event;
pub mod handler;
pub mod tui;
pub mod ui;

use ratatui::{backend::CrosstermBackend, Terminal};

use crate::{
    app::{App, AppResult},
    event::{Event, EventHandler},
    handler::handle_key_events,
    tui::Tui,
};

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();

    // let mut app = App::new();

    // let backend = CrosstermBackend::new(io::stdout());
    // let terminal = Terminal::new(backend)?;
    // let events = EventHandler::new(250);
    // let mut tui = Tui::new(terminal, events);
    // tui.init()?;

    // // Start the main loop.
    // while app.running {
    //     // Render the user interface.
    //     tui.draw(&mut app)?;
    //     // Handle events.
    //     match tui.events.next().await? {
    //         Event::Tick => app.tick(),
    //         Event::Key(key_event) => handle_key_events(key_event, &mut app)?,
    //         Event::Mouse(_) => {}
    //         Event::Resize(_, _) => {}
    //     }
    // }

    // tui.exit()?;
    // return Ok(());

    let var_data: Arc<RwLock<Vec<(String, String)>>> = Arc::new(RwLock::new(Vec::new()));

    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat(COMPARE_NODE_NAME.to_string()), None)
            .await?;

    // let (ndata_tx, mut ndata_rx) = tokio::sync::mpsc::channel(10);
    // let write_data = Arc::clone(&var_data);
    // tokio::spawn(async move {
    let mut sub = {
        let client = comparer.client.lock().await;
        let sub = client
            .coordinator_receive
            .read()
            .unwrap()
            .as_ref()
            .map(|sub| sub.subscribe())
            .ok_or(anyhow!(
                "Sender to Coordinator is available but Receiver is not."
            ))?;
        sub
    };

    loop {
        match sub.recv().await {
            Ok((packet, _)) => {
                if let sea::net::PacketKind::RatAction(crate::Action::Catch { source }) =
                    packet.data
                {
                    match comparer.get_cannon().catch_dyn(&source).await {
                        Ok((strrep, var_type)) => {
                            dbg!(strrep, var_type, source);
                            // write_data.write().unwrap()
                            // match ndata_tx.send((source, strrep, var_type)).await {
                            //     Ok(_) => {}
                            //     Err(e) => {
                            //         error!("Error sending received variable: {e}");
                            //     }
                            // }
                        }
                        Err(e) => {
                            error!("Could not catch dynamic typed var: {e}");
                        }
                    }
                }
            }
            Err(e) => {
                error!("Error receiving action for comparer: {e}");
            }
        }
    }
    // });

    // loop {
    // while let Some((source, sttrep, var_type)) = ndata_rx.recv().await {}
    // }

    // Ok(())
}
