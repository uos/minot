use log::error;
use sea::{coordinator::COMPARE_NODE_NAME, net::Packet, Action, Cannon, Ship, ShipKind};

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
    app::App,
    event::{Event, EventHandler},
    handler::handle_key_events,
    tui::Tui,
};

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let env = env_logger::Env::new().filter_or("LH_LOG", "off");
    env_logger::Builder::from_env(env).init();

    // let var_data: Arc<RwLock<Vec<(String, String)>>> = Arc::new(RwLock::new(Vec::new()));

    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat(COMPARE_NODE_NAME.to_string()), None)
            .await?;

    let (ndata_tx, ndata_rx) = tokio::sync::mpsc::channel(10);
    // let write_data = Arc::clone(&var_data);
    // tokio::spawn(async move {
    let (mut sub, recv) = {
        let client = comparer.client.lock().await;
        let sender = client
            .coordinator_send
            .read()
            .unwrap()
            .as_ref()
            .cloned()
            .expect("Sender coord not available");
        let sub = client
            .coordinator_receive
            .read()
            .unwrap()
            .as_ref()
            .map(|sub| sub.subscribe())
            .ok_or(anyhow!(
                "Sender to Coordinator is available but Receiver is not."
            ))?;
        (sub, sender)
    };

    let (tx, mut rx) = tokio::sync::mpsc::channel(10);

    tokio::spawn(async move {
        while let Some(ui_received) = rx.recv().await {
            match recv
                .send(Packet {
                    header: sea::net::Header::default(),
                    data: ui_received,
                })
                .await
            {
                Ok(_) => {}
                Err(e) => {
                    error!("Could not send LockNext to net client: {e}");
                }
            };
        }
    });

    tokio::spawn(async move {
        loop {
            match sub.recv().await {
                Ok((packet, _)) => {
                    match packet.data {
                        sea::net::PacketKind::RatAction {
                            action,
                            lock_until_ack: _,
                        } => {
                            if let crate::Action::Catch { source } = action {
                                match comparer.get_cannon().catch_dyn(&source).await {
                                    Ok((strrep, _var_type, var_name)) => {
                                        let name = match &source.kind {
                                            ShipKind::Rat(name) => name.clone(),
                                            ShipKind::Wind(_) => {
                                                error!("Received something from Wind.");
                                                continue;
                                            }
                                        };

                                        match ndata_tx.send((name, strrep, var_name)).await {
                                            Ok(_) => {}
                                            Err(e) => {
                                                error!("Error sending received variable: {e}");
                                            }
                                        }
                                    }
                                    Err(e) => {
                                        // TODO should print all errors somewhere in the TUI. maybe popup?
                                        error!("Could not catch dynamic typed var: {e}");
                                    }
                                }
                                // dbg!("Catched!");
                            }
                        }
                        _ => (),
                    }
                }
                Err(e) => {
                    error!("Error receiving action for comparer: {e}");
                }
            }
        }
    });

    let mut app = App::new(tx, ndata_rx).await;

    let backend = CrosstermBackend::new(io::stdout());
    let terminal = Terminal::new(backend)?;
    let events = EventHandler::new(250);
    let mut tui = Tui::new(terminal, events);
    tui.init()?;

    // // Start the main loop.
    while app.running {
        tokio::task::yield_now().await;
        // Render the user interface.
        tui.draw(&mut app)?;
        // Handle events.
        match tui.events.next().await? {
            Event::Tick => app.tick(),
            Event::Key(key_event) => handle_key_events(key_event, &mut app).await?,
            Event::Mouse(_) => {}
            Event::Resize(_, _) => {}
        }
    }

    tui.exit()?;
    // loop {
    //     tokio::task::yield_now().await;
    // }
    return Ok(());
}
