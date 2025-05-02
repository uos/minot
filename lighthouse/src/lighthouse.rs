use std::{path::PathBuf, str::FromStr};

use log::{error, info};
use rlc::COMPARE_NODE_NAME;
use sea::{Action, Cannon, Ship, ShipKind, net::Packet};

use anyhow::anyhow;

pub mod app;
pub mod coord;
pub mod event;
pub mod handler;
pub mod tui;
pub mod ui;

use ratatui::{Terminal, backend::CrosstermBackend};

use crate::{
    app::App,
    event::{Event, EventHandler},
    handler::handle_key_events,
    tui::Tui,
};

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tui_logger::init_logger(log::LevelFilter::Info).unwrap();
    tui_logger::set_default_level(log::LevelFilter::Info);
    // let env = env_logger::Env::new().filter_or("LH_LOG", "off");
    // env_logger::Builder::from_env(env).init();

    let file = std::env::args().nth(1).map(|f| PathBuf::from_str(&f));
    let file = match file {
        Some(pathres) => Some(pathres?),
        None => None,
    };

    let rules = match &file {
        Some(path) => {
            let rats = rlc::compile_file(&path, None, None)?; // evaluate entire file at first
            rats.rules
        }
        None => rlc::Rules::new(),
    };

    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat(COMPARE_NODE_NAME.to_string()), None)
            .await?;

    let (ndata_tx, ndata_rx) = tokio::sync::mpsc::channel(10);
    let (dyn_wind_tx, dyn_wind_rx) = tokio::sync::mpsc::channel(10);
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
        // override rules if there are some
        if !rules.raw().is_empty() {
            match recv
                .send(Packet {
                    header: sea::net::Header::default(),
                    data: sea::net::PacketKind::RulesClear,
                })
                .await
            {
                Ok(_) => {}
                Err(e) => {
                    error!("Could not send RulesClear to net client: {e}");
                }
            }

            for (var, rule) in rules.raw() {
                match recv
                    .send(Packet {
                        header: sea::net::Header::default(),
                        data: sea::net::PacketKind::RuleAppend {
                            variable: var.clone(), // TODO avoid both clone?
                            commands: rule.clone(),
                        },
                    })
                    .await
                {
                    Ok(_) => {}
                    Err(e) => {
                        error!("Could not send RuleAppend to net client: {e}");
                    }
                }
            }
        }

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
                Ok((packet, _)) => match packet.data {
                    sea::net::PacketKind::VariableTaskRequest(var) => {
                        info!("received var request: {var}");
                        match dyn_wind_tx.send(var).await {
                            Ok(_) => {}
                            Err(e) => {
                                error!("Could forward var request for wind: {e}");
                            }
                        }
                    }
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
                                    error!("Could not catch dynamic typed var: {e}");
                                }
                            }
                        }
                    }
                    _ => (),
                },
                Err(e) => {
                    error!("Error receiving action for comparer: {e}");
                }
            }
        }
    });

    let mut app = App::new(tx, ndata_rx, dyn_wind_rx, file).await;

    let backend = CrosstermBackend::new(std::io::stdout());
    let terminal = Terminal::new(backend)?;
    let events = EventHandler::new(250);
    let mut tui = Tui::new(terminal, events);
    tui.init()?;

    while app.running {
        tokio::task::yield_now().await;
        tui.draw(&mut app)?;
        match tui.events.next().await? {
            Event::Tick => app.tick(),
            Event::Key(key_event) => handle_key_events(key_event, &mut app).await?,
            Event::Mouse(_) => {}
            Event::Resize(_, _) => {}
        }
    }

    tui.exit()?;
    return Ok(());
}
