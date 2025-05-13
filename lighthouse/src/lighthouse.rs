use std::{path::PathBuf, str::FromStr};

use log::{error, info, warn};
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

const EMBED_ROS2_TURBINE_NAME: &'static str = "embedded_ros2_turbine";

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let file = std::env::args().nth(1).map(|f| PathBuf::from_str(&f));

    let file = match file {
        Some(pathres) => Some(pathres?),
        None => None,
    };

    #[allow(unused_mut)]
    let mut eval = match &file {
        Some(path) => {
            println!("Compiling {:#?}", &path.canonicalize()?);
            rlc::compile_file(&path, None, None)? // evaluate entire file at first
        }
        None => rlc::Evaluated::new(),
    };
    // TODO check if we can really give an empty eval when everything is embedded

    let log_level = if let Some(rhs) = eval.vars.resolve("_log_level")? {
        match rhs {
            rlc::Rhs::Val(rlc::Val::StringVal(v)) => match v.as_str() {
                "info" => Ok(log::LevelFilter::Info),
                "warn" => Ok(log::LevelFilter::Warn),
                "error" => Ok(log::LevelFilter::Error),
                _ => Err(anyhow!("unsupported log filter")),
            },
            _ => Err(anyhow!("Expected info, warn, error for _log_level.")),
        }
    } else {
        Ok(log::LevelFilter::Info)
    }?;
    tui_logger::init_logger(log_level).unwrap();
    // tui_logger::set_env_filter_from_string(&log_level);
    // tui_logger::set_default_level(log::LevelFilter::Error);

    #[cfg(feature = "embed-coordinator")]
    {
        let locked_start = if let Some(rhs) = eval.vars.resolve("_start_locked")? {
            match rhs {
                rlc::Rhs::Val(rlc::Val::BoolVal(locked)) => Ok(locked),
                _ => Err(anyhow!("Expected bool for _start_locked.")),
            }
        } else {
            Ok(false)
        }?;

        let coord_file = if let Some(rhs) = eval.vars.resolve("_rules")? {
            match rhs {
                rlc::Rhs::Val(rlc::Val::StringVal(file)) | rlc::Rhs::Path(file) => Ok(Some(file)),
                _ => Err(anyhow!("Expected _wind_file to be path or string.")),
            }
        } else {
            Ok(None)
        }?;

        let eval = match (file.as_ref(), coord_file) {
            (Some(cfg_filepath), Some(cfpath)) => {
                let cfg_parent = cfg_filepath
                    .parent()
                    .ok_or(anyhow!("Passed .rl file does not have a parent."))?
                    .canonicalize()
                    .expect("Should have failed in prev compilation.");
                let joined = cfg_parent.join(cfpath);
                println!("Compiling separate wind {:#?}", &joined);
                let rl = rlc::compile_file(&joined, None, None)?;
                rl
            }
            (_, _) => rlc::Evaluated::new(),
        };

        #[allow(unused_mut)]
        let mut clients = coord::get_clients(&eval)?;

        #[cfg(feature = "embed-coordinator")]
        {
            let wind_name = wind::get_env_or_default("wind_name", &EMBED_ROS2_TURBINE_NAME)?;
            clients.insert(wind_name);
        }

        coord::run_coordinator(locked_start, clients, eval.rules);
    }

    #[cfg(feature = "embed-ros2-turbine")]
    {
        let wind_name = wind::get_env_or_default("ros2_wind_name", &EMBED_ROS2_TURBINE_NAME)?;
        let rlc_wind_ns = eval.vars.filter_ns(&[&wind_name]);

        let namespace =
            rlc_wind_ns
                .resolve("namespace")?
                .map_or(Ok("/wind".to_owned()), |rhs| {
                    Ok(match rhs {
                        rlc::Rhs::Path(topic) => topic,
                        _ => {
                            return Err(anyhow!(
                                "Unexpected type for _lidar.topic, expected Path."
                            ));
                        }
                    })
                })?;

        tokio::spawn(async move {
            let res = wind::ros2::run_dyn_wind(&namespace, &wind_name).await;
            match res {
                Ok(_) => {} // will never return
                Err(e) => {
                    error!("Error in embedded Wind. Reload the App. {e}")
                }
            }
        });
    }

    #[cfg(feature = "embed-ros1-turbine")]
    {
        let wind_name = wind::get_env_or_default("ros1_wind_name", &EMBED_ROS2_TURBINE_NAME)?;

        let master_uri = wind::get_env_or_default("ROS_MASTER_URI", "http://localhost:11311")?;

        tokio::spawn(async move {
            let res = wind::ros1::run_dyn_wind(&master_uri, &wind_name).await;
            match res {
                Ok(None) => {
                    warn!("ROS1 Master not found. Destroying node.");
                }
                Ok(Some(_)) => {} // will never return
                Err(e) => {
                    error!("Error in embedded Wind. Reload the App. {e}")
                }
            }
        });
    }

    let rules = eval.rules;
    println!("Searching for coordinator..."); // TODO clear and redraw this as animation with running dots while waiting for init
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
                            // info!("received catch: {source:?}");
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

    info!("Welcome to Lighthouse TUI. Have fun!");
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
