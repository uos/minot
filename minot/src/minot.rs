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

#[cfg(feature = "embed-ros2-turbine")]
const EMBED_ROS2_TURBINE_NAME: &'static str = "embedded_ros2_turbine";
#[cfg(feature = "embed-ros2-c-turbine")]
const EMBED_ROS2_C_TURBINE_NAME: &'static str = "embedded_ros2_c_turbine";
#[cfg(feature = "embed-ros1-turbine")]
const EMBED_ROS1_TURBINE_NAME: &'static str = "embedded_ros1_turbine";
#[cfg(feature = "embed-ratpub-turbine")]
const EMBED_RATPUB_TURBINE_NAME: &'static str = "embedded_ratpub_turbine";

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
            let compiled = rlc::compile_file(path, None, None)?; // evaluate entire file at first
            // remove compile feedback
            print!("\x1b[1A"); // move cursor up 1 line
            print!("\x1b[2K"); // erase line
            compiled
        }
        None => rlc::Evaluated::new(),
    };
    // TODO check if we can really give an empty eval when everything is embedded

    let log_level = if let Some(rhs) = eval.vars.resolve("_log_level")? {
        match rhs {
            rlc::Rhs::Val(rlc::Val::StringVal(v)) => match v.as_str() {
                "info" => Ok(log::LevelFilter::Info),
                "debug" => Ok(log::LevelFilter::Debug),
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
    tui_logger::set_default_level(log_level);

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

        #[cfg(feature = "embed-ratpub-turbine")]
        {
            let wind_name =
                wind::get_env_or_default("ratpub_wind_name", &EMBED_RATPUB_TURBINE_NAME)?;

            tokio::spawn(async move {
                let res = wind::ratpub::run_dyn_wind(&wind_name).await;
                match res {
                    Ok(_) => {} // will never return
                    Err(e) => {
                        error!("Error in embedded ratpub turbine. Reload the App. {e}")
                    }
                }
            });
        }

        let eval = match (file.as_ref(), coord_file) {
            (Some(cfg_filepath), Some(cfpath)) => {
                let cfg_parent = cfg_filepath
                    .parent()
                    .ok_or(anyhow!("Passed .rl file does not have a parent."))?
                    .canonicalize()
                    .expect("Should have failed in prev compilation.");
                let joined = cfg_parent.join(cfpath);
                println!("Compiling separate wind {:#?}", &joined);

                rlc::compile_file(&joined, None, None)?
            }
            (_, _) => rlc::Evaluated::new(),
        };

        #[allow(unused_mut)]
        let mut clients = coord::get_clients(&eval)?;

        #[cfg(feature = "embed-ros2-turbine")]
        {
            let wind_name = wind::get_env_or_default("wind_ros2_name", &EMBED_ROS2_TURBINE_NAME)?;
            clients.insert(wind_name);
        }

        #[cfg(feature = "embed-ros2-c-turbine")]
        {
            let wind_name =
                wind::get_env_or_default("wind_ros2_c_name", &EMBED_ROS2_C_TURBINE_NAME)?;
            clients.insert(wind_name);
        }

        #[cfg(feature = "embed-ros1-turbine")]
        {
            let wind_name = wind::get_env_or_default("wind_ros1_name", &EMBED_ROS1_TURBINE_NAME)?;
            clients.insert(wind_name);
        }

        #[cfg(feature = "embed-ratpub-turbine")]
        {
            let wind_name =
                wind::get_env_or_default("wind_ratpub_name", &EMBED_RATPUB_TURBINE_NAME)?;
            clients.insert(wind_name.clone() + "_pub");
            clients.insert(wind_name);
        }

        coord::run_coordinator(locked_start, clients, eval.rules);
    }

    #[cfg(feature = "embed-ros2-turbine")]
    {
        let wind_name = wind::get_env_or_default("wind_ros2_name", &EMBED_ROS2_TURBINE_NAME)?;

        tokio::spawn(async move {
            let res = wind::ros2::run_dyn_wind(&wind_name).await;
            match res {
                Ok(_) => {} // will never return
                Err(e) => {
                    error!("Error in embedded ROS2 turbine. Reload the App. {e}")
                }
            }
        });
    }

    #[cfg(feature = "embed-ros2-c-turbine")]
    {
        let wind_name = wind::get_env_or_default("wind_ros2_c_name", &EMBED_ROS2_C_TURBINE_NAME)?;

        tokio::spawn(async move {
            let res = wind::ros2_r2r::run_dyn_wind(&wind_name).await;
            match res {
                Ok(_) => {} // will never return
                Err(e) => {
                    error!(
                        "Error in embedded ROS2-C turbine. Reload the App after the fix. Maybe you haven't sourced the message type you tried to publish. {e}"
                    )
                }
            }
        });
    }

    #[cfg(feature = "embed-ros1-turbine")]
    {
        let wind_name = wind::get_env_or_default("wind_ros1_name", &EMBED_ROS1_TURBINE_NAME)?;

        let master_uri = wind::get_env_or_default("ROS_MASTER_URI", "http://localhost:11311")?;

        tokio::spawn(async move {
            let res = wind::ros1::run_dyn_wind(&master_uri, &wind_name).await;
            match res {
                Ok(None) => {
                    log::warn!("ROS1 Master not found. Destroying node.");
                }
                Ok(Some(_)) => {} // will never return
                Err(e) => {
                    error!(
                        "Error in embedded ROS1 turbine. Reload the App to resolve if necessary: {e}"
                    )
                }
            }
        });
    }

    let rules = eval.rules;
    println!("Looking for coordinator..."); // TODO clear and redraw this as animation with running dots while waiting for init
    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat(COMPARE_NODE_NAME.to_string()), None, false)
            .await?;

    // remove searching feedback
    print!("\x1b[1A"); // move cursor up 1 line
    print!("\x1b[2K"); // erase line

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
                        action: crate::Action::Catch { source, id },
                        lock_until_ack: _,
                    } => match comparer.get_cannon().catch_dyn(id).await {
                        Ok(v) => {
                            let name = match &source.kind {
                                ShipKind::Rat(name) => name.clone(),
                                ShipKind::Wind(_) => {
                                    error!("Received something from Wind.");
                                    continue;
                                }
                            };

                            for (strrep, _var_type, var_name) in v {
                                match ndata_tx.send((name.clone(), strrep, var_name)).await {
                                    Ok(_) => {}
                                    Err(e) => {
                                        error!("Error sending received variable: {e}");
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            error!("Could not catch dynamic typed var: {e}");
                        }
                    },
                    _ => (),
                },
                Err(e) => {
                    error!("Error receiving action for comparer: {e}");
                }
            }
        }
    });

    let mut app = App::new(tx, ndata_rx, dyn_wind_rx, file).await;

    info!("Welcome to Minot TUI. Have fun!");
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
