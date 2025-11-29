use clap::{Parser, Subcommand, command};
mod runner;
use log::{error, info, warn};
use mtc::COMPARE_NODE_NAME;
use ratatui::{Terminal, backend::CrosstermBackend};
use sea::{Action, Cannon, Ship, ShipKind, net::Packet};
use serde::{Deserialize, Serialize};
use std::io::{self, Write};
use std::path::PathBuf;
use tokio::io::{AsyncBufReadExt, BufReader};

use anyhow::anyhow;

pub mod app;
pub mod coord;
pub mod event;
pub mod handler;
pub mod tui;
pub mod ui;

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

#[derive(Debug, Clone, Subcommand)]
pub enum ServeCommand {}

#[derive(Parser, Debug)]
#[command(version, about, author, long_about = None)]
/// Minot — A versatile toolset for debugging and verifying stateful robot perception software.
pub(crate) struct Args {
    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Debug, Clone, Parser)]
pub struct TuiArgs {
    /// Path to a '.mt' file. See the docs for a demo.
    pub file: PathBuf,
}

#[derive(Parser, Debug, Clone)]
pub struct HeadlessArgs {
    /// Path to the .mt file to execute
    pub file: PathBuf,

    /// Path to the minot binary. Defaults to "minot" (expected in PATH).
    #[arg(long, default_value = "minot")]
    pub minot_path: PathBuf,

    /// Wait for stdin input (any line) before executing the file. Useful when waiting for network connections.
    #[arg(long)]
    pub sync: bool,
}

#[derive(Debug, Clone, Subcommand)]
#[command()]
pub(crate) enum Commands {
    /// Run the Terminal UI with all features of Minot (recommended)
    Tui(TuiArgs),
    /// Start the stdin-stdout server for bagfile querying, commonly used in integrations
    Serve,
    /// Run a .mt file in headless (offline) mode, outputting JSON logs
    Headless(HeadlessArgs),
    /// Show compiled features or check if a specific feature is available
    Features {
        /// Optional feature name to check (e.g., "coord", "ratpub", "ros2") or comma-separated list (e.g., "coord,ros2")
        feature: Option<String>,
    },
    // Uninstall minot and minot-coord from the system
    Uninstall,
}

async fn tui(path: PathBuf) -> Result<(), Box<dyn std::error::Error>> {
    println!("Compiling {:#?}", &path.canonicalize()?);
    let eval = mtc::compile_file(&path, None, None)?;
    // remove compile feebback
    print!("\x1b[1A");
    print!("\x1b[2K");

    let log_level = if let Some(rhs) = eval.vars.resolve("_log_level")? {
        match rhs {
            mtc::Rhs::Val(mtc::Val::StringVal(v)) => match v.as_str() {
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

    #[cfg(feature = "embed-coord")]
    {
        let locked_start = if let Some(rhs) = eval.vars.resolve("_start_locked")? {
            match rhs {
                mtc::Rhs::Val(mtc::Val::BoolVal(locked)) => Ok(locked),
                _ => Err(anyhow!("Expected bool for _start_locked.")),
            }
        } else {
            Ok(false)
        }?;

        let coord_file = if let Some(rhs) = eval.vars.resolve("_rules")? {
            match rhs {
                mtc::Rhs::Val(mtc::Val::StringVal(file)) | mtc::Rhs::Path(file) => Ok(Some(file)),
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
                let (tx, rx) = tokio::sync::oneshot::channel();

                tokio::spawn(async move {
                    // dump ready answer
                    _ = rx.await;
                });
                let res = wind::ratpub::run_dyn_wind(&wind_name, tx).await;
                match res {
                    Ok(_) => {} // will never return
                    Err(e) => {
                        error!("Error in embedded ratpub turbine. Reload the App. {e}")
                    }
                }
            });
        }

        let eval = match (&path, coord_file) {
            (cfg_filepath, Some(cfpath)) => {
                let cfg_parent = cfg_filepath
                    .parent()
                    .ok_or(anyhow!("Passed .mt file does not have a parent."))?
                    .canonicalize()
                    .expect("Should have failed in prev compilation.");
                let joined = cfg_parent.join(cfpath);
                println!("Compiling separate wind {:#?}", &joined);

                mtc::compile_file(&joined, None, None)?
            }
            (_, _) => mtc::Evaluated::new(),
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
    println!("Looking for coordinator...");
    let comparer =
        sea::ship::NetworkShipImpl::init(ShipKind::Rat(COMPARE_NODE_NAME.to_string()), None, false)
            .await?;

    // remove searching feedback
    print!("\x1b[1A");
    print!("\x1b[2K");

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

    let mut app = App::new(tx, ndata_rx, dyn_wind_rx, Some(path)).await;

    info!("Welcome to Minot. Have fun!");
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

    tui.exit()
}

fn get_compiled_features() -> Vec<String> {
    let mut features = Vec::new();
    
    #[cfg(feature = "embed-coord")]
    features.push("coord".to_string());
    
    #[cfg(feature = "embed-ratpub")]
    features.push("ratpub".to_string());
    
    #[cfg(feature = "embed-ros1-turbine")]
    features.push("ros1".to_string());
    
    #[cfg(feature = "embed-ros2-turbine")]
    features.push("ros2".to_string());
    
    #[cfg(feature = "embed-ros2-c-turbine")]
    features.push("ros2-c".to_string());
    
    features
}

fn has_feature(feature_name: &str) -> bool {
    let normalized = feature_name.to_lowercase();
    
    match normalized.as_str() {
        "coord" => {
            #[cfg(feature = "embed-coord")]
            return true;
            #[cfg(not(feature = "embed-coord"))]
            return false;
        }
        "ratpub" => {
            #[cfg(feature = "embed-ratpub-turbine")]
            return true;
            #[cfg(not(feature = "embed-ratpub-turbine"))]
            return false;
        }
        "ros1" => {
            #[cfg(feature = "embed-ros1-turbine")]
            return true;
            #[cfg(not(feature = "embed-ros1-turbine"))]
            return false;
        }
        "ros2" => {
            #[cfg(feature = "embed-ros2-turbine")]
            return true;
            #[cfg(not(feature = "embed-ros2-turbine"))]
            return false;
        }
        "ros2-c" | "ros2_c" => {
            #[cfg(feature = "embed-ros2-c-turbine")]
            return true;
            #[cfg(not(feature = "embed-ros2-c-turbine"))]
            return false;
        }
        _ => false,
    }
}

fn features_command(feature: Option<String>) -> Result<(), Box<dyn std::error::Error>> {
    match feature {
        Some(feature_names) => {
            let features: Vec<&str> = feature_names.split(',').map(|s| s.trim()).collect();
            
            let mut all_present = true;
            for feature_name in features {
                if !has_feature(feature_name) {
                    all_present = false;
                    break;
                }
            }
            
            if all_present {
                std::process::exit(0);
            } else {
                std::process::exit(1);
            }
        }
        None => {
            let features = get_compiled_features();
            if features.is_empty() {
                println!("No embedded features compiled.");
            } else {
                println!("Compiled features:");
                for feature in features {
                    println!("  - {}", feature);
                }
            }
            Ok(())
        }
    }
}

fn uninstall() -> Result<(), Box<dyn std::error::Error>> {
    use std::fs;

    let current_exe = std::env::current_exe()?;
    let install_dir = current_exe.parent().ok_or_else(|| {
        anyhow::anyhow!(
            "Failed to get installation directory from executable path: {:?}",
            current_exe
        )
    })?;

    let minot_path = &current_exe;
    let minot_coord_path = install_dir.join("minot-coord");

    let mut failed = false;

    println!("Attempting to remove: {:?}", minot_path);
    match fs::remove_file(minot_path) {
        Ok(_) => println!("Successfully unlinked: {:?}", minot_path),
        Err(e) => {
            eprintln!("ERROR: Failed to remove: {:?}. Error: {}", minot_path, e);
            eprintln!("You may need to remove this file manually.");
            failed = true;
        }
    }

    if minot_coord_path.exists() {
        println!("Attempting to remove: {:?}", minot_coord_path);
        match fs::remove_file(&minot_coord_path) {
            Ok(_) => println!("Successfully removed: {:?}", minot_coord_path),
            Err(e) => {
                eprintln!(
                    "ERROR: Failed to remove: {:?}. Error: {}",
                    minot_coord_path, e
                );
                eprintln!("You may need to remove this file manually.");
                failed = true;
            }
        }
    } else {
        println!("'minot-coord' not found, skipping.");
    }

    if !failed {
        println!("Successfully removed Minot binaries.");
    } else {
        println!("Some files could not be removed. Please check the errors above.");
    }

    Ok(())
}

#[derive(Serialize, Debug)]
#[serde(tag = "type")]
enum ServerMessage {
    ServerReady,
    CommandResponse(ServerResponse),
    LogRecord {
        level: String,
        message: String,
        target: String,
    },
}

#[derive(Deserialize, Serialize, Debug)]
#[serde(tag = "type")]
enum ClientMessage {
    Init {
        file_content: String,
        file_path: PathBuf,
    },
    Command {
        command: TuiCommand,
    },
}

#[derive(Deserialize, Serialize, Debug)]
#[serde(tag = "action")] // Use an "action" field for the command name
enum TuiCommand {
    Quit,
    SendUnlock,
    SendLockNext {
        previous: bool,
    },
    CompileExecute {
        file_content: String,
        file_path: PathBuf,
    },
    ClearRules,
}

#[derive(Debug, Serialize)]
struct ServerResponse {
    status: String, // "ok", "error"
    #[serde(skip_serializing_if = "Option::is_none")]
    message: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    data: Option<serde_json::Value>,
}

impl ServerResponse {
    fn ok(message: Option<String>) -> Self {
        ServerResponse {
            status: "ok".to_string(),
            message,
            data: None,
        }
    }

    fn error(message: String) -> Self {
        ServerResponse {
            status: "error".to_string(),
            message: Some(message),
            data: None,
        }
    }
}

struct StdioLogger {
    stdout: std::sync::Arc<std::sync::Mutex<io::StdoutLock<'static>>>,
}

unsafe impl Send for StdioLogger {}
unsafe impl Sync for StdioLogger {}

impl log::Log for StdioLogger {
    fn enabled(&self, metadata: &log::Metadata) -> bool {
        // We can filter levels here, but it's easier
        // to just set the global `log::set_max_level`.
        // This logger will respect that global level.
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &log::Record) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let log_message = ServerMessage::LogRecord {
            level: record.level().to_string(),
            message: format!("{}", record.args()),
            target: record.target().to_string(),
        };

        let _ = write_json_message(&self.stdout, &log_message);
    }

    fn flush(&self) {
        if let Ok(mut handle) = self.stdout.lock() {
            let _ = handle.flush();
        }
    }
}

fn init_stdio_logger(
    stdout_handle: std::sync::Arc<std::sync::Mutex<io::StdoutLock<'static>>>,
    level: log::LevelFilter,
) -> Result<(), log::SetLoggerError> {
    let logger = StdioLogger {
        stdout: stdout_handle,
    };
    log::set_boxed_logger(Box::new(logger))?;
    log::set_max_level(level);
    Ok(())
}

async fn serve() -> Result<(), Box<dyn std::error::Error>> {
    let stdout = io::stdout();
    let stdout_lock = stdout.lock();
    let stdout_handle = std::sync::Arc::new(std::sync::Mutex::new(stdout_lock));
    init_stdio_logger(
        std::sync::Arc::clone(&stdout_handle),
        log::LevelFilter::Info,
    )
    .expect("Failed to initialize logger.");

    let mut app_state = None;

    let ready_message = ServerMessage::ServerReady;
    if write_json_message(&stdout_handle, &ready_message).is_err() {
        eprintln!(
            "[Server CRITICAL] Failed to write initial ServerReady message to stdout. Exiting."
        );
        return Err("Failed to write to stdout".into());
    }
    info!("Waiting for 'Init' message.");

    let (stdin_tx, mut stdin_rx) = tokio::sync::mpsc::unbounded_channel::<String>();
    tokio::spawn(async move {
        let stdin = tokio::io::stdin();
        let reader = BufReader::new(stdin);
        let mut lines = reader.lines();
        while let Ok(Some(line)) = lines.next_line().await {
            if stdin_tx.send(line).is_err() {
                break; // receiver dropped
            }
        }
    });

    loop {
        let input = match stdin_rx.recv().await {
            Some(line) => line,
            None => break, // stdin closed
        };

        let input = input.trim();
        if input.is_empty() {
            continue;
        }

        let response = match serde_json::from_str::<ClientMessage>(&input) {
            Ok(client_message) => {
                match (&mut app_state, client_message) {
                    (
                        None,
                        ClientMessage::Init {
                            file_content,
                            file_path,
                        },
                    ) => {
                        let lines = file_content.lines();
                        let end_line = lines.clone().count().saturating_sub(1);
                        let selected_lines =
                            lines.take(end_line + 1).collect::<Vec<_>>().join("\n") + "\n"; // append newline as funny hack to fix one-line problems

                        let dir = file_path.parent().ok_or(anyhow!(
                            "Could not get parent directory of source code file."
                        ))?;
                        let eval = match mtc::compile_code_with_state(
                            &selected_lines,
                            dir,
                            None,
                            app::ErrorWriter {},
                            false,
                        ) {
                            Ok(eval) => eval,
                            Err(e) => {
                                error!("{}", e);
                                continue;
                            }
                        };

                        // let (tx, rx) = tokio::sync::oneshot::channel();

                        // tokio::spawn(async move {
                        //     // dump ready answer
                        //     _ = rx.await;
                        // });
                        #[cfg(feature = "embed-coord")]
                        {
                            let locked_start =
                                if let Some(rhs) = eval.vars.resolve("_start_locked")? {
                                    match rhs {
                                        mtc::Rhs::Val(mtc::Val::BoolVal(locked)) => Ok(locked),
                                        _ => Err(anyhow!("Expected bool for _start_locked.")),
                                    }
                                } else {
                                    Ok(false)
                                }?;

                            let coord_file = if let Some(rhs) = eval.vars.resolve("_rules")? {
                                match rhs {
                                    mtc::Rhs::Val(mtc::Val::StringVal(file))
                                    | mtc::Rhs::Path(file) => Ok(Some(file)),
                                    _ => Err(anyhow!("Expected _wind_file to be path or string.")),
                                }
                            } else {
                                Ok(None)
                            }?;

                            let eval = match (&file_path, coord_file) {
                                (cfg_filepath, Some(cfpath)) => {
                                    let cfg_parent = cfg_filepath
                                        .parent()
                                        .ok_or(anyhow!("Passed .mt file does not have a parent."))?
                                        .canonicalize()
                                        .expect("Should have failed in prev compilation.");
                                    let joined = cfg_parent.join(cfpath);
                                    println!("Compiling separate wind {:#?}", &joined);

                                    mtc::compile_file(&joined, None, None)?
                                }
                                (_, _) => mtc::Evaluated::new(),
                            };

                            #[allow(unused_mut)]
                            let mut clients = coord::get_clients(&eval)?;

                            #[cfg(feature = "embed-ros2-turbine")]
                            {
                                let wind_name = wind::get_env_or_default(
                                    "wind_ros2_name",
                                    &EMBED_ROS2_TURBINE_NAME,
                                )?;
                                clients.insert(wind_name);
                            }

                            #[cfg(feature = "embed-ros2-c-turbine")]
                            {
                                let wind_name = wind::get_env_or_default(
                                    "wind_ros2_c_name",
                                    &EMBED_ROS2_C_TURBINE_NAME,
                                )?;
                                clients.insert(wind_name);
                            }

                            #[cfg(feature = "embed-ros1-turbine")]
                            {
                                let wind_name = wind::get_env_or_default(
                                    "wind_ros1_name",
                                    &EMBED_ROS1_TURBINE_NAME,
                                )?;
                                clients.insert(wind_name);
                            }

                            #[cfg(feature = "embed-ratpub-turbine")]
                            {
                                let wind_name = wind::get_env_or_default(
                                    "wind_ratpub_name",
                                    &EMBED_RATPUB_TURBINE_NAME,
                                )?;
                                clients.insert(wind_name.clone() + "_pub");
                                clients.insert(wind_name);
                            }

                            coord::run_coordinator(locked_start, clients, eval.rules);
                        }

                        #[cfg(feature = "embed-ratpub-turbine")]
                        let ratpub_ready_rx = {
                            let wind_name = wind::get_env_or_default(
                                "ratpub_wind_name",
                                &EMBED_RATPUB_TURBINE_NAME,
                            )?;

                            let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
                            tokio::spawn(async move {
                                let res = wind::ratpub::run_dyn_wind(&wind_name, ready_tx).await;
                                match res {
                                    Ok(_) => {} // will never return
                                    Err(e) => {
                                        error!(
                                            "Error in embedded ratpub turbine. Reload the App. {e}"
                                        )
                                    }
                                }
                            });
                            Some(ready_rx)
                        };
                        #[cfg(not(feature = "embed-ratpub-turbine"))]
                        let ratpub_ready_rx: Option<
                            tokio::sync::oneshot::Receiver<()>,
                        > = None;

                        #[cfg(feature = "embed-ros2-turbine")]
                        let ros2_ready_rx = {
                            let wind_name = wind::get_env_or_default(
                                "wind_ros2_name",
                                &EMBED_ROS2_TURBINE_NAME,
                            )?;

                            let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
                            tokio::spawn(async move {
                                let res = wind::ros2::run_dyn_wind(&wind_name, ready_tx).await;
                                match res {
                                    Ok(_) => {} // will never return
                                    Err(e) => {
                                        error!(
                                            "Error in embedded ROS2 turbine. Reload the App. {e}"
                                        )
                                    }
                                }
                            });
                            Some(ready_rx)
                        };
                        #[cfg(not(feature = "embed-ros2-turbine"))]
                        let ros2_ready_rx: Option<
                            tokio::sync::oneshot::Receiver<()>,
                        > = None;

                        #[cfg(feature = "embed-ros2-c-turbine")]
                        let ros2_c_ready_rx = {
                            let wind_name = wind::get_env_or_default(
                                "wind_ros2_c_name",
                                &EMBED_ROS2_C_TURBINE_NAME,
                            )?;

                            let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
                            tokio::spawn(async move {
                                let res = wind::ros2_r2r::run_dyn_wind(&wind_name, ready_tx).await;
                                match res {
                                    Ok(_) => {} // will never return
                                    Err(e) => {
                                        error!(
                                            "Error in embedded ROS2-C turbine. Reload the App after the fix. Maybe you haven't sourced the message type you tried to publish. {e}"
                                        )
                                    }
                                }
                            });
                            Some(ready_rx)
                        };
                        #[cfg(not(feature = "embed-ros2-c-turbine"))]
                        let ros2_c_ready_rx: Option<
                            tokio::sync::oneshot::Receiver<()>,
                        > = None;

                        #[cfg(feature = "embed-ros1-turbine")]
                        let ros1_ready_rx = {
                            let wind_name = wind::get_env_or_default(
                                "wind_ros1_name",
                                &EMBED_ROS1_TURBINE_NAME,
                            )?;

                            let master_uri = wind::get_env_or_default(
                                "ROS_MASTER_URI",
                                "http://localhost:11311",
                            )?;

                            let (ready_tx, ready_rx) = tokio::sync::oneshot::channel();
                            tokio::spawn(async move {
                                let res =
                                    wind::ros1::run_dyn_wind(&master_uri, &wind_name, ready_tx)
                                        .await;
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
                            Some(ready_rx)
                        };
                        #[cfg(not(feature = "embed-ros1-turbine"))]
                        let ros1_ready_rx: Option<
                            tokio::sync::oneshot::Receiver<()>,
                        > = None;

                        let rules = eval.rules;
                        info!("Looking for coordinator...");
                        let comparer = sea::ship::NetworkShipImpl::init(
                            ShipKind::Rat(COMPARE_NODE_NAME.to_string()),
                            None,
                            false,
                        )
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
                                                    error!(
                                                        "Could forward var request for wind: {e}"
                                                    );
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
                                                    match ndata_tx
                                                        .send((name.clone(), strrep, var_name))
                                                        .await
                                                    {
                                                        Ok(_) => {}
                                                        Err(e) => {
                                                            error!(
                                                                "Error sending received variable: {e}"
                                                            );
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

                        let app = App::new(tx, ndata_rx, dyn_wind_rx, Some(file_path)).await;

                        // Wait for all winds to be connected before responding
                        let mut ready_futures = vec![];
                        if let Some(rx) = ratpub_ready_rx {
                            ready_futures.push(tokio::spawn(async move {
                                match rx.await {
                                    Ok(_) => info!("ratpub wind ready"),
                                    Err(_) => warn!("ratpub wind ready signal dropped"),
                                }
                            }));
                        }
                        if let Some(rx) = ros2_ready_rx {
                            ready_futures.push(tokio::spawn(async move {
                                match rx.await {
                                    Ok(_) => info!("ros2 wind ready"),
                                    Err(_) => warn!("ros2 wind ready signal dropped"),
                                }
                            }));
                        }
                        if let Some(rx) = ros2_c_ready_rx {
                            ready_futures.push(tokio::spawn(async move {
                                match rx.await {
                                    Ok(_) => info!("ros2-c wind ready"),
                                    Err(_) => warn!("ros2-c wind ready signal dropped"),
                                }
                            }));
                        }
                        if let Some(rx) = ros1_ready_rx {
                            ready_futures.push(tokio::spawn(async move {
                                match rx.await {
                                    Ok(_) => info!("ros1 wind ready"),
                                    Err(_) => warn!("ros1 wind ready signal dropped"),
                                }
                            }));
                        }

                        // Wait for all winds to signal ready
                        for future in ready_futures {
                            let _ = future.await;
                        }

                        info!("Connection established. Minot initialized.");

                        app_state = Some(app);
                        ServerResponse::ok(Some("Initialized.".to_string()))
                    }
                    (Some(state), ClientMessage::Command { command }) => {
                        handle_command(command, state).await
                    }
                    (None, ClientMessage::Command { .. }) => ServerResponse::error(
                        "Server not initialized. Please send 'Init' message first.".to_string(),
                    ),
                    (Some(_), ClientMessage::Init { .. }) => ServerResponse::error(
                        "Server already initialized. Cannot re-initialize.".to_string(),
                    ),
                }
            }
            Err(e) => ServerResponse::error(format!("Failed to parse JSON message: {}", e)),
        };
        let message_to_send = ServerMessage::CommandResponse(response);

        if write_json_message(&stdout_handle, &message_to_send).is_err() {
            eprintln!("[Server CRITICAL] Failed to write to stdout. Exiting.");
            break;
        }
    }

    eprintln!("[Server LOG] Client disconnected. Shutting down.");
    Ok(())
}

async fn handle_command(command: TuiCommand, app: &mut App) -> ServerResponse {
    match command {
        TuiCommand::Quit => {
            std::process::exit(0);
        }
        TuiCommand::SendUnlock => {
            app.send_unlock().await;
            app.set_var_unlocked();
            ServerResponse::ok(Some("Unlocked.".to_string()))
        }
        TuiCommand::SendLockNext { previous } => {
            app.send_lock_next(previous).await;
            app.set_var_locked();
            ServerResponse::ok(Some(format!("Locked next (previous: {}).", previous)))
        }
        TuiCommand::ClearRules => {
            app.clear_rules().await;
            ServerResponse::ok(Some("Rules cleared.".to_string()))
        }
        TuiCommand::CompileExecute {
            file_content,
            file_path,
        } => {
            let start_compile = std::time::Instant::now();
            let var_state = {
                let mut wc = app.wind_cursor.write().unwrap();
                wc.compile_state = app::WindCompile::Compiling;
                wc.variable_cache.clone()
            };

            let lines = file_content.lines();
            let selected_lines = lines.collect::<Vec<_>>().join("\n") + "\n"; // append newline as funny hack to fix one-line problems

            let dir = file_path.parent().ok_or(anyhow!(
                "Could not get parent directory of source code file."
            ));
            let dir = match dir {
                Ok(dir) => dir,
                Err(e) => {
                    return ServerResponse::error(e.to_string());
                }
            };
            let eval = mtc::compile_code_with_state(
                &selected_lines,
                dir,
                Some(var_state),
                app::ErrorWriter {},
                false,
            );

            let mut eval = match eval {
                Ok(eval) => eval,
                Err(e) => {
                    return ServerResponse::error(e.to_string());
                }
            };

            {
                let mut wc = app.wind_cursor.write().unwrap();
                eval.vars.populate_cache();
                wc.variable_cache = eval.vars.var_cache.clone();
                wc.compile_state = app::WindCompile::Done;
            }
            info!("compiled in {:?}", start_compile.elapsed());

            let mut vars = 0;
            for (var, rules) in eval.rules.raw() {
                vars += 1;
                app.send_coordinator
                    .send(sea::net::PacketKind::RuleAppend {
                        variable: var.clone(),
                        commands: rules.clone(),
                    })
                    .await
                    .unwrap();
            }

            if vars != 0 {
                info!("sent {vars} new rules");
            }

            {
                let mut wind_cursor = app.wind_cursor.write().unwrap();
                let wind_queue = app.wind_worker_tx.clone();
                match wind_cursor.wind_send_state {
                    app::WindSendState::Acked | app::WindSendState::Sent => {
                        wind_queue.send(eval).unwrap();
                        wind_cursor.wind_work_queue += 1;
                    }
                    app::WindSendState::NotRun => {
                        wind_queue.send(eval).unwrap();
                        wind_cursor.wind_work_queue += 1;
                        wind_cursor.wind_send_state = app::WindSendState::Sent;
                    }
                }
            }

            ServerResponse::ok(Some(format!("Enqueued compilation.",)))
        }
    }
}

fn write_json_message(
    stdout: &std::sync::Arc<std::sync::Mutex<io::StdoutLock<'static>>>,
    message: &ServerMessage,
) -> io::Result<()> {
    let response_json = match serde_json::to_string(message) {
        Ok(json) => json,
        Err(e) => {
            let error_msg = ServerMessage::LogRecord {
                level: "ERROR".to_string(),
                message: format!("Failed to serialize server message: {}", e),
                target: "server::write_json_message".to_string(),
            };
            serde_json::to_string(&error_msg).unwrap_or_else(|_| {
                r#"{"type":"LogRecord","level":"ERROR","message":"Unserializable error"}"#
                    .to_string()
            })
        }
    };

    // Lock the mutex to get exclusive access to stdout
    let mut handle = stdout.lock().map_err(|e| {
        io::Error::new(
            io::ErrorKind::Other,
            format!("Stdout mutex was poisoned: {}", e),
        )
    })?;

    handle.write_all(response_json.as_bytes())?;
    handle.write_all(b"\n")?;
    handle.flush()?; // CRITICAL: Flush to send immediately

    Ok(())
}

#[tokio::main(flavor = "multi_thread")]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();

    match args.command {
        Commands::Tui(tui_args) => tui(tui_args.file).await,
        Commands::Serve => serve().await,
        Commands::Headless(headless_args) => {
            runner::run(headless_args.file, headless_args.minot_path, headless_args.sync).await
        }
        Commands::Features { feature } => features_command(feature),
        Commands::Uninstall => uninstall(),
    }
}
