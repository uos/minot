use anyhow::anyhow;
use log::{info, warn};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::process::Stdio;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::process::Command;

type BoxError = Box<dyn std::error::Error>;

#[derive(Serialize, Debug)]
#[serde(tag = "type")]
#[allow(dead_code)]
enum ClientMessage {
    Init {
        file_content: String,
        file_path: PathBuf,
    },
    Command {
        command: TuiCommand,
    },
}

#[derive(Serialize, Debug)]
#[serde(tag = "action")]
#[allow(dead_code)]
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

#[derive(Deserialize, Debug)]
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

#[derive(Deserialize, Debug)]
struct ServerResponse {
    status: String,
    message: Option<String>,
    #[allow(dead_code)]
    data: Option<serde_json::Value>,
}

pub async fn run(file_path: PathBuf, minot_path: PathBuf, sync: bool) -> Result<(), BoxError> {
    let file_content = std::fs::read_to_string(&file_path)
        .map_err(|e| format!("Failed to read file {:?}: {}", file_path, e))?;

    // If the provided minot_path is the default "minot" (no path), prefer the current running
    // executable so headless launched from the built binary uses the same version.
    // #[allow(clippy::cmp_owned)] // clippy warning would be an error in 1.85
    let spawn_path = if minot_path.to_string_lossy() == "minot" {
        std::env::current_exe().unwrap_or(minot_path.clone())
    } else {
        minot_path.clone()
    };

    let mut child = Command::new(&spawn_path)
        .arg("serve")
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|e| format!("Failed to spawn minot at {:?}: {}", minot_path, e))?;

    let stdin = child.stdin.take().expect("Failed to open stdin");
    let stdout = child.stdout.take().expect("Failed to open stdout");
    let mut reader = BufReader::new(stdout);
    let mut writer = tokio::io::BufWriter::new(stdin);

    // Wait for ServerReady
    wait_for_message(&mut reader, |msg| matches!(msg, ServerMessage::ServerReady))
        .await
        .map_err(|e| format!("Failed to wait for ServerReady: {}", e))?;
    info!("Minot server ready");

    // Send Init
    let init_msg = ClientMessage::Init {
        file_content: file_content.clone(),
        file_path: file_path.clone(),
    };
    // debug output removed
    send_message(&mut writer, &init_msg).await?;

    // Wait for Init response
    let resp = wait_for_message(&mut reader, |msg| {
        matches!(msg, ServerMessage::CommandResponse(_))
    })
    .await
    .map_err(|e| format!("Failed to wait for Init response: {}", e))?;

    match resp {
        ServerMessage::CommandResponse(resp) => {
            if resp.status != "ok" {
                return Err(anyhow!("Init failed: {:?}", resp.message).into());
            }
            info!("Init successful: {:?}", resp.message);
        }
        _ => unreachable!(),
    }

    // Wait for user input if sync flag is set
    if sync {
        eprintln!("Waiting for input on stdin before execution (--sync mode)...");
        let mut user_input = String::new();
        let stdin = tokio::io::stdin();
        let mut stdin_reader = BufReader::new(stdin);
        stdin_reader
            .read_line(&mut user_input)
            .await
            .map_err(|e| format!("Failed to read from stdin: {}", e))?;
        eprintln!("Received input, proceeding with execution");
    }

    // After successful Init, send a CompileExecute command so the server compiles the file
    let compile_cmd = ClientMessage::Command {
        command: TuiCommand::CompileExecute {
            file_content: file_content.clone(),
            file_path: file_path.clone(),
        },
    };
    send_message(&mut writer, &compile_cmd).await?;

    // Wait for the compile response (ok/error)
    let compile_resp = wait_for_message(&mut reader, |msg| {
        matches!(msg, ServerMessage::CommandResponse(_))
    })
    .await
    .map_err(|e| format!("Failed to wait for CompileExecute response: {}", e))?;

    // If compilation failed, send Quit and return an error; if it succeeded, keep the server running.
    match compile_resp {
        ServerMessage::CommandResponse(resp) => {
            if resp.status != "ok" {
                let quit_cmd = ClientMessage::Command {
                    command: TuiCommand::Quit,
                };
                let _ = send_message(&mut writer, &quit_cmd).await;
                return Err(anyhow!("Compile failed: {:?}", resp.message).into());
            }
        }
        _ => unreachable!(),
    }

    // Monitor logs
    let mut line = String::new();
    loop {
        line.clear();
        let n = reader.read_line(&mut line).await?;
        if n == 0 {
            break;
        }
        if let Ok(msg) = serde_json::from_str::<ServerMessage>(&line) {
            match msg {
                ServerMessage::LogRecord {
                    level,
                    message,
                    target,
                } => {
                    println!("[{}] {}: {}", level, target, message);
                }
                ServerMessage::CommandResponse(resp) => {
                    info!("Received command response: {:?}", resp);
                }
                ServerMessage::ServerReady => {
                    warn!("Received unexpected ServerReady");
                }
            }
        } else {
            print!("{}", line);
        }
    }

    // Cleanup
    let _ = child.kill().await;
    Ok(())
}

async fn send_message<W: AsyncWriteExt + Unpin>(
    writer: &mut W,
    msg: &ClientMessage,
) -> Result<(), BoxError> {
    let json = serde_json::to_string(msg)?;
    writer.write_all(json.as_bytes()).await?;
    writer.write_all(b"\n").await?;
    writer.flush().await?;
    Ok(())
}

async fn wait_for_message<R: AsyncBufReadExt + Unpin, F>(
    reader: &mut R,
    predicate: F,
) -> Result<ServerMessage, BoxError>
where
    F: Fn(&ServerMessage) -> bool,
{
    let mut line = String::new();
    loop {
        line.clear();
        if reader.read_line(&mut line).await? == 0 {
            return Err(anyhow!("Connection closed while waiting for message").into());
        }
        if let Ok(msg) = serde_json::from_str::<ServerMessage>(&line) {
            match &msg {
                ServerMessage::LogRecord {
                    level,
                    message,
                    target,
                } => {
                    println!("[{}] {}: {}", level, target, message);
                }
                _ => {
                    if predicate(&msg) {
                        return Ok(msg);
                    } else {
                        warn!("Ignored unexpected message while waiting: {:?}", msg);
                    }
                }
            }
        } else {
            print!("{}", line);
        }
    }
}
