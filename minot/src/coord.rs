use std::collections::HashSet;
use std::path::PathBuf;

use anyhow::anyhow;
use clap::Parser;
use log::{info, warn};

use mt_mtc::{Evaluated, Rhs, Val, VariableHistory};
use mt_net::{ActionPlan, Rules};

pub use mt_coord::run_coordinator;

#[derive(Parser, Debug)]
#[command(version, about, author, long_about = None)]
/// Minot Coordinator — Network Manager for MtPubsub Nodes
pub struct Args {
    /// Restrict Minot discovery and communication to this machine.
    #[arg(long, global = true)]
    pub local_only: bool,

    /// Path to .mt file for initialization
    pub file: Option<PathBuf>,
}

pub fn topic_from_eval_or_default(
    eval: &Evaluated,
    path: &str,
    default: &str,
) -> anyhow::Result<String> {
    let topic = eval.vars.resolve(path)?;
    if let Some(topic) = topic {
        match topic {
            Rhs::Path(topic) | Rhs::Val(Val::StringVal(topic)) => Ok(topic),
            _ => Err(anyhow!("Expected String or Path for sending lidar topic")),
        }
    } else {
        Ok(default.to_owned())
    }
}

pub fn get_clients(eval: &Evaluated) -> anyhow::Result<HashSet<String>> {
    let mut clients = HashSet::new();

    // ships from rules
    for (_, inner_clients) in eval.rules.raw().iter() {
        inner_clients.iter().for_each(|client| {
            if let Some(strat) = &client.strategy {
                match strat {
                    ActionPlan::Sail => {}
                    ActionPlan::Shoot { target, id: _ } => {
                        clients.extend(target.clone());
                    }
                    ActionPlan::Catch { source, id: _ } => {
                        clients.insert(source.clone());
                    }
                }
            };
            clients.insert(client.ship.clone());
        });
    }

    // find all winds using variables
    let winds = eval.vars.resolve("_wind")?;
    let winds = if let Some(winds) = winds {
        match winds {
            Rhs::Array(items)
                if items
                    .iter()
                    .all(|item| matches!(**item, Rhs::Val(Val::StringVal(_)))) =>
            {
                items
                    .into_iter()
                    .map(|item| match *item {
                        Rhs::Val(Val::StringVal(wind)) => wind,
                        _ => {
                            unreachable!("Catched in higher match.")
                        }
                    })
                    .collect::<Vec<_>>()
            }
            Rhs::Val(Val::StringVal(single_wind)) => {
                vec![single_wind]
            }
            _ => {
                return Err(anyhow::anyhow!(
                    "Expected _wind variable to be String or Array<String>"
                ));
            }
        }
    } else {
        vec![]
    };

    clients.extend(winds);

    Ok(clients)
}

#[tokio::main(flavor = "multi_thread")]
pub async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let args = Args::parse();
    mt_sea::network::set_local_only(args.local_only);
    let filepath = args.file;

    let eval = if let Some(fp) = filepath {
        let rules_file = std::fs::canonicalize(&fp)?;
        mt_mtc::compile_file(&rules_file, None, None)?
    } else {
        mt_mtc::Evaluated {
            rules: Rules::new(),
            wind: Vec::new(),
            vars: VariableHistory::new(Vec::new()),
        }
    };

    let locked_start = eval.vars.resolve("_start_locked")?;
    let locked_start = if let Some(rhs) = locked_start {
        match rhs {
            Rhs::Val(Val::BoolVal(locked)) => Ok(locked),
            _ => Err(anyhow!("Expected bool for _start_locked.")),
        }
    } else {
        Ok(false)
    }?;

    let clients = get_clients(&eval)?;
    let (torpedo_tx, mut torpedo_rx) = tokio::sync::mpsc::channel::<()>(1);

    if !mt_coord::try_start_with_rules(locked_start, clients, eval.rules, Some(torpedo_tx)) {
        warn!("Another coordinator instance is already running. Exiting.");
        return Ok(());
    }

    tokio::select! {
        _ = tokio::signal::ctrl_c() => {
            info!("Ctrl-C received. Shutting down...");
        }
        _ = torpedo_rx.recv() => {
            info!("Torpedo received. Shutting down...");
        }
    }

    Ok(())
}
