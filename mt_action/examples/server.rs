use log::{info, warn};
use mt_action::{ActionServer, messages::*, runner::Action};
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::select;
use tokio::time::{Duration, sleep};
use tokio_util::sync::CancellationToken;

struct MyAction {}

impl MyAction {
    pub fn new() -> Self {
        Self {}
    }
}

impl Action<msg::String, msg::String, msg::String> for MyAction {
    async fn perform_goal(
        &self,
        goal: &mut mt_action::runner::Goal<msg::String, msg::String>,
        stoken: CancellationToken,
        feedback: mt_action::runner::FeedbackSender<msg::String>,
    ) -> anyhow::Result<()> {
        info!("MyAction is performing goal {}", goal.id);

        // refuse to work on misbehaving goals
        if goal.data.data.contains("MyAction sucks") {
            goal.state = GoalState::Aborted;
            return Ok(());
        }

        // send progress feedback once per second for 5 seconds
        for step in 1..=5u32 {
            select! {
                _ = sleep(Duration::from_secs(1)) => {
                    let pct = step * 20;
                    info!("Goal {} progress: {}%", goal.id, pct);
                    feedback.send(msg::String {
                        data: format!("{}% complete", pct),
                    }).await?;
                }
                _ = stoken.cancelled() => {
                    warn!("Goal {} received cancellation request", goal.id);
                    goal.state = GoalState::Canceled;
                    return Ok(());
                }
            }
        }

        info!("MyAction finished goal {}", goal.id);

        goal.state = GoalState::Succeeded;
        goal.result = Some(msg::String {
            data: format!("Hello from MyAction. You said: {}", goal.data.data),
        });

        Ok(())
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Arc::new(Node::create(NodeConfig::new("server")).await?);

    let action = MyAction::new();

    let action_server = ActionServer::new(
        node.clone(),
        "/my_amazing_action".to_owned(),
        action,
        Qos::Reliable,
    )
    .await?;

    ActionServer::start(action_server, node.shutdown_token()).await?;

    Ok(())
}
