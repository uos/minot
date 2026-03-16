use log::{error, info, warn};
use mt_action::{
    ActionServer,
    messages::*,
    runner::{Action, Goal},
};
use mt_pubsub::{Node, NodeConfig, Publisher, Qos};
use ros2_interfaces_jazzy_rkyv::geometry_msgs;
use ros2_interfaces_jazzy_rkyv::std_msgs;
use std::sync::Arc;
use tokio::select;
use tokio::time::{Duration, interval, sleep};
use tokio_util::sync::CancellationToken;

struct RobotAction {
    /// publisher for /cmd_vel
    pubber_cmd_vel: Publisher<geometry_msgs::msg::Twist>,
}

impl RobotAction {
    /**
     * Create new MyAction
     */
    pub async fn new(node: Arc<Node>) -> anyhow::Result<Self> {
        let pubber_cmd_vel = node
            .create_publisher("/cmd_vel".to_owned(), Qos::Reliable)
            .await?;
        Ok(Self { pubber_cmd_vel })
    }
}

impl Action<std_msgs::msg::String, (u64, geometry_msgs::msg::Twist), std_msgs::msg::String>
    for RobotAction
{
    async fn perform_goal(
        &self,
        goal: &mut Goal<(u64, geometry_msgs::msg::Twist), std_msgs::msg::String>,
        stoken: CancellationToken,
        _feedback: mt_action::runner::FeedbackSender<std_msgs::msg::String>,
    ) -> anyhow::Result<()> {
        info!(
            "RobotAction is peforming goal {} for 3 seconds: {:?}",
            goal.id, goal.data
        );

        let duration = goal.data.0;
        let twist = goal.data.1.clone();
        let mut interval = interval(Duration::from_millis(100));

        info!("Publishing {:?} to /cmd_vel at 100Hz", &twist);
        select! {
            // goal cancelled
            _ = stoken.cancelled() => {
                warn!("Goal cancelled");
                goal.state = GoalState::Canceled;
                return Ok(());
            }

            // goal finished after provided duration
            _ = sleep(Duration::from_millis(duration)) => {}

            // actually performs the publishing and
            // *should* stop when another branch finishes
            // because that drops the future
            _ = async {
                loop {
                    if self.pubber_cmd_vel.publish(&twist).await.is_err() {
                        error!("Error publishing to /cmd_vel");
                        goal.state = GoalState::Aborted;
                        return;
                    }
                    interval.tick().await;
                }
            } => {}
        }

        if goal.state == GoalState::Aborted {
            return Ok(());
        }

        // explicit stop afterwards
        // not sure if this is needed but whatever
        let twist = geometry_msgs::msg::Twist {
            linear: geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            angular: geometry_msgs::msg::Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
        };

        if self.pubber_cmd_vel.publish(&twist).await.is_err() {
            error!("Error publishing to /cmd_vel");
            goal.state = GoalState::Aborted;
            return Ok(());
        }

        goal.state = GoalState::Succeeded;
        goal.result = Some(std_msgs::msg::String {
            data: format!("Successfully moved for {} milliseconds", duration),
        });

        Ok(())
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Arc::new(Node::create(NodeConfig::new("robot_server")).await?);

    let action = RobotAction::new(node.clone()).await?;

    let action_server = ActionServer::new(
        node.clone(),
        "/robot_action".to_owned(),
        action,
        Qos::Reliable,
    )
    .await?;

    let shutdown = CancellationToken::new();
    ActionServer::start(action_server, shutdown).await?;

    Ok(())
}
