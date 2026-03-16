use anyhow::anyhow;
use log::{error, info, warn};
use mt_action::ActionClient;
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::Duration;

use mt_action::messages::*;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    mt_sea::init_logging();

    let node = Arc::new(Node::create(NodeConfig::new("client")).await?);

    let client: Arc<ActionClient<msg::String, msg::String, msg::String>> =
        ActionClient::new(node.clone(), "/my_amazing_action".to_owned(), Qos::Reliable).await?;

    if !ActionClient::wait_for_action_server(client.clone(), Duration::from_secs(5)).await {
        return Err(anyhow!("Action server not ready after 5s"));
    }

    info!("Action server ready");

    let goals = [
        (0, "Hello server"),
        (1, "wow"),
        (2, "so many goals"),
        (0, "duplicate"),
        (3, "MyAction sucks >:p"),
        (4, "whoops cancel this one pls"),
    ];

    let status_client = client.clone();
    let status_handle = tokio::spawn(async move {
        while let Ok(status) = ActionClient::get_status(status_client.clone()).await {
            info!("Got status update: {:?}", status);
        }
    });

    let feedback_client = client.clone();
    let feedback_handle = tokio::spawn(async move {
        while let Ok(feedback) = ActionClient::get_feedback(feedback_client.clone()).await {
            info!(
                "Got feedback update for {}: {}",
                feedback.id, feedback.msg.data
            );
        }
    });

    info!("Sending some goal request");
    for goal in goals.iter() {
        info!("Sending: {:?}", &goal);
        match ActionClient::send_goal(
            client.clone(),
            ActionSendGoalRequest {
                id: goal.0,
                goal: msg::String {
                    data: goal.1.to_string(),
                },
            },
            None,
        )
        .await
        {
            Ok(res) => {
                if res.accepted {
                    info!("{:?}", res);
                } else {
                    error!("{:?}", res);
                }
            }
            Err(e) => {
                error!("{:?}", e);
            }
        }
    }

    info!("Cancel goal 4");
    match ActionClient::cancel_goal(
        client.clone(),
        ActionCancelGoalRequest {
            id: Some(4),
            time: None,
        },
        None,
    )
    .await
    {
        Ok(res) => {
            info!("{:?}", res);
        }
        Err(e) => {
            error!("{:?}", e);
        }
    }

    info!("Waiting for the results");
    for goal in goals.iter() {
        info!("Requesting: {:?}", &goal.0);
        match ActionClient::get_result(client.clone(), ActionGetResultRequest { id: goal.0 }, None)
            .await
        {
            Ok(res) => match res.status {
                GoalState::Aborted => {
                    error!("{:?}", res);
                }
                GoalState::Canceled => {
                    warn!("{:?}", res);
                }
                _ => {
                    info!("{:?}", res);
                }
            },
            Err(e) => {
                error!("{:?}", e);
            }
        }
    }

    // we don't need these anymore
    status_handle.abort();
    feedback_handle.abort();

    Ok(())
}
