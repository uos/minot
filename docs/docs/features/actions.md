# Actions

Actions are for long-running tasks. They provide Goal semantics, streaming Feedback, and a final Result. 

!!! tip "Golang Support"

    The `mt_service` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

`mt_action` implements a full ROS2-compatible action goal lifecycle natively over Minot. It handles:
- **Goal Acceptance**: Negotiating whether a goal is accepted or rejected.
- **Goal Execution**: Running the actual task in the background.
- **Feedback Streaming**: Periodically sending progress updates to the client.
- **Result Reporting**: Returning the final result once the goal reaches a terminal state.
- **Cancellation**: Safely aborting or canceling ongoing goals.

For more detailed implementation details, we recommend checking the `mt_action/examples/` directory in the Minot repository, which contains complete code for action clients and servers matching ROS2 conventions.

## Client

The client sends a goal and monitors its progress.

~~~rust title="action_client.rs"
use anyhow::anyhow;
use log::{info, error};
use mt_action::{ActionClient, messages::*};
use mt_pubsub::{Node, NodeConfig};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::Duration;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Arc::new(Node::create(NodeConfig::new("action_client")).await?);
    
    // Create client (Goal, Feedback, Result types)
    let client: Arc<ActionClient<msg::String, msg::String, msg::String>> =
        ActionClient::new(node.clone(), "/my_action".to_owned()).await?;

    if !ActionClient::wait_for_action_server(client.clone(), Duration::from_secs(5)).await {
        return Err(anyhow!("Action server not ready"));
    }

    // Spawn feedback listener
    let feedback_client = client.clone();
    tokio::spawn(async move {
        while let Ok(fb) = ActionClient::get_feedback(feedback_client.clone()).await {
            info!("Got feedback: {}", fb.msg.data);
        }
    });

    // Send a goal
    let res = ActionClient::send_goal(
        client.clone(),
        ActionSendGoalRequest { id: 1, goal: msg::String { data: "Start task".to_owned() } },
        None
    ).await?;
    
    if res.accepted {
        info!("Goal accepted!");
        // Wait for final result
        let result = ActionClient::get_result(client.clone(), ActionGetResultRequest { id: 1 }, None).await?;
        info!("Final state: {:?}", result.status);
    }

    Ok(())
}
~~~

## Server

The server implements the state machine for processing goals.

The implementation involves creating a struct that implements the `Action` trait and starting the `ActionServer`. See `mt_action/examples/server.rs` for a full boilerplate!
