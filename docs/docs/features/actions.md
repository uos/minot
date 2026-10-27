# Actions

Actions are for long-running tasks. They provide Goal semantics, streaming Feedback, and a final Result.

!!! tip "Golang Support"

    The `mt_action` library is written in Rust but it can be used from Golang as well. See our [Go Guide](./api.md) on how to create a Go project for Minot.

`mt_action` implements the ROS2 action goal lifecycle natively over Minot [Services](./services.md) and [Pub/Sub](./pubsub.md). It handles:

- **Goal Acceptance**: The client picks the goal ID. A goal whose ID is already known is rejected.
- **Goal Execution**: Goals run one after another in the background, in the order they were accepted.
- **Feedback Streaming**: The running goal sends progress updates to every client.
- **Result Reporting**: A result request waits until the goal reaches a terminal state, then returns it once.
- **Cancellation**: Canceling one goal, all goals accepted before a given time, or all goals.

An action on `/my_action` uses these topics:

| Topic | Kind |
|---|---|
| `/my_action/_action/send_goal` | service |
| `/my_action/_action/cancel_goal` | service |
| `/my_action/_action/get_result` | service |
| `/my_action/_action/feedback` | topic, with the QoS you choose |
| `/my_action/_action/status` | topic, reliable |

A goal moves through these states:

| State | Meaning |
|---|---|
| `Accepted` | Waiting to run |
| `Executing` | Running |
| `Canceling` | A cancel request arrived, the action is stopping |
| `Succeeded`, `Aborted`, `Canceled` | Terminal, the result is ready |

## Server

An action is a struct that implements the `Action` trait. `perform_goal` runs one goal. It sends feedback through `feedback`, watches `stoken` for cancellation, and must leave the goal in a terminal state.

~~~rust title="action_server.rs"
use mt_action::runner::{Action, FeedbackSender, Goal};
use mt_action::{ActionServer, messages::GoalState};
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::select;
use tokio::time::{Duration, sleep};
use tokio_util::sync::CancellationToken;

struct Countdown;

// Feedback, goal and result types
impl Action<msg::String, msg::String, msg::String> for Countdown {
    async fn perform_goal(
        &self,
        goal: &mut Goal<msg::String, msg::String>,
        stoken: CancellationToken,
        feedback: FeedbackSender<msg::String>,
    ) -> anyhow::Result<()> {
        for step in 1..=5u32 {
            select! {
                _ = sleep(Duration::from_secs(1)) => {
                    feedback.send(msg::String { data: format!("{}% complete", step * 20) }).await?;
                }
                _ = stoken.cancelled() => {
                    goal.state = GoalState::Canceled;
                    return Ok(());
                }
            }
        }

        goal.state = GoalState::Succeeded;
        goal.result = Some(msg::String {
            data: format!("Done with: {}", goal.data.data),
        });
        Ok(())
    }
}

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Arc::new(Node::create(NodeConfig::new("action_server")).await?);

    let server = ActionServer::new(
        node.clone(),
        "/my_action".to_owned(),
        Countdown,
        Qos::Reliable, // feedback QoS
    )
    .await?;

    // Runs until the node shuts down.
    ActionServer::start(server, node.shutdown_token()).await?;

    Ok(())
}
~~~

To abort a goal, set `goal.state = GoalState::Aborted` and return `Ok(())`.

## Client

The client sends a goal, listens to its feedback, and waits for the result.

~~~rust title="action_client.rs"
use anyhow::anyhow;
use mt_action::{ActionClient, messages::*};
use mt_pubsub::{Node, NodeConfig, Qos};
use ros2_interfaces_jazzy_rkyv::std_msgs::msg;
use std::sync::Arc;
use tokio::time::Duration;

#[tokio::main]
async fn main() -> anyhow::Result<()> {
    let node = Arc::new(Node::create(NodeConfig::new("action_client")).await?);

    // Feedback, goal and result types, and the feedback QoS
    let client: Arc<ActionClient<msg::String, msg::String, msg::String>> =
        ActionClient::new(node.clone(), "/my_action".to_owned(), Qos::Reliable).await?;

    if !ActionClient::wait_for_action_server(client.clone(), Duration::from_secs(5)).await {
        return Err(anyhow!("Action server not ready"));
    }

    // Feedback of every running goal
    let feedback_client = client.clone();
    tokio::spawn(async move {
        while let Ok(fb) = ActionClient::get_feedback(feedback_client.clone()).await {
            println!("Goal {}: {}", fb.id, fb.msg.data);
        }
    });

    let goal = ActionSendGoalRequest {
        id: 1,
        goal: msg::String { data: "Start task".to_owned() },
    };
    let response = ActionClient::send_goal(client.clone(), goal, None)
        .await
        .map_err(anyhow::Error::msg)?;
    if !response.accepted {
        return Err(anyhow!("Goal rejected"));
    }

    // Waits until the goal is Succeeded, Aborted or Canceled
    let result = ActionClient::get_result(client.clone(), ActionGetResultRequest { id: 1 }, None)
        .await
        .map_err(anyhow::Error::msg)?;
    println!("{:?}: {:?}", result.status, result.result.map(|r| r.data));

    Ok(())
}
~~~

`send_goal`, `cancel_goal` and `get_result` are service calls. They take the same optional timeout as a [service request](./services.md#client) and return `Err(String)` on failure. The result of a goal is handed out once. The server forgets the goal after delivering it.

### Canceling

`ActionCancelGoalRequest` selects the goals to cancel. Only goals that are `Accepted` or `Executing` are affected.

| `id` | `time` | Cancels |
|---|---|---|
| `Some(id)` | `None` | That goal |
| `None` | `Some(t)` | Every goal accepted at or before `t` (Unix milliseconds) |
| `Some(id)` | `Some(t)` | That goal, and every goal accepted at or before `t` |
| `None` | `None` | All goals |

~~~rust
let response = ActionClient::cancel_goal(
    client.clone(),
    ActionCancelGoalRequest { id: Some(1), time: None },
    None,
)
.await
.map_err(anyhow::Error::msg)?;
println!("Canceling {:?}", response.goals);
~~~

`get_status` returns the state of every goal each time it changes.

Complete clients and servers are in `mt_action/examples/` in the Minot repository.
