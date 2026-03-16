use log::{debug, warn};
use mt_pubsub::{Node, Publisher, Qos, Subscriber};
use mt_sea::Sendable;
use mt_service::{ServiceClient, ServiceServer};
use std::sync::Arc;
use tokio::select;
use tokio::sync::Mutex;
use tokio::task::JoinSet;
use tokio::time::{Duration, Instant, sleep_until};
use tokio_util::sync::CancellationToken;

pub mod messages;
use crate::messages::*;

pub mod runner;
use crate::runner::Action;
use crate::runner::ActionRunner;

/*
 * TODO:
 * - How do we enforce T1-3 match T0 (the Action)?
 *   Currently these are all seperate but very likely
 *   each Action has its own (unique) corresponding types
 *   for its goal request/feedback/result
 */

/**
 * The Action Server
 *
 * The generic parameters mean the following:
 * T0: The action that will be used
 * T1: Data type published as feedback
 * T2: Data type sent in send_goal requests
 * T3: Data type returned in get_result requests
 */
pub struct ActionServer<T0, T1, T2, T3>
where
    T0: Action<T1, T2, T3> + Send + Sync + 'static,
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /// the action state machine
    pub action: Option<Arc<ActionRunner<T0, T1, T2, T3>>>,

    /// publisher for current status
    /// published on state change
    pubber_status: Publisher<ActionStatusMsg>,

    /// publisher for feedback on the current goal
    pubber_feedback: Publisher<ActionFeedbackMsg<T1>>,

    /// service for the new goals
    service_send_goal: Arc<ServiceServer<ActionSendGoalRequest<T2>, ActionSendGoalResponse>>,

    /// service for the canceling goals
    service_cancel_goal: Arc<ServiceServer<ActionCancelGoalRequest, ActionCancelGoalResponse>>,

    /// service for receiving a result
    service_get_result: Arc<ServiceServer<ActionGetResultRequest, ActionGetResultResponse<T3>>>,

    /// service for requesting internal server state
    service_internal_server:
        Arc<ServiceServer<InternalActionServerStateRequest, InternalActionServerStateResponse>>,
}

impl<T0, T1, T2, T3> ActionServer<T0, T1, T2, T3>
where
    T0: Action<T1, T2, T3> + Send + Sync + 'static,
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /**
     * Construct new Action Server
     *
     * @param node    Reference to the managing Node
     * @param topic   Root topic of action
     * @param action  Action state machine (struct implementing the Action trait)
     */
    pub async fn new(
        node: Arc<Node>,
        topic: String,
        action: T0,
        feedback_qos: Qos,
    ) -> anyhow::Result<Arc<Self>> {
        let server = Self {
            action: None,
            pubber_status: node
                .create_publisher(format!("{}/_action/status", &topic), Qos::Reliable)
                .await?,
            pubber_feedback: node
                .create_publisher(format!("{}/_action/feedback", &topic), feedback_qos)
                .await?,
            service_send_goal: ServiceServer::new(
                node.clone(),
                format!("{}/_action/send_goal", &topic),
            )
            .await?,
            service_cancel_goal: ServiceServer::new(
                node.clone(),
                format!("{}/_action/cancel_goal", &topic),
            )
            .await?,
            service_get_result: ServiceServer::new(
                node.clone(),
                format!("{}/_action/get_result", &topic),
            )
            .await?,
            service_internal_server: ServiceServer::new(
                node.clone(),
                format!("{}/_action/internal_server", &topic),
            )
            .await?,
        };

        // Clone the publisher before the Arc wrapping so it's available for the runner.
        let pubber_feedback = server.pubber_feedback.clone();
        let server = Arc::new(server);

        // Arc::get_mut() would fail here because ActionRunner::new() stores a Weak
        // reference to `server`, making the weak-ref count non-zero. The unsafe write
        // is safe: we hold the only strong reference and no task has started yet.
        let action = Some(Arc::new(ActionRunner::new(
            Arc::downgrade(&server),
            action,
            pubber_feedback,
        )));
        let server_ptr = Arc::as_ptr(&server) as *mut Self;
        unsafe {
            (*server_ptr).action = action;
        }

        Ok(server)
    }

    /**
     * Consume and start the server
     *
     * @param shutdown  CancellationToken that stops all internal tasks when cancelled
     */
    pub async fn start(this: Arc<Self>, shutdown: CancellationToken) -> anyhow::Result<()> {
        let runner = this
            .action
            .as_ref()
            .expect("ActionRunner should be initialized by now")
            .clone();

        let mut tasks = JoinSet::new();
        tasks.spawn(Self::handle_send_goal(
            runner.clone(),
            this.service_send_goal.clone(),
        ));
        tasks.spawn(Self::handle_cancel_goal(
            runner.clone(),
            this.service_cancel_goal.clone(),
        ));
        tasks.spawn(Self::handle_get_result(
            runner.clone(),
            this.service_get_result.clone(),
        ));
        tasks.spawn(Self::handle_internal_server(
            runner.clone(),
            this.service_internal_server.clone(),
        ));
        tasks.spawn(ActionRunner::start(runner, shutdown.clone()));

        loop {
            select! {
                _ = shutdown.cancelled() => {
                    warn!("Coordinator unreachable, shutting down action server.");
                    tasks.abort_all();
                    break;
                }
                result = tasks.join_next() => {
                    if result.is_none() {
                        break;
                    }
                }
            }
        }

        Ok(())
    }

    /**
     * Publish a status message
     */
    pub async fn publish_status(this: Arc<Self>, msg: ActionStatusMsg) -> anyhow::Result<()> {
        let pubber = this.pubber_status.clone();
        pubber.publish(&msg).await?;
        Ok(())
    }

    /**
     * Publish a feedback message
     */
    pub async fn publish_feedback(
        this: Arc<Self>,
        msg: ActionFeedbackMsg<T1>,
    ) -> anyhow::Result<()> {
        let pubber = this.pubber_feedback.clone();
        pubber.publish(&msg).await?;
        Ok(())
    }

    async fn handle_send_goal(
        action: Arc<ActionRunner<T0, T1, T2, T3>>,
        service: Arc<ServiceServer<ActionSendGoalRequest<T2>, ActionSendGoalResponse>>,
    ) {
        ServiceServer::start(
            service,
            Arc::new(move |msg| {
                let action = action.clone();
                async move { Ok(action.add_goal(&msg).await) }
            }),
        )
        .await;
    }

    async fn handle_cancel_goal(
        action: Arc<ActionRunner<T0, T1, T2, T3>>,
        service: Arc<ServiceServer<ActionCancelGoalRequest, ActionCancelGoalResponse>>,
    ) {
        ServiceServer::start(
            service,
            Arc::new(move |msg| {
                let action = action.clone();
                async move { Ok(action.cancel_goal(&msg).await) }
            }),
        )
        .await;
    }

    async fn handle_get_result(
        action: Arc<ActionRunner<T0, T1, T2, T3>>,
        service: Arc<ServiceServer<ActionGetResultRequest, ActionGetResultResponse<T3>>>,
    ) {
        ServiceServer::start(
            service,
            Arc::new(move |msg| {
                let action = action.clone();
                async move { Ok(action.get_result(&msg).await) }
            }),
        )
        .await;
    }

    async fn handle_internal_server(
        _action: Arc<ActionRunner<T0, T1, T2, T3>>,
        service: Arc<
            ServiceServer<InternalActionServerStateRequest, InternalActionServerStateResponse>,
        >,
    ) {
        ServiceServer::start(
            service,
            Arc::new(move |_msg| async move { Ok(InternalActionServerStateResponse::Ready) }),
        )
        .await;
    }
}

/**
 * The Action Client
 */
pub struct ActionClient<T1, T2, T3>
where
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /// subscriber for current status
    /// published on state change
    subber_status: Mutex<Subscriber<ActionStatusMsg>>,

    /// subscriber for feedback on the current goal
    subber_feedback: Mutex<Subscriber<ActionFeedbackMsg<T1>>>,

    /// service for the new goals
    service_send_goal: ServiceClient<ActionSendGoalRequest<T2>, ActionSendGoalResponse>,

    /// service for the canceling goals
    service_cancel_goal: ServiceClient<ActionCancelGoalRequest, ActionCancelGoalResponse>,

    /// service for receiving a result
    service_get_result: ServiceClient<ActionGetResultRequest, ActionGetResultResponse<T3>>,

    /// service for interal server requests
    service_internal_server:
        ServiceClient<InternalActionServerStateRequest, InternalActionServerStateResponse>,
}

impl<T1, T2, T3> ActionClient<T1, T2, T3>
where
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /**
     * Create new ActionClient
     *
     * @param node   Reference to the managing Node
     * @param topic  Topic for the action, should match the ActionServer
     */
    pub async fn new(
        node: Arc<Node>,
        topic: String,
        feedback_qos: Qos,
    ) -> anyhow::Result<Arc<Self>> {
        let subber_queue_size = 10;
        Ok(Arc::new(Self {
            subber_status: Mutex::new(
                node.create_subscriber(
                    format!("{}/_action/status", &topic),
                    subber_queue_size,
                    Qos::Reliable,
                )
                .await?,
            ),
            subber_feedback: Mutex::new(
                node.create_subscriber(
                    format!("{}/_action/feedback", &topic),
                    subber_queue_size,
                    feedback_qos,
                )
                .await?,
            ),
            service_send_goal: ServiceClient::new(
                node.clone(),
                format!("{}/_action/send_goal", &topic),
            )
            .await?,
            service_cancel_goal: ServiceClient::new(
                node.clone(),
                format!("{}/_action/cancel_goal", &topic),
            )
            .await?,
            service_get_result: ServiceClient::new(
                node.clone(),
                format!("{}/_action/get_result", &topic),
            )
            .await?,
            service_internal_server: ServiceClient::new(
                node.clone(),
                format!("{}/_action/internal_server", &topic),
            )
            .await?,
        }))
    }

    /**
     * Check if the action server is ready
     * Waits until the server replies with a "ready" message
     *
     * @param this     Reference to ActionClient
     * @param timeout  Optional timeout
     * @returns        true if server was ready within timeout, else true
     */
    pub async fn wait_for_action_server(this: Arc<Self>, timeout: Duration) -> bool {
        let timeout = Instant::now() + timeout;
        loop {
            select! {
                _ = sleep_until(timeout) => {
                    debug!("Timeout reached");
                    return false;
                }
                state = this.service_internal_server.request(InternalActionServerStateRequest{}, None) => {
                    match state {
                        Ok(InternalActionServerStateResponse::Starting) => {
                            debug!("Server is starting");
                            continue;
                        }
                        Ok(InternalActionServerStateResponse::Ready) => {
                            debug!("Server is ready");
                            return true;
                        }
                        Ok(InternalActionServerStateResponse::Broken) => {
                            debug!("Server is broken");
                            return false;
                        }
                        Err(e) => {
                            debug!("Server is... I don't even know: {}", e);
                            return false;
                        }
                    }
                }
            }
        }
    }

    /**
     * Request the current status
     */
    pub async fn get_status(this: Arc<Self>) -> Result<ActionStatusMsg, String> {
        let mut subber = this.subber_status.lock().await;
        match subber.next().await {
            Some(res) => Ok(res),
            None => Err("subber_status returned None".to_owned()),
        }
    }

    /**
     * Request feedback of the current goal
     */
    pub async fn get_feedback(this: Arc<Self>) -> Result<ActionFeedbackMsg<T1>, String> {
        let mut subber = this.subber_feedback.lock().await;
        match subber.next().await {
            Some(res) => Ok(res),
            None => Err("subber_feedback returned None".to_owned()),
        }
    }

    /**
     * Send a new goal
     */
    pub async fn send_goal(
        this: Arc<Self>,
        request: ActionSendGoalRequest<T2>,
        timeout: Option<Duration>,
    ) -> Result<ActionSendGoalResponse, String> {
        return this.service_send_goal.request(request, timeout).await;
    }

    /**
     * Cancel one or more goals
     */
    pub async fn cancel_goal(
        this: Arc<Self>,
        request: ActionCancelGoalRequest,
        timeout: Option<Duration>,
    ) -> Result<ActionCancelGoalResponse, String> {
        return this.service_cancel_goal.request(request, timeout).await;
    }

    /**
     * Get result for a goal
     */
    pub async fn get_result(
        this: Arc<Self>,
        request: ActionGetResultRequest,
        timeout: Option<Duration>,
    ) -> Result<ActionGetResultResponse<T3>, String> {
        return this.service_get_result.request(request, timeout).await;
    }
}
