use std::future::Future;

use crate::ActionServer;
use log::{debug, error};
use mt_pubsub::Publisher;
use mt_sea::Sendable;
use std::sync::{Arc, Weak};
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::select;
use tokio::sync::broadcast;
use tokio::sync::mpsc;
use tokio::sync::mpsc::error::TrySendError;
use tokio::sync::{Mutex, MutexGuard};
use tokio_util::sync::CancellationToken;

use crate::messages::*;

/// Allows an action implementation to push feedback to the client
/// while `perform_goal` is running.
pub struct FeedbackSender<T1>
where
    T1: Sendable + Clone,
{
    publisher: Publisher<ActionFeedbackMsg<T1>>,
    pub goal_id: u64,
}

impl<T1: Sendable + Clone> FeedbackSender<T1> {
    pub async fn send(&self, msg: T1) -> anyhow::Result<()> {
        self.publisher
            .publish(&ActionFeedbackMsg {
                id: self.goal_id,
                msg,
            })
            .await
    }
}

/**
 * Trait for an Action state machine
 * that should be run on an ActionServer/ActionRunner.
 *
 * The ActionServer essentially acts as a forwarder
 * while communicating with the underlying Action over channels.
 *
 * It's up to the Action implementation to handle these messages properly.
 */
pub trait Action<T1, T2, T3>
where
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /**
     * Perform the provided goal
     * The parameter goal should be updated in-place with all state changes
     * that occurred.
     *
     * The goal should always end up in a terminal state
     * i.e. Succeeded, Aborted or Canceled
     *
     * @param goal      The goal to perform
     * @param stoken    CancellationToken indicating a goal should be canceled
     * @param feedback  Sender for pushing progress feedback to the client
     */
    fn perform_goal(
        &self,
        goal: &mut Goal<T2, T3>,
        stoken: CancellationToken,
        feedback: FeedbackSender<T1>,
    ) -> impl Future<Output = anyhow::Result<()>> + Send;
}

#[derive(Clone)]
pub struct Goal<T0, T1>
where
    T0: Sendable + Clone,
    T1: Sendable + Clone,
{
    /// ID of the goal
    pub id: u64,

    /// time when goal was accepted, in millis
    pub accepted: u128,

    /// goal state
    pub state: GoalState,

    /// goal data
    pub data: T0,

    /// result data
    pub result: Option<T1>,
}

pub struct ActionRunner<T0, T1, T2, T3>
where
    T0: Action<T1, T2, T3> + Send + Sync + 'static,
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /// reference to the managing ActionServer
    server: Weak<ActionServer<T0, T1, T2, T3>>,

    /// the action to run
    action: T0,

    /// publisher for feedback messages
    pubber_feedback: Publisher<ActionFeedbackMsg<T1>>,

    /// channel to trigger wakeups in the runner
    wakeup_runner_channel: (mpsc::Sender<u8>, Mutex<mpsc::Receiver<u8>>),

    /// channel to trigger wakeups in the reconcile thread
    wakeup_reconcile_channel: (mpsc::Sender<u8>, Mutex<mpsc::Receiver<u8>>),

    /// channel to notify goal updates
    goal_update_channel: (broadcast::Sender<u8>, broadcast::Receiver<u8>),

    /// mapping of ID -> Goal
    goals: Mutex<Vec<Goal<T2, T3>>>,
}

impl<T0, T1, T2, T3> ActionRunner<T0, T1, T2, T3>
where
    T0: Action<T1, T2, T3> + Send + Sync + 'static,
    T1: Sendable + Clone,
    T2: Sendable + Clone,
    T3: Sendable + Clone,
{
    /**
     * Create new ActionRunner
     */
    pub fn new(
        server: Weak<ActionServer<T0, T1, T2, T3>>,
        action: T0,
        pubber_feedback: Publisher<ActionFeedbackMsg<T1>>,
    ) -> Self {
        let (wakeup_runner_tx, wakeup_runner_rx) = mpsc::channel(1);
        let wakeup_runner_rx = Mutex::new(wakeup_runner_rx);
        let (wakeup_reconcile_tx, wakeup_reconcile_rx) = mpsc::channel(1);
        let wakeup_reconcile_rx = Mutex::new(wakeup_reconcile_rx);
        Self {
            server,
            action,
            pubber_feedback,
            wakeup_runner_channel: (wakeup_runner_tx, wakeup_runner_rx),
            wakeup_reconcile_channel: (wakeup_reconcile_tx, wakeup_reconcile_rx),
            goal_update_channel: broadcast::channel(10),
            goals: Mutex::new(Vec::new()),
        }
    }

    /**
     * Start the ActionRunner main loop
     */
    pub async fn start(this: Arc<Self>, shutdown: CancellationToken) {
        // currently executing goal as (ID, CancellationToken) tuple
        let current_goal: Arc<Mutex<Option<(u64, CancellationToken)>>> = Arc::new(Mutex::new(None));

        let this_reconcile = this.clone();
        let current_goal_reconcile = current_goal.clone();
        let shutdown_reconcile = shutdown.clone();
        let _reconcile_handle = tokio::spawn(async move {
            ActionRunner::start_reconcile_goals(
                this_reconcile,
                current_goal_reconcile,
                shutdown_reconcile,
            )
            .await;
        });

        let mut wakeup_runner = this.wakeup_runner_channel.1.lock().await;

        loop {
            let next_goal = this.next_goal().await;

            // wait until a goal is available
            if next_goal.is_none() {
                debug!("No goal - waiting for wakeup_runner_channel notification");
                select! {
                    _ = shutdown.cancelled() => {
                        debug!("ActionRunner shutting down");
                        break;
                    }
                    result = wakeup_runner.recv() => {
                        if result.is_none() {
                            error!("wakeup_runner channel closed unexpectedly");
                        }
                    }
                }

                // restart loop to grab new next_goal
                continue;
            }

            let mut goal = next_goal.unwrap();

            let stoken = CancellationToken::new();
            current_goal.lock().await.replace((goal.id, stoken.clone()));

            debug!("Executing new goal: {:?}", goal.id);

            goal.state = GoalState::Executing;

            if this.update_goal(goal.clone()).await.is_err() {
                error!("Failed to update goal");
            }

            let feedback = FeedbackSender {
                publisher: this.pubber_feedback.clone(),
                goal_id: goal.id,
            };

            if this
                .action
                .perform_goal(&mut goal, stoken.clone(), feedback)
                .await
                .is_err()
            {
                error!("Error while performing goal");
            }

            if this.update_goal(goal).await.is_err() {
                error!("Failed to update goal");
            }

            // clear current_goal
            current_goal.lock().await.take();
        }
    }

    /**
     * Reconcile current goals thread (mostly for canceling)
     *
     * Goals not currently executing enter canceled state
     * immediately, if a goal is currently executing a cancellation
     * request is sent to it.
     *
     */
    async fn start_reconcile_goals(
        this: Arc<Self>,
        current_goal: Arc<Mutex<Option<(u64, CancellationToken)>>>,
        shutdown: CancellationToken,
    ) {
        let mut wakeup_reconcile = this.wakeup_reconcile_channel.1.lock().await;

        loop {
            select! {
                _ = shutdown.cancelled() => {
                    debug!("Reconcile task shutting down");
                    break;
                }
                result = wakeup_reconcile.recv() => {
                    if result.is_none() {
                        break;
                    }
                }
            }

            debug!("Reconcile thread notified");

            let mut goals = this.goals.lock().await;
            for goal in goals.clone() {
                if goal.state != GoalState::Canceling {
                    continue;
                }

                let executing = current_goal.lock().await;
                if let Some((id, stoken)) = executing.clone() {
                    if goal.id == id {
                        stoken.cancel();

                        // in this case we expect
                        // Action::perform_goal() to handle the
                        // cancellation properly
                        continue;
                    }
                }

                if this
                    .update_goal_reentrant(
                        Goal {
                            id: goal.id,
                            accepted: goal.accepted,
                            state: GoalState::Canceled,
                            data: goal.data.clone(),
                            result: goal.result.clone(),
                        },
                        &mut goals,
                    )
                    .await
                    .is_err()
                {
                    error!("Failed to update goal");
                }
            }
        }
    }

    /**
     * Get the next goal in Accepted state, or None
     */
    async fn next_goal(&self) -> Option<Goal<T2, T3>> {
        let goals = self.goals.lock().await;

        for goal in goals.iter() {
            if goal.state == GoalState::Accepted {
                return Some(goal.clone());
            }
        }

        None
    }

    /**
     * Lock self.goals and update a goal
     */
    async fn update_goal(&self, new_goal: Goal<T2, T3>) -> Result<(), ()> {
        let mut goals = self.goals.lock().await;
        self.update_goal_reentrant(new_goal, &mut goals).await
    }

    /**
     * Update a goal, matched by their ID field
     * Returns error if no matching Goal is in self.goals
     *
     * Reentrant version to be used if the calling function
     * already has a lock on self.gloals
     */
    async fn update_goal_reentrant(
        &self,
        new_goal: Goal<T2, T3>,
        goals: &mut MutexGuard<'_, Vec<Goal<T2, T3>>>,
    ) -> Result<(), ()> {
        let mut updated = false;
        let mut state_changed = false;

        for old_goal in goals.iter_mut() {
            if old_goal.id == new_goal.id {
                if old_goal.state != new_goal.state {
                    state_changed = true;
                }

                *old_goal = new_goal;
                updated = true;
                break;
            }
        }

        if updated && state_changed {
            debug!("Goal state changed updated - notifying");
            if let Some(server) = self.server.upgrade() {
                let mut new_goals = Vec::new();
                for old_goal in goals.iter() {
                    new_goals.push(GoalStatus {
                        id: old_goal.id,
                        accepted: old_goal.accepted,
                        state: old_goal.state,
                    });
                }

                let msg = ActionStatusMsg { goals: new_goals };

                if server.pubber_status.publish(&msg).await.is_err() {
                    error!("Error publishing status update")
                }
            }
        }

        if updated {
            debug!("Goal updated - notifying");
            if self.goal_update_channel.0.send(1).is_err() {
                error!("Error notifying goal update");
            }

            return Ok(());
        }

        error!("No matching goal for goal update");
        Err(())
    }

    /**
     * Add a goal
     */
    pub async fn add_goal(&self, request: &ActionSendGoalRequest<T2>) -> ActionSendGoalResponse {
        debug!("Got new goal with id {}", request.id);

        let mut goals = self.goals.lock().await;
        let accepted_at = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("Getting UNIX time failed")
            .as_millis();

        // prevent duplicates
        for goal in goals.iter() {
            if request.id == goal.id {
                return ActionSendGoalResponse {
                    accepted: false,
                    time: None,
                };
            }
        }

        goals.push(Goal {
            id: request.id,
            accepted: accepted_at,
            state: GoalState::Accepted,
            data: request.goal.clone(),
            result: None,
        });

        match self.wakeup_runner_channel.0.try_send(1) {
            Ok(_) | Err(TrySendError::Full(_)) => {
                debug!("Notified wakeup_runner_channel");
            }
            Err(TrySendError::Closed(_)) => {
                error!("wakeup_runner_channel closed unexpectedly");
            }
        }

        ActionSendGoalResponse {
            accepted: true,
            time: Some(accepted_at),
        }
    }

    /**
     * Cancel on or more goals
     */
    pub async fn cancel_goal(&self, request: &ActionCancelGoalRequest) -> ActionCancelGoalResponse {
        let mut goals = self.goals.lock().await;
        let mut cancelled_goal_ids: Vec<u64> = Vec::new();

        for goal in goals.iter_mut() {
            // cancellation is only possible from these states
            // the others are either terminal or already "canceling"
            match goal.state {
                GoalState::Accepted | GoalState::Executing => {}
                _ => continue,
            }

            let mut should_cancel: bool = false;
            match request.id {
                Some(id) => match request.time {
                    Some(time) => {
                        if goal.id == id {
                            should_cancel = true;
                        }
                        if goal.accepted <= time {
                            should_cancel = true;
                        }
                    }
                    None => {
                        if goal.id == id {
                            should_cancel = true;
                        }
                    }
                },
                None => match request.time {
                    Some(time) => {
                        if goal.accepted <= time {
                            should_cancel = true;
                        }
                    }
                    None => {
                        should_cancel = true;
                    }
                },
            }

            if should_cancel {
                cancelled_goal_ids.push(goal.id);
                goal.state = GoalState::Canceling;
            }
        }

        match self.wakeup_reconcile_channel.0.try_send(1) {
            Ok(_) | Err(TrySendError::Full(_)) => {
                debug!("Notified wakeup_reconcile_channel");
            }
            Err(TrySendError::Closed(_)) => {
                error!("wakeup_reconcile_channel closed unexpectedly");
            }
        }

        ActionCancelGoalResponse {
            status: CancelGoalRequestState::Ok,
            goals: cancelled_goal_ids,
        }
    }

    /**
     * Get result for a goal
     */
    pub async fn get_result(
        &self,
        request: &ActionGetResultRequest,
    ) -> ActionGetResultResponse<T3> {
        // subscribe once before the loop to avoid missing notifications
        // between the goals-lock check and the next recv()
        let mut update_rx = self.goal_update_channel.0.subscribe();

        // assume terminal state by default
        let mut is_terminal = true;

        loop {
            // during last check goal was in non-terminal state
            if !is_terminal {
                debug!("Waiting for goal update");
                if update_rx.recv().await.is_err() {
                    error!("Goal channel update closed unexpectedly");
                }
                debug!("Received goal update notification");
                is_terminal = true;
            }

            let mut goals = self.goals.lock().await;

            let mut found_idx = None;
            for (idx, goal) in goals.iter().enumerate() {
                if request.id != goal.id {
                    continue;
                }

                // We only want to return a result in a terminal state.
                match goal.state {
                    GoalState::Succeeded | GoalState::Canceled | GoalState::Aborted => {}
                    _ => {
                        debug!(
                            "Skipping get_result for goal ID {:?} in non-terminal state {:?}",
                            goal.id, goal.state
                        );
                        is_terminal = false;
                        break;
                    }
                }

                found_idx = Some((idx, goal.state, goal.result.clone()));
                break;
            }

            if let Some((idx, state, result)) = found_idx {
                // remove goal after delivering terminal response (ROS2 behavior)
                goals.remove(idx);
                return ActionGetResultResponse {
                    status: state,
                    result,
                };
            }

            // wait for update and check again
            if !is_terminal {
                continue;
            }

            // no matching goal
            return ActionGetResultResponse {
                status: GoalState::InvalidGoalID,
                result: None,
            };
        }
    }
}
