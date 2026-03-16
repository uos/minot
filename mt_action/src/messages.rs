use rkyv::{Archive, Deserialize, Serialize};

use mt_sea::Sendable;

#[derive(Archive, Serialize, Deserialize, Copy, Clone)]
pub struct InternalActionServerStateRequest {}

#[derive(Archive, Serialize, Deserialize, Copy, Clone)]
pub enum InternalActionServerStateResponse {
    Starting,
    Ready,
    Broken,
}

#[derive(Archive, Serialize, Deserialize, Copy, Clone, Debug, PartialEq)]
pub enum GoalState {
    InvalidGoalID,
    Accepted,
    Executing,
    Canceling,
    Succeeded,
    Aborted,
    Canceled,
}

#[derive(Archive, Serialize, Deserialize, Copy, Clone, Debug)]
pub enum CancelGoalRequestState {
    Ok,
    Rejected,
    InvalidGoalID,
}

#[derive(Archive, Serialize, Deserialize, Copy, Clone, Debug)]
pub struct GoalStatus {
    /// goal ID
    pub id: u64,

    /// UNIX timestamp when goal entered ACCEPTED state, in millis
    pub accepted: u128,

    /// current state of the goal
    pub state: GoalState,
}

#[derive(Archive, Serialize, Deserialize, Clone, Debug)]
pub struct ActionStatusMsg {
    /// in progress (accepted and later) goals
    pub goals: Vec<GoalStatus>,
}

#[derive(Archive, Serialize, Deserialize, Clone)]
pub struct ActionFeedbackMsg<T: Sendable + Clone> {
    /// goal id
    pub id: u64,

    /// user provided msg
    pub msg: T,
}

#[derive(Archive, Serialize, Deserialize, Clone)]
pub struct ActionSendGoalRequest<T: Sendable + Clone> {
    /// goal id
    pub id: u64,

    /// description of the goal
    pub goal: T,
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct ActionSendGoalResponse {
    /// whether the goal was accepted or not
    pub accepted: bool,

    /// UNIX timestamp of the accept time, if accepted, in millis
    pub time: Option<u128>,
}

/**
 * almost like ROS (but with Option instead of != 0):
 * - id == None && time == None -> cancel all goals
 * - id == None && time != None -> cancel all goals accepted before given timestamp (inclusive)
 * - id != None && time == None -> cancel the given goal, if it exists
 * - id != None && time != None -> cancel the given goal and all goals accepted before given
 *   timestamp (inclusive)
 */
#[derive(Archive, Serialize, Deserialize, Clone)]
pub struct ActionCancelGoalRequest {
    /// goal id to cancel
    pub id: Option<u64>,

    /// UNIX timestamp of the accept time, if accepted, in millis
    pub time: Option<u128>,
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct ActionCancelGoalResponse {
    /// status of the goal request
    pub status: CancelGoalRequestState,

    /// list of the cancelled goal IDs
    pub goals: Vec<u64>,
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct ActionGetResultRequest {
    /// goal id to get result for
    pub id: u64,
}

#[derive(Archive, Serialize, Deserialize, Debug, Clone)]
pub struct ActionGetResultResponse<T: Sendable + Clone> {
    /// goal id to get result for
    pub status: GoalState,

    /// user defined result, if status == Succeeded
    pub result: Option<T>,
}
