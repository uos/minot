# Changelog - 0.7.0

## Architectural Changes & Highlights
- **Automatic Coordinator (`mt_coord`)**: Minot networking nodes (Ships) now feature an automatic embedded coordinator fallback. If a Ship fails to register with an existing coordinator on the network within a timeout, it will automatically spawn its own embedded coordinator and retry. This drastically simplifies the developer experience for small scripts and examples, as a separate `minot-coord` process is no longer strictly required.
- **P2P Peer Monitoring & Torpedo System**: The architecture is now more resilient and peer-to-peer. While the Coordinator still calculates routes and rules, the data flow and peer health monitoring are fully P2P.
  - Ships actively monitor their connected peers directly via Zenoh `Heartbeat` queries with `DataHigh` priority.
  - If a peer dies unexpectedly, the monitoring Ship reports it via a `PeerDead` message.
  - The `Torpedo` mechanism cleans up zombie nodes or reacts to explicit network teardowns, instructing nodes to shut down safely.
- **Quality of Service (QoS) - BestEffort vs Reliable**: Introduced explicit `Qos` configurations. You can now spawn BestEffort publishers that fire-and-forget over Zenoh using `Background` priority and `Drop` congestion control, ensuring they don't overwhelm critical network traffic.
- **Asynchronous & Less Noisy Logging**: Logging has been cleaned up and made less noisy across the stack. Heavy asynchronous operations have been optimized.

## CLI & Command Changes
- **`minot tui` renamed to `minot sync`**: The primary command to start the Terminal UI is now `sync`. This better reflects its role in synchronizing states and routes across the network.
- **`wind-rat` renamed to `wind-mt-pubsub`**: The standalone native publisher binary has been renamed to match the new crate naming convention.
- **New Command: `minot async`**: Introduced a dedicated real-time bag playback command. It plays bag files (.mcap, .db3) in real-time to all connected "Winds" (ROS2, mt_pubsub, etc.) without needing a full TUI or a script file. Supports playback rate control via `--rate`.

## Crate Refactoring & New Features
- **`ratpub` Renamed to `mt_pubsub`**: The native publish/subscribe library has been renamed to `mt_pubsub` to better reflect its place in the Minot ecosystem.
- **New Crate: `mt_service`**: Implements ROS2-compatible Request/Reply service primitives natively in Minot.
- **New Crate: `mt_action`**: Implements ROS2-compatible Action server/client lifecycles.
  - Full support for the standard ROS2 Action state machine (Accepted, Executing, Canceling, Succeeded, Aborted, Canceled).
- **New Crate: `mt_scope`**: Extracted scope logic into its own crate for improved modularity.
- **Upgraded `mcap` Dependency**: Updated from 0.17.0 to 0.24.0, maintaining Rust 1.85 compatibility.

## What is P2P and what is not?
- **P2P**: 
  - **Data Transfer**: All `mt_pubsub`, `mt_service`, and `mt_action` data packets flow directly between nodes via Zenoh.
  - **Health Monitoring**: Nodes constantly ping each other using a Zenoh queryable to ensure the data path is alive.
- **Centralized**: 
  - **Rule Distribution**: The `mt_coord` coordinator calculates dynamic routes based on rules and pushes them to nodes.
  - **Initial Discovery**: Nodes discover each other by first registering with the Coordinator.
