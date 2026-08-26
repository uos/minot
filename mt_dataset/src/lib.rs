//! Shared dataset management for Minot and Marina.
//!
//! This crate is the dataset *kernel*: configuration, the local cache, registry
//! drivers, and the client half of the `minot://` transport — everything needed
//! to resolve, pull, and stream a dataset. Both applications depend on it, and
//! it depends on neither.
//!
//! Marina's application layer lives in the `marina` crate, which re-exports this
//! API as its stable facade and adds the command-line interface, the C ABI, and
//! the `marina serve` side of the `minot://` protocol.
//!
//! ## Add the dependency
//!
//! ```toml
//! [dependencies]
//! mt_dataset = "0.10"
//! tokio = { version = "1", features = ["rt", "macros"] }
//! ```
//!
//! Marina's async methods require a **tokio** runtime. A single-threaded runtime
//! (`#[tokio::main(flavor = "current_thread")]`) is sufficient.
//!
//! ## Resolve a dataset
//!
//! [`Marina::resolve_target`] checks whether a dataset is already cached, available locally,
//! or only reachable via a remote registry.
//!
//! ```no_run
//! use mt_dataset::{Marina, ResolveResult};
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let marina = Marina::load()?;
//!
//!     match marina.resolve_target("outdoor-run:v2", None).await? {
//!         ResolveResult::LocalPath(p) | ResolveResult::Cached(p) => {
//!             println!("ready at {}", p.display());
//!         }
//!         ResolveResult::RemoteAvailable { bag, registry, .. } => {
//!             println!("remote: {bag} in {registry}");
//!         }
//!         ResolveResult::Ambiguous { candidates } => {
//!             println!("found in {} registries", candidates.len());
//!         }
//!     }
//!     Ok(())
//! }
//! ```
//!
//! Pass a registry name as the second argument to restrict the search:
//!
//! ```no_run
//! # use mt_dataset::Marina;
//! # #[tokio::main] async fn main() -> anyhow::Result<()> {
//! # let marina = Marina::load()?;
//! marina.resolve_target("outdoor-run:v2", Some("team-ssh")).await?;
//! # Ok(()) }
//! ```
//!
//! ## Pull a dataset
//!
//! [`Marina::pull_exact`] downloads a dataset if it is not already cached and returns the local path:
//!
//! ```no_run
//! use mt_dataset::{Marina, BagRef};
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let mut marina = Marina::load()?;
//!     let bag: BagRef = "outdoor-run:v2".parse()?;
//!     let path = marina.pull_exact(&bag, None).await?;
//!     println!("cached at {}", path.display());
//!     Ok(())
//! }
//! ```
//!
//! ## Pull with progress reporting
//!
//! Implement [`ProgressSink`] to receive phase events during download and decompression:
//!
//! ```no_run
//! use mt_dataset::{Marina, BagRef, ProgressEvent, ProgressReporter, ProgressSink};
//!
//! struct MyProgress;
//!
//! impl ProgressSink for MyProgress {
//!     fn emit(&mut self, event: ProgressEvent) {
//!         println!("[{}] {}", event.phase, event.message);
//!     }
//! }
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let mut marina = Marina::load()?;
//!     let bag: BagRef = "outdoor-run:v2".parse()?;
//!
//!     let mut sink = MyProgress;
//!     let mut reporter = ProgressReporter::new(&mut sink);
//!     let path = marina.pull_exact_with_progress(&bag, None, &mut reporter).await?;
//!     println!("done: {}", path.display());
//!     Ok(())
//! }
//! ```
//!
//! [`WriterProgress`] is a built-in sink that writes human-readable output to any [`std::io::Write`]:
//!
//! ```no_run
//! use mt_dataset::{Marina, BagRef, ProgressReporter, WriterProgress};
//!
//! #[tokio::main]
//! async fn main() -> anyhow::Result<()> {
//!     let mut marina = Marina::load()?;
//!     let bag: BagRef = "outdoor-run:v2".parse()?;
//!
//!     let mut stdout = std::io::stdout();
//!     let mut sink = WriterProgress::new(&mut stdout);
//!     let mut reporter = ProgressReporter::new(&mut sink);
//!     marina.pull_exact_with_progress(&bag, None, &mut reporter).await?;
//!     Ok(())
//! }
//! ```

pub mod cleanup;
pub mod core;
pub mod io;
pub mod model;
pub mod progress;
pub mod registry;
pub mod storage;

pub use core::{
    AccessMode, CacheMirrorOptions, CacheMirrorStats, CachedBagInfo, CachedSizeStats,
    DatasetAccess, Marina, PullOptions, PushOptions, RemoteBagHit, RemovedRegistry, ResolveResult,
};
/// Parsed bag reference like `namespace/name:tag1:tag2[attachment.txt]`.
pub use model::bag_ref::BagRef;
/// Progress primitives to receive phase-based feedback from push/pull operations.
pub use progress::{ProgressEvent, ProgressReporter, ProgressSink, WriterProgress};
