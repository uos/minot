//! A file on another machine that reads like a local one.
//!
//! [`RemoteFile`] implements `Read + Seek + Send + Sync`, which is all
//! `mt_bagread` needs to open a bag: MCAP parsing there is sans-io, asking for
//! byte ranges and being handed the bytes. Put one of these behind it and a bag
//! on a robot reads like a bag on disk.
//!
//! # Why blocks, and why they are not optional
//!
//! Reads are served in fixed-size blocks, never as the exact range asked for.
//! Phase 0 measured why: opening an MCAP reads its summary in **2130 separate
//! reads averaging 59 bytes**, and those reads are inherently serial — each one
//! depends on the parse state the last produced, so no amount of readahead
//! helps. At 200 ms round trip that is over four hundred seconds to open one
//! bag.
//!
//! Fetching whole blocks collapses that: 2130 tiny reads fall inside a handful
//! of blocks, and every read after the first in a block is a memory copy. The
//! block size is therefore a correctness property, not a tuning knob, and
//! `an_open_costs_only_a_handful_of_requests` guards it.
//!
//! # Why a thread
//!
//! The reader above is synchronous and may itself be running inside a Tokio
//! runtime. Blocking a runtime worker on a future that needs that same runtime
//! deadlocks, so fetches are handed to a dedicated OS thread — which is not a
//! runtime worker, and may therefore block on a runtime handle safely — and
//! answered over a plain channel. Ugly, but it is the honest way to put an
//! async transport under a synchronous `Read`.

use std::io::{self, Read, Seek, SeekFrom};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Condvar, Mutex};

use anyhow::Result;

use super::block_store::{BlockStore, CacheMode, NullBlockStore};

/// Bytes per block.
///
/// Phase 0 measured MCAP chunk reads at a p95 of ~700 KiB. One MiB normally
/// contains a whole chunk, while keeping an underrun short enough that a slow
/// link does not turn it into a long visible playback freeze.
pub const DEFAULT_BLOCK_BYTES: usize = 1024 * 1024;

/// How many blocks ahead to fetch in the background on a forward read.
///
/// Chunk offsets were measured to be strictly monotonic, so "the next few
/// blocks" is a good guess and needs no cleverness. Sixteen one-MiB blocks keep
/// the same 16 MiB byte window as four four-MiB blocks, but make each possible
/// underrun four times shorter.
pub const DEFAULT_READAHEAD_BLOCKS: usize = 16;

/// Fetches byte ranges for a [`RemoteFile`].
///
/// A trait so the cache and read logic can be tested against a local file with
/// no network at all, which is what makes the equivalence test cheap enough to
/// run every time.
pub trait RangeFetcher: Send + Sync {
    /// Read up to `len` bytes at `offset`. A short read means end of file.
    fn fetch(&self, path: &str, offset: u64, len: u32) -> Result<Vec<u8>>;
}

/// Counts requests, so a test can assert that block caching is doing its job.
pub struct CountingFetcher<F: RangeFetcher> {
    inner: F,
    requests: AtomicU64,
    bytes: AtomicU64,
}

impl<F: RangeFetcher> CountingFetcher<F> {
    pub fn new(inner: F) -> Self {
        Self {
            inner,
            requests: AtomicU64::new(0),
            bytes: AtomicU64::new(0),
        }
    }

    pub fn requests(&self) -> u64 {
        self.requests.load(Ordering::Relaxed)
    }

    pub fn bytes(&self) -> u64 {
        self.bytes.load(Ordering::Relaxed)
    }
}

impl<F: RangeFetcher> RangeFetcher for CountingFetcher<F> {
    fn fetch(&self, path: &str, offset: u64, len: u32) -> Result<Vec<u8>> {
        self.requests.fetch_add(1, Ordering::Relaxed);
        let data = self.inner.fetch(path, offset, len)?;
        self.bytes.fetch_add(data.len() as u64, Ordering::Relaxed);
        Ok(data)
    }
}

/// Reads ranges straight out of a local file. For tests, and for a server and
/// client that happen to be the same machine.
pub struct LocalFetcher {
    pub root: std::path::PathBuf,
}

impl RangeFetcher for LocalFetcher {
    fn fetch(&self, path: &str, offset: u64, len: u32) -> Result<Vec<u8>> {
        use std::io::{Read, Seek, SeekFrom};
        let mut file = std::fs::File::open(self.root.join(path))?;
        let size = file.metadata()?.len();
        if offset >= size {
            return Ok(Vec::new());
        }
        let readable = (size - offset).min(len as u64) as usize;
        let mut buffer = vec![0u8; readable];
        file.seek(SeekFrom::Start(offset))?;
        file.read_exact(&mut buffer)?;
        Ok(buffer)
    }
}

/// How many blocks stay in memory in front of the store.
///
/// This is what makes the tiny-read case cheap: an MCAP summary read touches
/// the same block thousands of times, and going to the store — even a local
/// file — for each of those would be thousands of syscalls. Small, because its
/// job is to catch repeats, not to be the cache.
pub const DEFAULT_HOT_BLOCKS: usize = 8;

enum PrefetchedBlock {
    Pending,
    Ready(Result<Vec<u8>, String>),
}

struct PrefetchState {
    blocks: std::collections::HashMap<u64, PrefetchedBlock>,
}

/// A bounded, best-effort worker that keeps network reads off the playback
/// thread. `RemoteFile` owns the block store. The
/// worker only fetches bytes, and the reader commits them when it needs them.
struct Prefetch {
    requests: std::sync::mpsc::SyncSender<u64>,
    state: Arc<(Mutex<PrefetchState>, Condvar)>,
    capacity: usize,
}

impl Prefetch {
    fn spawn(
        fetcher: Arc<dyn RangeFetcher>,
        path: String,
        block_bytes: usize,
        capacity: usize,
    ) -> Self {
        let (requests, incoming) = std::sync::mpsc::sync_channel(capacity);
        let state = Arc::new((
            Mutex::new(PrefetchState {
                blocks: std::collections::HashMap::new(),
            }),
            Condvar::new(),
        ));
        let worker_state = Arc::clone(&state);
        std::thread::Builder::new()
            .name("marina-readahead".to_string())
            .spawn(move || {
                while let Ok(index) = incoming.recv() {
                    let offset = index * block_bytes as u64;
                    let result = fetcher
                        .fetch(&path, offset, block_bytes as u32)
                        .map_err(|error| error.to_string());
                    let (lock, ready) = &*worker_state;
                    let mut state = lock.lock().unwrap_or_else(|error| error.into_inner());
                    state.blocks.insert(index, PrefetchedBlock::Ready(result));
                    ready.notify_all();
                }
            })
            .expect("the readahead thread should start");
        Self {
            requests,
            state,
            capacity,
        }
    }

    fn schedule(&self, index: u64) {
        let (lock, _) = &*self.state;
        let mut state = lock.lock().unwrap_or_else(|error| error.into_inner());
        if state.blocks.contains_key(&index) || state.blocks.len() >= self.capacity {
            return;
        }
        state.blocks.insert(index, PrefetchedBlock::Pending);
        if self.requests.try_send(index).is_err() {
            state.blocks.remove(&index);
        }
    }

    /// Wait for a block that is already on its way. Waiting here is still
    /// useful: normally the request started while the previous block was being
    /// decoded, including reads issued between block boundaries.
    fn take(&self, index: u64) -> Option<Result<Vec<u8>, String>> {
        let (lock, ready) = &*self.state;
        let mut state = lock.lock().unwrap_or_else(|error| error.into_inner());
        loop {
            match state.blocks.remove(&index) {
                Some(PrefetchedBlock::Ready(result)) => return Some(result),
                Some(PrefetchedBlock::Pending) => {
                    state.blocks.insert(index, PrefetchedBlock::Pending);
                    state = ready.wait(state).unwrap_or_else(|error| error.into_inner());
                }
                None => return None,
            }
        }
    }

    fn discard_ready(&self) {
        let (lock, _) = &*self.state;
        let mut state = lock.lock().unwrap_or_else(|error| error.into_inner());
        state
            .blocks
            .retain(|_, block| matches!(block, PrefetchedBlock::Pending));
    }
}

/// A bounded most-recently-used window of blocks held in memory.
///
/// Bounded is the point: in ephemeral mode this is the *only* thing holding
/// blocks, so its limit is what keeps memory flat while reading a file of any
/// size.
struct HotBlocks {
    limit: usize,
    blocks: std::collections::HashMap<u64, Arc<Vec<u8>>>,
    /// Least recently used first.
    order: std::collections::VecDeque<u64>,
}

impl HotBlocks {
    fn new(limit: usize) -> Self {
        Self {
            limit: limit.max(1),
            blocks: std::collections::HashMap::new(),
            order: std::collections::VecDeque::new(),
        }
    }

    fn get(&mut self, index: u64) -> Option<Arc<Vec<u8>>> {
        let block = self.blocks.get(&index)?;
        let block = Arc::clone(block);
        self.touch(index);
        Some(block)
    }

    fn touch(&mut self, index: u64) {
        if let Some(position) = self.order.iter().position(|held| *held == index) {
            self.order.remove(position);
        }
        self.order.push_back(index);
    }

    fn insert(&mut self, index: u64, block: Arc<Vec<u8>>) {
        self.blocks.insert(index, block);
        self.touch(index);
        while self.order.len() > self.limit {
            if let Some(evicted) = self.order.pop_front() {
                self.blocks.remove(&evicted);
            }
        }
    }

    fn len(&self) -> usize {
        self.blocks.len()
    }
}

impl std::fmt::Debug for RemoteFile {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RemoteFile")
            .field("path", &self.path)
            .field("size", &self.size)
            .field("position", &self.position)
            .field("hot_blocks", &self.hot.len())
            .field("persists", &self.persists())
            .finish_non_exhaustive()
    }
}

pub struct RemoteFile {
    fetcher: Arc<dyn RangeFetcher>,
    path: String,
    size: u64,
    position: u64,
    block_bytes: usize,
    readahead: usize,
    last_readahead: Option<u64>,
    prefetch: Option<Prefetch>,
    hot: HotBlocks,
    /// Built on first use, not at construction.
    ///
    /// A store is tied to a block size — its map records which *blocks* it
    /// holds, so the same index means a different byte range at a different
    /// size. Building eagerly meant that setting the block size afterwards
    /// discarded a cache that had just been opened, and a second run would
    /// throw away everything the first had fetched. Deferring until the first
    /// read lets the builder be called in any order.
    store: Option<Box<dyn BlockStore>>,
    mode: CacheMode,
}

impl RemoteFile {
    /// A file that keeps nothing: blocks are dropped as the read moves past.
    pub fn new(fetcher: Arc<dyn RangeFetcher>, path: impl Into<String>, size: u64) -> Self {
        Self {
            fetcher,
            path: path.into(),
            size,
            position: 0,
            block_bytes: DEFAULT_BLOCK_BYTES,
            readahead: DEFAULT_READAHEAD_BLOCKS,
            last_readahead: None,
            prefetch: None,
            hot: HotBlocks::new(DEFAULT_HOT_BLOCKS),
            store: None,
            mode: CacheMode::Ephemeral,
        }
    }

    /// A file whose blocks are kept according to `mode`.
    ///
    /// With [`CacheMode::Disk`] a partial read survives a restart and a complete
    /// read leaves a byte-exact local copy. With [`CacheMode::Ephemeral`] this
    /// is the same as [`RemoteFile::new`].
    pub fn with_cache(
        fetcher: Arc<dyn RangeFetcher>,
        path: impl Into<String>,
        size: u64,
        mode: &CacheMode,
    ) -> Self {
        Self {
            fetcher,
            path: path.into(),
            size,
            position: 0,
            block_bytes: DEFAULT_BLOCK_BYTES,
            readahead: DEFAULT_READAHEAD_BLOCKS,
            last_readahead: None,
            prefetch: None,
            hot: HotBlocks::new(DEFAULT_HOT_BLOCKS),
            store: None,
            mode: mode.clone(),
        }
    }

    /// Whether this file's blocks are being kept on disk.
    pub fn persists(&self) -> bool {
        matches!(self.mode, CacheMode::Disk { .. })
    }

    /// Whether every block has been fetched, so the local copy is complete.
    ///
    /// False before the first read, when there is nothing to have completed.
    pub fn is_complete(&self) -> bool {
        self.store
            .as_ref()
            .map(|store| store.is_complete())
            .unwrap_or(false)
    }

    /// Blocks currently held in memory. See [`HotBlocks`] for the bound.
    pub fn hot_blocks(&self) -> usize {
        self.hot.len()
    }

    /// Set the block size.
    ///
    /// Rebuilds the store, because a store's map records which *blocks* it
    /// holds and the same index means a different byte range at a different
    /// block size. Without this, calling it after [`RemoteFile::with_cache`]
    /// would leave the two disagreeing and quietly corrupt the cache.
    /// Set the block size. Must be called before the first read.
    pub fn with_block_bytes(mut self, block_bytes: usize) -> Self {
        assert!(block_bytes > 0, "block size must be greater than zero");
        debug_assert!(
            self.store.is_none(),
            "the block size must be set before the first read"
        );
        self.block_bytes = block_bytes;
        self
    }

    /// How many blocks stay in memory. In ephemeral mode this is the read's
    /// entire memory footprint.
    pub fn with_hot_blocks(mut self, blocks: usize) -> Self {
        self.hot = HotBlocks::new(blocks);
        self
    }

    pub fn with_readahead(mut self, blocks: usize) -> Self {
        self.readahead = blocks;
        self
    }

    pub fn size(&self) -> u64 {
        self.size
    }

    /// Open the block store, now that the block size is settled.
    ///
    /// Cache startup failures leave reads on the network path.
    fn ensure_store(&mut self) -> &dyn BlockStore {
        if self.store.is_none() {
            let store = self
                .mode
                .build(&self.path, self.size, self.block_bytes)
                .unwrap_or_else(|error| {
                    log::warn!(
                        "could not open the block cache for '{}', reading without one: {error}",
                        self.path
                    );
                    Box::new(NullBlockStore)
                });
            self.store = Some(store);
        }
        self.store.as_deref().expect("just built")
    }

    /// A block, from memory, then the store, then the network.
    fn block(&mut self, index: u64) -> io::Result<Arc<Vec<u8>>> {
        if let Some(block) = self.hot.get(index) {
            self.schedule_readahead(index);
            return Ok(block);
        }
        if let Some(stored) = self.ensure_store().get(index) {
            let block = Arc::new(stored);
            self.hot.insert(index, Arc::clone(&block));
            self.schedule_readahead(index);
            return Ok(block);
        }
        let prefetched = self
            .prefetch
            .as_ref()
            .and_then(|prefetch| prefetch.take(index));
        let data = match prefetched {
            Some(Ok(data)) => data,
            Some(Err(error)) => {
                log::debug!("readahead of block {index} failed, retrying on demand: {error}");
                self.fetch_block(index)?
            }
            None => self.fetch_block(index)?,
        };
        let block = Arc::new(data);
        self.ensure_store().put(index, &block);
        self.hot.insert(index, Arc::clone(&block));
        self.schedule_readahead(index);
        Ok(block)
    }

    fn fetch_block(&self, index: u64) -> io::Result<Vec<u8>> {
        let offset = index * self.block_bytes as u64;
        self.fetcher
            .fetch(&self.path, offset, self.block_bytes as u32)
            .map_err(|error| io::Error::other(format!("remote read failed: {error}")))
    }

    fn holds(&mut self, index: u64) -> bool {
        self.hot.blocks.contains_key(&index) || self.ensure_store().get(index).is_some()
    }

    /// Start pulling the next few blocks after `index` that are not cached yet.
    ///
    /// This is deliberately non-blocking. A failed speculative read is retried
    /// normally if playback actually reaches that block.
    fn schedule_readahead(&mut self, index: u64) {
        if self.readahead == 0 || self.last_readahead == Some(index) {
            return;
        }
        self.last_readahead = Some(index);
        if self.prefetch.is_none() {
            self.prefetch = Some(Prefetch::spawn(
                Arc::clone(&self.fetcher),
                self.path.clone(),
                self.block_bytes,
                self.readahead,
            ));
        }
        let last = self.size.div_ceil(self.block_bytes as u64);
        for ahead in 1..=self.readahead as u64 {
            let next = index + ahead;
            if next >= last || self.holds(next) {
                continue;
            }
            self.prefetch.as_ref().expect("just built").schedule(next);
        }
    }
}

impl Read for RemoteFile {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        if buf.is_empty() || self.position >= self.size {
            return Ok(0);
        }
        let index = self.position / self.block_bytes as u64;
        let within = (self.position % self.block_bytes as u64) as usize;

        let block = self.block(index)?;
        if within >= block.len() {
            // Past the end of a short final block.
            return Ok(0);
        }
        let available = &block[within..];
        let taken = available.len().min(buf.len());
        buf[..taken].copy_from_slice(&available[..taken]);
        self.position += taken as u64;

        Ok(taken)
    }
}

impl Seek for RemoteFile {
    fn seek(&mut self, pos: SeekFrom) -> io::Result<u64> {
        let target = match pos {
            SeekFrom::Start(offset) => offset as i128,
            SeekFrom::Current(delta) => self.position as i128 + delta as i128,
            SeekFrom::End(delta) => self.size as i128 + delta as i128,
        };
        if target < 0 {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "cannot seek before the start of the file",
            ));
        }
        // Seeking past the end is legal and reads return nothing, matching a
        // real file.
        self.position = target as u64;
        let target_block = self.position / self.block_bytes as u64;
        let inside_active_window = self.hot.blocks.contains_key(&target_block)
            || self.last_readahead.is_some_and(|anchor| {
                target_block >= anchor
                    && target_block <= anchor.saturating_add(self.readahead as u64)
            });
        if !inside_active_window {
            self.last_readahead = None;
            if let Some(prefetch) = &self.prefetch {
                prefetch.discard_ready();
            }
        }
        Ok(self.position)
    }
}

/// Fetches ranges over a Minot service, from synchronous code.
///
/// The bridge uses a dedicated OS thread. A caller may already be inside a
/// Tokio runtime. Blocking that runtime's worker on its own future deadlocks.
/// The dedicated thread can safely block on the runtime handle.
pub struct NetworkFetcher {
    requests: std::sync::mpsc::Sender<FetchRequest>,
}

struct FetchRequest {
    path: String,
    offset: u64,
    len: u32,
    reply: std::sync::mpsc::Sender<Result<Vec<u8>>>,
}

impl NetworkFetcher {
    /// `fetch` is run on the given runtime, from a thread of this fetcher's own.
    ///
    /// Takes a closure so the transport stays out of this module and tests can
    /// drive it without a network.
    pub fn spawn<F, Fut>(handle: tokio::runtime::Handle, fetch: F) -> Self
    where
        F: Fn(String, u64, u32) -> Fut + Send + 'static,
        Fut: std::future::Future<Output = Result<Vec<u8>>> + Send,
    {
        let (requests, incoming) = std::sync::mpsc::channel::<FetchRequest>();
        std::thread::Builder::new()
            .name("marina-range-fetch".to_string())
            .spawn(move || {
                // Ends when the last `NetworkFetcher` is dropped and the channel
                // closes, so the thread cannot outlive its users.
                while let Ok(request) = incoming.recv() {
                    let result = handle.block_on(fetch(request.path, request.offset, request.len));
                    // A gone receiver means the reader stopped caring. Nothing
                    // to do but drop the bytes.
                    let _ = request.reply.send(result);
                }
            })
            .expect("the range-fetch thread should start");
        Self { requests }
    }
}

impl RangeFetcher for NetworkFetcher {
    fn fetch(&self, path: &str, offset: u64, len: u32) -> Result<Vec<u8>> {
        let (reply, answer) = std::sync::mpsc::channel();
        self.requests
            .send(FetchRequest {
                path: path.to_string(),
                offset,
                len,
                reply,
            })
            .map_err(|_| anyhow::anyhow!("the range-fetch thread has stopped"))?;
        answer
            .recv()
            .map_err(|_| anyhow::anyhow!("the range-fetch thread dropped the request"))?
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    struct GatedFetcher {
        bytes: Vec<u8>,
        release_readahead: Arc<(Mutex<bool>, Condvar)>,
    }

    impl RangeFetcher for GatedFetcher {
        fn fetch(&self, _path: &str, offset: u64, len: u32) -> Result<Vec<u8>> {
            if offset > 0 {
                let (lock, released) = &*self.release_readahead;
                let mut open = lock.lock().unwrap();
                while !*open {
                    open = released.wait(open).unwrap();
                }
            }
            let start = offset as usize;
            let end = (start + len as usize).min(self.bytes.len());
            Ok(self.bytes[start..end].to_vec())
        }
    }

    fn payload(len: usize) -> Vec<u8> {
        (0..len).map(|i| (i.wrapping_mul(31) % 251) as u8).collect()
    }

    /// A local file plus a `RemoteFile` reading it through a counting fetcher.
    fn fixture(
        size: usize,
        block_bytes: usize,
    ) -> (
        tempfile::TempDir,
        Vec<u8>,
        RemoteFile,
        Arc<CountingFetcher<LocalFetcher>>,
    ) {
        let dir = tempfile::tempdir().unwrap();
        let bytes = payload(size);
        let mut file = std::fs::File::create(dir.path().join("data.bin")).unwrap();
        file.write_all(&bytes).unwrap();
        file.sync_all().unwrap();

        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let remote = RemoteFile::new(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            size as u64,
        )
        .with_block_bytes(block_bytes)
        .with_readahead(0);
        (dir, bytes, remote, fetcher)
    }

    #[test]
    fn a_sequential_read_matches_the_file_exactly() {
        let (_dir, bytes, mut remote, _) = fixture(300_000, 4096);
        let mut read = Vec::new();
        remote.read_to_end(&mut read).unwrap();
        assert_eq!(read, bytes, "a streamed read must equal the local bytes");
    }

    #[test]
    fn random_seeks_return_the_same_bytes_as_the_file() {
        let (_dir, bytes, mut remote, _) = fixture(200_000, 4096);
        // Deterministic pseudo-random offsets, so a failure is reproducible.
        let mut offset = 12345usize;
        for _ in 0..500 {
            offset = offset.wrapping_mul(1103515245).wrapping_add(12345) % bytes.len();
            let len = (offset % 5000) + 1;
            let end = (offset + len).min(bytes.len());

            remote.seek(SeekFrom::Start(offset as u64)).unwrap();
            let mut buffer = vec![0u8; end - offset];
            remote.read_exact(&mut buffer).unwrap();
            assert_eq!(
                buffer,
                &bytes[offset..end],
                "mismatch reading {} bytes at {offset}",
                end - offset
            );
        }
    }

    #[test]
    fn reading_past_the_end_returns_nothing() {
        let (_dir, _bytes, mut remote, _) = fixture(1000, 4096);
        remote.seek(SeekFrom::Start(5000)).unwrap();
        let mut buffer = [0u8; 16];
        assert_eq!(remote.read(&mut buffer).unwrap(), 0);
    }

    #[test]
    fn a_short_final_block_is_handled() {
        // Deliberately not a multiple of the block size.
        let (_dir, bytes, mut remote, _) = fixture(4096 * 3 + 17, 4096);
        remote.seek(SeekFrom::End(-17)).unwrap();
        let mut tail = Vec::new();
        remote.read_to_end(&mut tail).unwrap();
        assert_eq!(tail, &bytes[bytes.len() - 17..]);
    }

    #[test]
    fn seeking_before_the_start_is_an_error() {
        let (_dir, _bytes, mut remote, _) = fixture(1000, 4096);
        assert!(remote.seek(SeekFrom::Start(10)).is_ok());
        assert!(remote.seek(SeekFrom::Current(-100)).is_err());
        assert!(remote.seek(SeekFrom::End(-5000)).is_err());
    }

    /// The guard for the finding that motivated block caching at all.
    ///
    /// An MCAP summary read is thousands of tiny sequential reads. Served
    /// naively that is thousands of round trips. Served in blocks it is a
    /// handful. If this ever regresses, opening a bag over a real link goes
    /// from under a second to several minutes.
    #[test]
    fn an_open_costs_only_a_handful_of_requests() {
        let (_dir, _bytes, mut remote, fetcher) = fixture(300_000, DEFAULT_BLOCK_BYTES);

        // 2130 reads of ~59 bytes, the shape Phase 0 measured.
        let mut buffer = [0u8; 59];
        for _ in 0..2130 {
            if remote.read(&mut buffer).unwrap() == 0 {
                remote.seek(SeekFrom::Start(0)).unwrap();
            }
        }

        assert!(
            fetcher.requests() <= 10,
            "2130 small reads must not become {} round trips — block caching is \
             what makes opening a remote bag possible at all",
            fetcher.requests()
        );
    }

    /// Disk mode: a second reader pays nothing for what the first fetched.
    #[test]
    fn a_disk_cache_is_reused_by_a_later_read() {
        let dir = tempfile::tempdir().unwrap();
        let cache = tempfile::tempdir().unwrap();
        let bytes = payload(64 * 1024 * 3);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();

        let mode = CacheMode::Disk {
            root: cache.path().to_path_buf(),
            validity: "hash-1".to_string(),
        };

        let first_requests = {
            let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
                root: dir.path().to_path_buf(),
            }));
            let mut remote = RemoteFile::with_cache(
                Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
                "data.bin",
                bytes.len() as u64,
                &mode,
            )
            .with_readahead(0);
            let mut read = Vec::new();
            remote.read_to_end(&mut read).unwrap();
            assert_eq!(read, bytes);
            assert!(remote.is_complete(), "a full read should fill the cache");
            fetcher.requests()
        };
        assert!(first_requests > 0, "the first read must actually fetch");

        // A fresh file, fresh fetcher, same cache directory.
        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let mut remote = RemoteFile::with_cache(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
            &mode,
        )
        .with_readahead(0);
        let mut again = Vec::new();
        remote.read_to_end(&mut again).unwrap();

        assert_eq!(
            again, bytes,
            "the cached read must match the original bytes"
        );
        assert_eq!(
            fetcher.requests(),
            0,
            "a fully cached file must be read without touching the network"
        );
    }

    /// A read interrupted partway leaves usable progress behind.
    #[test]
    fn a_partial_read_resumes_from_saved_blocks() {
        let dir = tempfile::tempdir().unwrap();
        let cache = tempfile::tempdir().unwrap();
        let bytes = payload(64 * 1024 * 6);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();
        let mode = CacheMode::Disk {
            root: cache.path().to_path_buf(),
            validity: "hash-1".to_string(),
        };

        {
            let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
                root: dir.path().to_path_buf(),
            }));
            let mut remote = RemoteFile::with_cache(
                Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
                "data.bin",
                bytes.len() as u64,
                &mode,
            )
            .with_block_bytes(64 * 1024)
            .with_readahead(0);
            // Only the first half, then abandoned.
            let mut half = vec![0u8; bytes.len() / 2];
            remote.read_exact(&mut half).unwrap();
            assert!(!remote.is_complete());
        }

        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let mut remote = RemoteFile::with_cache(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
            &mode,
        )
        .with_block_bytes(64 * 1024)
        .with_readahead(0);
        let mut all = Vec::new();
        remote.read_to_end(&mut all).unwrap();

        assert_eq!(all, bytes);
        assert!(
            fetcher.requests() < 6,
            "the second read should only fetch what the first did not, got {} requests \
             for a 6-block file",
            fetcher.requests()
        );
        assert!(remote.is_complete());
    }

    /// Ephemeral mode: memory stays flat no matter how large the file is, and
    /// nothing is written anywhere.
    #[test]
    fn an_ephemeral_read_is_bounded_in_memory_and_writes_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let cache = tempfile::tempdir().unwrap();
        // Far more blocks than the hot window can hold.
        let bytes = payload(4096 * 200);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();

        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let mut remote = RemoteFile::with_cache(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
            &CacheMode::Ephemeral,
        )
        .with_block_bytes(4096)
        .with_hot_blocks(4)
        .with_readahead(0);

        let mut read = Vec::new();
        remote.read_to_end(&mut read).unwrap();

        assert_eq!(read, bytes, "an ephemeral read still returns every byte");
        assert!(
            remote.hot_blocks() <= 4,
            "memory must stay bounded while reading a 200-block file, held {}",
            remote.hot_blocks()
        );
        assert!(!remote.persists());
        assert!(
            std::fs::read_dir(cache.path()).unwrap().count() == 0,
            "an ephemeral read must leave nothing on disk"
        );
    }

    /// The distinction the two modes exist for, stated as a test.
    #[test]
    fn only_disk_mode_leaves_a_local_copy() {
        let dir = tempfile::tempdir().unwrap();
        let cache = tempfile::tempdir().unwrap();
        let bytes = payload(8192);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();

        for (mode, should_persist) in [
            (
                CacheMode::Disk {
                    root: cache.path().to_path_buf(),
                    validity: "hash".to_string(),
                },
                true,
            ),
            (CacheMode::Ephemeral, false),
        ] {
            let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
                root: dir.path().to_path_buf(),
            }));
            let mut remote = RemoteFile::with_cache(
                Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
                "data.bin",
                bytes.len() as u64,
                &mode,
            );
            let mut read = Vec::new();
            remote.read_to_end(&mut read).unwrap();
            assert_eq!(read, bytes, "both modes must return identical bytes");
            assert_eq!(remote.persists(), should_persist);
            assert_eq!(remote.is_complete(), should_persist);
        }
    }

    #[test]
    fn a_cached_block_is_not_fetched_twice() {
        let (_dir, _bytes, mut remote, fetcher) = fixture(100_000, 65536);
        let mut buffer = [0u8; 128];
        remote.seek(SeekFrom::Start(0)).unwrap();
        remote.read_exact(&mut buffer).unwrap();
        let after_first = fetcher.requests();

        // Same block, different offsets within it.
        for offset in [10u64, 200, 4000, 60000] {
            remote.seek(SeekFrom::Start(offset)).unwrap();
            remote.read_exact(&mut buffer).unwrap();
        }
        assert_eq!(
            fetcher.requests(),
            after_first,
            "re-reading inside a cached block must not touch the network"
        );
    }

    /// The deadlock this design exists to avoid.
    ///
    /// A synchronous reader used from inside a Tokio runtime must not wedge.
    /// Blocking a worker thread on a future that needs the same runtime is the
    /// classic way to hang, and it is exactly what a bag reader driven from an
    /// async task would do.
    #[test]
    fn reading_from_inside_a_runtime_does_not_deadlock() {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .worker_threads(2)
            .enable_all()
            .build()
            .unwrap();

        let dir = tempfile::tempdir().unwrap();
        let bytes = payload(200_000);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();
        let root = dir.path().to_path_buf();

        let fetcher = Arc::new(NetworkFetcher::spawn(
            runtime.handle().clone(),
            move |path, offset, len| {
                let root = root.clone();
                async move {
                    // Genuinely async, so the future must make progress on the
                    // runtime for the read to complete.
                    tokio::task::yield_now().await;
                    LocalFetcher { root }.fetch(&path, offset, len)
                }
            },
        ));

        let mut remote = RemoteFile::new(
            fetcher as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
        )
        .with_block_bytes(16384)
        .with_readahead(2);

        // Driven from inside the runtime, on a blocking-friendly thread, which
        // is how a bag reader would actually be called.
        let read = runtime.block_on(async move {
            tokio::task::spawn_blocking(move || {
                let mut out = Vec::new();
                remote.read_to_end(&mut out).unwrap();
                out
            })
            .await
            .unwrap()
        });
        assert_eq!(read, bytes, "the read must complete and match");
    }

    #[test]
    fn readahead_pulls_forward_blocks_without_changing_what_is_read() {
        let dir = tempfile::tempdir().unwrap();
        let bytes = payload(65536 * 8);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();
        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let mut remote = RemoteFile::new(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
        )
        .with_block_bytes(65536)
        .with_readahead(3);

        let mut first = vec![0u8; 65536];
        remote.read_exact(&mut first).unwrap();
        assert_eq!(first, bytes[..65536]);
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
        while fetcher.requests() == 1 && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert!(
            fetcher.requests() > 1,
            "reading a block should have started background readahead"
        );

        // The content must be unaffected by prefetching.
        let mut rest = Vec::new();
        remote.read_to_end(&mut rest).unwrap();
        assert_eq!(rest, &bytes[65536..]);
    }

    #[test]
    fn readahead_never_holds_up_the_current_block() {
        let bytes = payload(4096 * 5);
        let gate = Arc::new((Mutex::new(false), Condvar::new()));
        let fetcher = Arc::new(GatedFetcher {
            bytes: bytes.clone(),
            release_readahead: Arc::clone(&gate),
        });
        let mut remote = RemoteFile::new(
            fetcher as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
        )
        .with_block_bytes(4096)
        .with_readahead(3);

        let (finished, answer) = std::sync::mpsc::channel();
        std::thread::spawn(move || {
            let mut byte = [0u8; 1];
            remote.read_exact(&mut byte).unwrap();
            finished.send(byte[0]).unwrap();
        });

        let result = answer.recv_timeout(std::time::Duration::from_millis(250));
        let (lock, released) = &*gate;
        *lock.lock().unwrap() = true;
        released.notify_all();

        assert_eq!(
            result.expect("the current block must not wait for speculative reads"),
            bytes[0]
        );
    }

    #[test]
    fn a_forward_seek_inside_the_window_keeps_prefetched_blocks() {
        let dir = tempfile::tempdir().unwrap();
        let bytes = payload(4096 * 4);
        std::fs::write(dir.path().join("data.bin"), &bytes).unwrap();
        let fetcher = Arc::new(CountingFetcher::new(LocalFetcher {
            root: dir.path().to_path_buf(),
        }));
        let mut remote = RemoteFile::new(
            Arc::clone(&fetcher) as Arc<dyn RangeFetcher>,
            "data.bin",
            bytes.len() as u64,
        )
        .with_block_bytes(4096)
        .with_readahead(3);

        let mut first = [0u8; 1];
        remote.read_exact(&mut first).unwrap();

        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(1);
        while fetcher.requests() < 4 && std::time::Instant::now() < deadline {
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(fetcher.requests(), 4, "the complete window should be warm");

        remote.seek(SeekFrom::Start(4096 * 2 + 17)).unwrap();
        let mut byte = [0u8; 1];
        remote.read_exact(&mut byte).unwrap();
        assert_eq!(byte[0], bytes[4096 * 2 + 17]);
        assert_eq!(
            fetcher.requests(),
            4,
            "a normal forward seek must use the prefetched block, not refetch it"
        );
    }
}
