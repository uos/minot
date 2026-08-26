//! Where fetched blocks live between reads.
//!
//! Two policies share one mechanism. A read asks the store for a block. On a miss it
//! fetches and offers the block back. What the store does with it is the whole
//! difference between "streaming warms a local copy" and "streaming leaves no
//! trace".
//!
//! # Disk
//!
//! [`DiskBlockStore`] keeps a sparse file plus a map of which blocks it holds.
//! A partial read survives the process, so re-reading the same dataset costs
//! nothing, and once every block is present the sparse file *is* the complete
//! file — at which point streaming and pulling have converged, which is the
//! property the whole design is built around.
//!
//! # Ephemeral
//!
//! [`NullBlockStore`] keeps nothing. Blocks live only in the small in-memory
//! window in front of the store and are dropped as the read moves past them, so
//! memory stays bounded and the disk is never touched. For reading a dataset
//! you deliberately do not want a copy of — a quick look at someone else's
//! recording, a machine with no room, or anywhere leaving data behind would be
//! wrong.

use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};

/// Somewhere fetched blocks can be kept.
pub trait BlockStore: Send + Sync {
    /// A previously stored block, if this store has it.
    fn get(&self, index: u64) -> Option<Vec<u8>>;
    /// Offer a freshly fetched block. Failures are not fatal to a read — a
    /// store that cannot keep a block just means it will be fetched again.
    fn put(&self, index: u64, data: &[u8]);
    /// Whether every block of the file is present.
    fn is_complete(&self) -> bool {
        false
    }
    /// Whether anything is kept at all. Used to describe behaviour to a user.
    fn persists(&self) -> bool {
        false
    }
}

/// Keeps nothing: the ephemeral, leave-no-trace policy.
#[derive(Debug, Default)]
pub struct NullBlockStore;

impl BlockStore for NullBlockStore {
    fn get(&self, _index: u64) -> Option<Vec<u8>> {
        None
    }
    fn put(&self, _index: u64, _data: &[u8]) {}
}

/// The layout on disk for one cached file.
///
/// `<root>/<escaped rel path>.part`   — sparse data
/// `<root>/<escaped rel path>.blocks` — one byte per block, 1 = present
///
/// The map uses one byte per block. A 640 MB file
/// at 1 MiB blocks needs 640 bytes either way once the filesystem has rounded
/// up, and a file you can read with `xxd` is worth more during a bad afternoon
/// than the bytes saved.
struct Layout {
    data: PathBuf,
    map: PathBuf,
}

fn layout(root: &Path, relative: &str) -> Layout {
    // Flattened, because a relative path may contain directories that do not
    // exist here and escaping is one fewer thing to get wrong.
    let escaped = relative.replace(['/', '\\'], "__");
    Layout {
        data: root.join(format!("{escaped}.part")),
        map: root.join(format!("{escaped}.blocks")),
    }
}

pub struct DiskBlockStore {
    data: std::sync::Mutex<std::fs::File>,
    map_path: PathBuf,
    /// Mirror of the on-disk map, so a hit costs no syscall to discover.
    present: std::sync::Mutex<Vec<u8>>,
    block_bytes: usize,
    size: u64,
    blocks: u64,
}

impl std::fmt::Debug for DiskBlockStore {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DiskBlockStore")
            .field("size", &self.size)
            .field("blocks", &self.blocks)
            .field("cached", &self.cached_blocks())
            .finish_non_exhaustive()
    }
}

impl DiskBlockStore {
    /// Open (or start) a cache for one file.
    ///
    /// `validity` identifies *which version* of the file this is — a bundle
    /// hash, or anything that changes when the contents do. A cache whose
    /// validity or size must match. Mismatched cached data is discarded before new
    /// data, because serving half of one version and half of another would be
    /// silent corruption of exactly the kind nobody would think to look for.
    pub fn open(
        root: &Path,
        relative: &str,
        size: u64,
        block_bytes: usize,
        validity: &str,
    ) -> Result<Self> {
        anyhow::ensure!(block_bytes > 0, "block size must be greater than zero");
        std::fs::create_dir_all(root)
            .with_context(|| format!("could not create the stream cache at {}", root.display()))?;

        let paths = layout(root, relative);
        let blocks = size.div_ceil(block_bytes as u64);
        // Header: what this cache is of. One line, so it can be read by eye.
        let stamp = format!("v1 {size} {block_bytes} {validity}\n");

        let existing_stamp = std::fs::read_to_string(&paths.map)
            .ok()
            .and_then(|text| text.lines().next().map(|line| format!("{line}\n")));
        let reusable = existing_stamp.as_deref() == Some(stamp.as_str());

        if !reusable {
            if paths.map.exists() || paths.data.exists() {
                log::debug!(
                    "discarding a stale stream cache for '{relative}': it was written for a \
                     different version or block size"
                );
            }
            let _ = std::fs::remove_file(&paths.map);
            let _ = std::fs::remove_file(&paths.data);
        }

        let mut present = vec![0u8; blocks as usize];
        if reusable {
            let raw = std::fs::read(&paths.map).unwrap_or_default();
            // Everything after the header line is the map.
            if let Some(offset) = raw.iter().position(|byte| *byte == b'\n') {
                let body = &raw[offset + 1..];
                let copy = body.len().min(present.len());
                present[..copy].copy_from_slice(&body[..copy]);
            }
        } else {
            let mut map = std::fs::File::create(&paths.map)
                .with_context(|| format!("could not create {}", paths.map.display()))?;
            map.write_all(stamp.as_bytes())?;
            map.write_all(&present)?;
            map.sync_all()?;
        }

        let data = std::fs::OpenOptions::new()
            .read(true)
            .write(true)
            .create(true)
            .truncate(false)
            .open(&paths.data)
            .with_context(|| format!("could not open {}", paths.data.display()))?;
        // Sparse: setting the length costs no blocks until they are written.
        data.set_len(size)?;

        Ok(Self {
            data: std::sync::Mutex::new(data),
            map_path: paths.map,
            present: std::sync::Mutex::new(present),
            block_bytes,
            size,
            blocks,
        })
    }

    /// How many blocks are held, for progress reporting.
    pub fn cached_blocks(&self) -> u64 {
        self.present
            .lock()
            .map(|present| present.iter().filter(|byte| **byte == 1).count() as u64)
            .unwrap_or(0)
    }

    pub fn total_blocks(&self) -> u64 {
        self.blocks
    }

    /// Fraction of the file held locally, 0.0 to 1.0.
    pub fn cached_fraction(&self) -> f32 {
        if self.blocks == 0 {
            return 1.0;
        }
        self.cached_blocks() as f32 / self.blocks as f32
    }

    /// The path of the sparse file. Once [`BlockStore::is_complete`] is true it
    /// is a byte-exact copy of the remote file and can simply be moved.
    pub fn data_path(&self) -> PathBuf {
        self.map_path.with_extension("part")
    }

    /// Record that a block is present, in memory and on disk.
    fn mark(&self, index: u64) -> Result<()> {
        let mut present = self
            .present
            .lock()
            .map_err(|_| anyhow::anyhow!("the block map lock was poisoned"))?;
        if present.get(index as usize).copied() == Some(1) {
            return Ok(());
        }
        if let Some(slot) = present.get_mut(index as usize) {
            *slot = 1;
        }
        // Only this one byte is rewritten, so marking a block does not cost a
        // rewrite of the whole map.
        let mut map = std::fs::OpenOptions::new()
            .write(true)
            .open(&self.map_path)?;
        let header = std::fs::read(&self.map_path)?
            .iter()
            .position(|byte| *byte == b'\n')
            .map(|offset| offset + 1)
            .unwrap_or(0) as u64;
        map.seek(SeekFrom::Start(header + index))?;
        map.write_all(&[1u8])?;
        Ok(())
    }
}

impl BlockStore for DiskBlockStore {
    fn get(&self, index: u64) -> Option<Vec<u8>> {
        let known = {
            let present = self.present.lock().ok()?;
            present.get(index as usize).copied() == Some(1)
        };
        if !known {
            return None;
        }
        let offset = index * self.block_bytes as u64;
        let length = (self.size.saturating_sub(offset)).min(self.block_bytes as u64) as usize;
        let mut buffer = vec![0u8; length];
        let mut data = self.data.lock().ok()?;
        data.seek(SeekFrom::Start(offset)).ok()?;
        data.read_exact(&mut buffer).ok()?;
        Some(buffer)
    }

    fn put(&self, index: u64, block: &[u8]) {
        let offset = index * self.block_bytes as u64;
        let written = (|| -> Result<()> {
            let mut data = self
                .data
                .lock()
                .map_err(|_| anyhow::anyhow!("the block file lock was poisoned"))?;
            data.seek(SeekFrom::Start(offset))?;
            data.write_all(block)?;
            drop(data);
            self.mark(index)
        })();
        if let Err(error) = written {
            // Not fatal: the read that prompted this still has its bytes, and a
            // block that failed to cache is simply fetched again next time.
            log::debug!("could not cache block {index}: {error}");
        }
    }

    fn is_complete(&self) -> bool {
        self.cached_blocks() == self.blocks
    }

    fn persists(&self) -> bool {
        true
    }
}

/// Whether a cached file is complete, without opening it for reading.
///
/// Used by promotion to decide whether a streamed dataset has become an
/// ordinary local one. Returns the sparse file's path when it is complete —
/// at that point it is byte-for-byte the remote file and can simply be moved.
pub fn completed_file(
    root: &Path,
    relative: &str,
    size: u64,
    block_bytes: usize,
    validity: &str,
) -> Option<PathBuf> {
    let store = DiskBlockStore::open(root, relative, size, block_bytes, validity).ok()?;
    // `open` discards a cache whose stamp does not match, so reaching here with
    // every block present means these bytes really are this version's.
    store.is_complete().then(|| store.data_path())
}

/// How a [`super::RemoteFile`] treats the blocks it fetches.
#[derive(Debug, Clone)]
pub enum CacheMode {
    /// Keep blocks in a sparse file under `root`.
    ///
    /// Re-reading costs nothing, a partial read survives a restart, and a
    /// complete read leaves a byte-exact local copy. `validity` must change
    /// whenever the remote file's contents do — a bundle hash is ideal.
    Disk { root: PathBuf, validity: String },
    /// Keep nothing. Blocks are dropped as the read moves past them. Memory
    /// stays bounded and the disk is never written.
    Ephemeral,
}

impl CacheMode {
    pub fn build(
        &self,
        relative: &str,
        size: u64,
        block_bytes: usize,
    ) -> Result<Box<dyn BlockStore>> {
        match self {
            CacheMode::Disk { root, validity } => Ok(Box::new(DiskBlockStore::open(
                root,
                relative,
                size,
                block_bytes,
                validity,
            )?)),
            CacheMode::Ephemeral => Ok(Box::new(NullBlockStore)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn block(byte: u8, len: usize) -> Vec<u8> {
        vec![byte; len]
    }

    #[test]
    fn a_stored_block_comes_back_identical() {
        let dir = tempfile::tempdir().unwrap();
        let store = DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "hash").unwrap();
        assert!(store.get(0).is_none(), "nothing is cached to begin with");

        store.put(1, &block(7, 4096));
        assert_eq!(store.get(1), Some(block(7, 4096)));
    }

    #[test]
    fn a_cache_survives_being_reopened() {
        let dir = tempfile::tempdir().unwrap();
        {
            let store =
                DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "hash").unwrap();
            store.put(2, &block(9, 4096));
        }
        let reopened =
            DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "hash").unwrap();
        assert_eq!(
            reopened.get(2),
            Some(block(9, 4096)),
            "a block cached before a restart must still be there"
        );
        assert_eq!(reopened.cached_blocks(), 1);
    }

    #[test]
    fn a_cache_for_a_different_version_is_discarded() {
        let dir = tempfile::tempdir().unwrap();
        {
            let store =
                DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "old").unwrap();
            store.put(0, &block(1, 4096));
        }
        let reopened = DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "new").unwrap();
        assert!(
            reopened.get(0).is_none(),
            "mixing blocks from two versions would be silent corruption"
        );
        assert_eq!(reopened.cached_blocks(), 0);
    }

    #[test]
    fn a_cache_for_a_different_size_is_discarded() {
        let dir = tempfile::tempdir().unwrap();
        {
            let store =
                DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 3, 4096, "hash").unwrap();
            store.put(0, &block(1, 4096));
        }
        let reopened =
            DiskBlockStore::open(dir.path(), "run.mcap", 4096 * 9, 4096, "hash").unwrap();
        assert!(reopened.get(0).is_none());
    }

    #[test]
    fn completeness_is_reached_only_when_every_block_is_present() {
        let dir = tempfile::tempdir().unwrap();
        // Deliberately not a multiple of the block size: the last block is short.
        let size = 4096 * 2 + 10;
        let store = DiskBlockStore::open(dir.path(), "run.mcap", size, 4096, "hash").unwrap();
        assert_eq!(store.total_blocks(), 3);

        store.put(0, &block(1, 4096));
        store.put(1, &block(2, 4096));
        assert!(!store.is_complete());
        assert!((store.cached_fraction() - 2.0 / 3.0).abs() < 0.001);

        store.put(2, &block(3, 10));
        assert!(store.is_complete());
        assert_eq!(store.cached_fraction(), 1.0);
    }

    #[test]
    fn a_complete_cache_is_the_file_itself() {
        let dir = tempfile::tempdir().unwrap();
        let size = 4096 + 5;
        let store = DiskBlockStore::open(dir.path(), "run.mcap", size, 4096, "hash").unwrap();
        store.put(0, &block(0xAB, 4096));
        store.put(1, &block(0xCD, 5));
        assert!(store.is_complete());

        // The sparse file can simply be moved into place once complete.
        let written = std::fs::read(store.data_path()).unwrap();
        assert_eq!(written.len(), size as usize);
        assert_eq!(&written[..4096], &block(0xAB, 4096)[..]);
        assert_eq!(&written[4096..], &block(0xCD, 5)[..]);
    }

    #[test]
    fn the_ephemeral_store_keeps_nothing() {
        let store = NullBlockStore;
        store.put(0, &block(1, 4096));
        assert!(
            store.get(0).is_none(),
            "the whole point is that nothing is retained"
        );
        assert!(!store.persists());
        assert!(!store.is_complete());
    }

    #[test]
    fn ephemeral_mode_writes_nothing_to_disk() {
        let dir = tempfile::tempdir().unwrap();
        let store = CacheMode::Ephemeral.build("run.mcap", 8192, 4096).unwrap();
        store.put(0, &block(1, 4096));
        assert_eq!(
            std::fs::read_dir(dir.path()).unwrap().count(),
            0,
            "ephemeral reads must leave no trace"
        );
    }

    #[test]
    fn a_flattened_name_cannot_escape_the_cache_directory() {
        let root = Path::new("/cache");
        let paths = layout(root, "nested/deep/run.mcap");
        assert_eq!(paths.data, root.join("nested__deep__run.mcap.part"));
        assert!(
            paths.data.starts_with(root),
            "a relative path with directories must stay inside the cache root"
        );
    }
}
