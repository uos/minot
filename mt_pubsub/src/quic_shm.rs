//! Shared memory between QUIC peers on one host, with the `shm` feature as for
//! Zenoh. Without it, every sample goes over QUIC.
//!
//! A large sample is written once into an anonymous memory object, and its
//! file descriptor is passed to every subscriber on the same host over a Unix
//! socket; the subscriber maps it read-only. The kernel counts the references,
//! so the memory is freed when the last process lets go of it, also after a
//! crash. There is no pool, no named segment left behind and no collector.
//!
//! The memory object is a sealed `memfd` on Linux and an already unlinked
//! POSIX shared memory object elsewhere (macOS). Only Linux can seal: there a
//! subscriber refuses memory its publisher could still change or shrink. On
//! macOS a subscriber trusts the publisher, which the socket's permissions
//! limit to processes of the same user.
//!
//! Peers find each other through the socket path in the QUIC hello: a path
//! that can be connected to is a node on this host. QUIC keeps carrying
//! everything else, including order and acknowledgements of reliable samples;
//! see `quic.rs`.

// Without shared memory, which needs the `shm` feature and a Unix other than
// iOS and Android, the settings and counters below go unused.
#![cfg_attr(
    not(all(
        feature = "shm",
        unix,
        not(any(target_os = "ios", target_os = "android"))
    )),
    allow(dead_code)
)]

use std::sync::atomic::{AtomicU64, Ordering};

fn limit(name: &str, default: usize) -> usize {
    std::env::var(name)
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(default)
}

fn disabled() -> bool {
    std::env::var("MINOT_SHM_DISABLED").is_ok_and(|v| v == "1" || v.eq_ignore_ascii_case("true"))
}

static SENT: AtomicU64 = AtomicU64::new(0);
static RECEIVED: AtomicU64 = AtomicU64::new(0);
/// Samples this process has passed to and received from peers through shared
/// memory, counted once per subscribing node.
pub fn shm_samples() -> (u64, u64) {
    (
        SENT.load(Ordering::Relaxed),
        RECEIVED.load(Ordering::Relaxed),
    )
}

#[cfg(all(
    feature = "shm",
    unix,
    not(any(target_os = "ios", target_os = "android"))
))]
pub(crate) use unix::{Hub, Lazy, Link};

#[cfg(not(all(
    feature = "shm",
    unix,
    not(any(target_os = "ios", target_os = "android"))
)))]
pub(crate) use none::{Hub, Lazy, Link};

#[cfg(all(
    feature = "shm",
    unix,
    not(any(target_os = "ios", target_os = "android"))
))]
mod unix {
    use super::disabled;
    use bytes::Bytes;
    use rustix::net::{
        RecvAncillaryBuffer, RecvAncillaryMessage, RecvFlags, SendAncillaryBuffer,
        SendAncillaryMessage, SendFlags, recvmsg, sendmsg,
    };
    use std::{
        collections::HashMap,
        fs::File,
        io::{self, IoSlice, IoSliceMut},
        mem::MaybeUninit,
        os::fd::{AsFd, OwnedFd},
        path::PathBuf,
        sync::{
            Arc, Mutex,
            atomic::{AtomicU64, Ordering},
        },
        time::{Duration, Instant},
    };
    use tokio::{
        io::{AsyncReadExt, AsyncWriteExt, Interest},
        net::{UnixListener, UnixStream},
        sync::Notify,
    };
    use tokio_util::sync::CancellationToken;

    /// kind, token, topic hash, length; the descriptor rides on the first byte.
    const FRAME: usize = 25;
    const LOSSY: u8 = 0;
    const ORDERED: u8 = 1;
    /// An ordered sample whose QUIC marker never came is forgotten after this.
    const UNCLAIMED: Duration = Duration::from_secs(10);
    #[cfg(target_vendor = "apple")]
    const HOLD: Duration = Duration::from_secs(2);
    /// Open descriptors are scarce on macOS (256 per process by default).
    #[cfg(target_vendor = "apple")]
    const HELD: usize = 16;

    fn other(message: &'static str) -> io::Error {
        io::Error::other(message)
    }

    #[cfg(target_os = "linux")]
    fn anonymous() -> io::Result<OwnedFd> {
        use rustix::fs::{MemfdFlags, memfd_create};
        Ok(memfd_create(
            "minot",
            MemfdFlags::CLOEXEC | MemfdFlags::ALLOW_SEALING,
        )?)
    }
    #[cfg(not(target_os = "linux"))]
    fn anonymous() -> io::Result<OwnedFd> {
        unlinked()
    }
    /// A POSIX shared memory object whose name is removed right away, so only
    /// descriptors keep it alive.
    #[cfg(any(test, not(target_os = "linux")))]
    fn unlinked() -> io::Result<OwnedFd> {
        use rustix::{fs::Mode, shm};
        static NEXT: AtomicU64 = AtomicU64::new(0);
        // macOS allows 31 characters.
        let name = format!(
            "/mn{:x}.{:x}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        );
        let flags = shm::OFlags::CREATE | shm::OFlags::EXCL | shm::OFlags::RDWR;
        let fd = shm::open(&name, flags, Mode::RUSR | Mode::WUSR)?;
        shm::unlink(&name)?;
        Ok(fd)
    }

    /// No more writes or size changes, for anyone holding the descriptor.
    #[cfg(target_os = "linux")]
    fn sealed() -> rustix::fs::SealFlags {
        use rustix::fs::SealFlags;
        SealFlags::SHRINK | SealFlags::GROW | SealFlags::WRITE | SealFlags::SEAL
    }

    fn fill(fd: OwnedFd, bytes: &[u8]) -> io::Result<File> {
        let file = File::from(fd);
        file.set_len(bytes.len() as u64)?;
        // macOS shared memory objects cannot be written to, only mapped.
        // SAFETY: nobody else has the descriptor yet.
        let mut map = unsafe {
            memmap2::MmapOptions::new()
                .len(bytes.len())
                .map_mut(&file)?
        };
        map.copy_from_slice(bytes);
        drop(map);
        #[cfg(target_os = "linux")]
        rustix::fs::fcntl_add_seals(&file, sealed())?;
        Ok(file)
    }

    /// Map a received descriptor; `len` is what its sender announced.
    fn map(fd: OwnedFd, len: usize) -> io::Result<Bytes> {
        let file = File::from(fd);
        if len == 0 || len > super::MAX || file.metadata()?.len() < len as u64 {
            return Err(other("shared sample has an invalid size"));
        }
        // A sender that could still shrink the object would crash this process.
        #[cfg(target_os = "linux")]
        if !rustix::fs::fcntl_get_seals(&file).is_ok_and(|seals| seals.contains(sealed())) {
            return Err(other("shared sample is not sealed"));
        }
        // SAFETY: sealed against changes on Linux; elsewhere the sender is a
        // process of the same user and never touches a sample it has sent.
        let map = unsafe { memmap2::MmapOptions::new().len(len).map(&file)? };
        Ok(Bytes::from_owner(map))
    }

    /// One sample's memory object, created when the first same-host
    /// subscriber needs it.
    pub(crate) struct Lazy<'a> {
        bytes: &'a [u8],
        made: Option<Option<Arc<File>>>,
    }
    impl<'a> Lazy<'a> {
        pub(crate) fn new(bytes: &'a [u8]) -> Self {
            Self { bytes, made: None }
        }
        /// The memory object and the sample's length. The object itself may
        /// be longer: macOS rounds it up to whole pages.
        fn get(&mut self) -> io::Result<(&Arc<File>, usize)> {
            let bytes = self.bytes;
            self.made
                .get_or_insert_with(|| {
                    let worth = super::worth_sharing(bytes.len());
                    let file = worth.then(|| anonymous().and_then(|fd| fill(fd, bytes)))?;
                    file.inspect_err(|error| log::debug!("no shared memory for sample: {error}"))
                        .ok()
                        .map(Arc::new)
                })
                .as_ref()
                .map(|file| (file, bytes.len()))
                .ok_or_else(|| other("not shared"))
        }
    }

    /// Sending side: the connection to one peer's hub.
    #[derive(Debug)]
    pub(crate) struct Link {
        stream: UnixStream,
        /// A frame goes out in one piece.
        gate: Mutex<()>,
        /// macOS is reported to lose a descriptor that its sender closes
        /// before the receiver has read it, so the last few sent are held.
        #[cfg(target_vendor = "apple")]
        held: Mutex<std::collections::VecDeque<(Instant, Arc<File>)>>,
    }
    impl Link {
        /// `None` unless `path` is the hub of a node on this host.
        pub(crate) async fn connect(path: &str, id: u64) -> Option<Arc<Self>> {
            if path.is_empty() || disabled() {
                return None;
            }
            let attempt = async {
                let mut stream = UnixStream::connect(path).await?;
                stream.write_all(&id.to_le_bytes()).await?;
                io::Result::Ok(stream)
            };
            let stream = tokio::time::timeout(Duration::from_secs(1), attempt)
                .await
                .ok()?
                .ok()?;
            // Few frames in flight: a stalled subscriber must not pin memory.
            let _ = socket2::SockRef::from(&stream).set_send_buffer_size(4096);
            Some(Arc::new(Self {
                stream,
                gate: Mutex::default(),
                #[cfg(target_vendor = "apple")]
                held: Mutex::default(),
            }))
        }

        fn try_send(
            &self,
            kind: u8,
            token: u64,
            hash: u64,
            (file, len): (&Arc<File>, usize),
        ) -> io::Result<()> {
            let mut frame = [0u8; FRAME];
            frame[0] = kind;
            frame[1..9].copy_from_slice(&token.to_le_bytes());
            frame[9..17].copy_from_slice(&hash.to_le_bytes());
            frame[17..].copy_from_slice(&(len as u64).to_le_bytes());
            let _gate = self.gate.lock().unwrap();
            let sent = self.stream.try_io(Interest::WRITABLE, || {
                let mut space = [MaybeUninit::uninit(); rustix::cmsg_space!(ScmRights(1))];
                let mut control = SendAncillaryBuffer::new(&mut space);
                let fds = [file.as_fd()];
                control.push(SendAncillaryMessage::ScmRights(&fds));
                #[cfg(target_os = "linux")]
                let flags = SendFlags::NOSIGNAL;
                #[cfg(not(target_os = "linux"))]
                let flags = SendFlags::empty();
                Ok(sendmsg(
                    &self.stream,
                    &[IoSlice::new(&frame)],
                    &mut control,
                    flags,
                )?)
            })?;
            if sent != FRAME {
                // The stream is out of step; every later send fails and falls
                // back to the network.
                let _ = socket2::SockRef::from(&self.stream).shutdown(std::net::Shutdown::Both);
                return Err(other("shared memory link broke"));
            }
            #[cfg(target_vendor = "apple")]
            {
                let mut held = self.held.lock().unwrap();
                while held.len() >= HELD
                    || held
                        .front()
                        .is_some_and(|(since, _)| since.elapsed() > HOLD)
                {
                    held.pop_front();
                }
                held.push_back((Instant::now(), Arc::clone(file)));
            }
            super::SENT.fetch_add(1, Ordering::Relaxed);
            crate::transfer::shm_sent(len);
            Ok(())
        }

        /// Pass a best-effort sample. `WouldBlock`: the subscriber is behind.
        pub(crate) fn lossy(&self, hash: u64, sample: &mut Lazy) -> io::Result<()> {
            self.try_send(LOSSY, 0, hash, sample.get()?)
        }

        /// Pass an ordered sample; the returned token goes to the peer on the
        /// publisher's QUIC stream, where the peer picks the sample up in turn.
        pub(crate) async fn ordered(&self, hash: u64, sample: &mut Lazy<'_>) -> io::Result<u64> {
            // Counted for the whole process, not per link: a link that
            // replaces one to the same peer must not reuse the tokens of
            // samples that still wait at the peer's hub.
            static TOKEN: AtomicU64 = AtomicU64::new(0);
            let (file, len) = sample.get()?;
            let token = TOKEN.fetch_add(1, Ordering::Relaxed);
            loop {
                self.stream.writable().await?;
                match self.try_send(ORDERED, token, hash, (file, len)) {
                    Err(error) if error.kind() == io::ErrorKind::WouldBlock => continue,
                    result => return result.map(|()| token),
                }
            }
        }
    }

    /// Ordered samples by peer, token and topic hash, with their arrival.
    type Waiting = HashMap<(u64, u64, u64), (Instant, Bytes)>;

    /// Receiving side: the node's socket and the ordered samples that wait for
    /// their turn on a QUIC stream.
    #[derive(Debug)]
    pub(crate) struct Hub {
        path: PathBuf,
        listener: Mutex<Option<UnixListener>>,
        waiting: Mutex<Waiting>,
        arrived: Notify,
    }
    impl Drop for Hub {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.path);
        }
    }
    impl Hub {
        pub(crate) fn open(id: u64) -> Option<Arc<Self>> {
            use std::os::unix::fs::PermissionsExt;
            if disabled() {
                return None;
            }
            let path = std::env::temp_dir().join(format!("minot-{id:016x}.sock"));
            // Left behind by a node with this identifier that did not exit cleanly.
            let _ = std::fs::remove_file(&path);
            let bound = UnixListener::bind(&path).and_then(|listener| {
                std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))?;
                Ok(listener)
            });
            match bound {
                Ok(listener) => Some(Arc::new(Self {
                    path,
                    listener: Mutex::new(Some(listener)),
                    waiting: Mutex::default(),
                    arrived: Notify::new(),
                })),
                Err(error) => {
                    log::debug!("shared memory is unavailable: {error}");
                    None
                }
            }
        }
        pub(crate) fn path(&self) -> String {
            self.path.to_string_lossy().into_owned()
        }

        /// Accept links until `shutdown`; best-effort samples go to `deliver`.
        pub(crate) async fn run(
            self: Arc<Self>,
            shutdown: CancellationToken,
            deliver: impl Fn(u64, Bytes) + Clone + Send + Sync + 'static,
        ) {
            let Some(listener) = self.listener.lock().unwrap().take() else {
                return;
            };
            let mut sweep = tokio::time::interval(UNCLAIMED);
            loop {
                tokio::select! {
                    _ = shutdown.cancelled() => break,
                    // Samples whose token never came.
                    _ = sweep.tick() => self.expire(),
                    accepted = listener.accept() => {
                        let Ok((stream, _)) = accepted else { continue; };
                        let (hub, deliver, stop) = (Arc::clone(&self), deliver.clone(), shutdown.clone());
                        tokio::spawn(async move {
                            tokio::select! {
                                _ = stop.cancelled() => {}
                                ended = hub.receive(stream, deliver) => {
                                    if let Err(error) = ended {
                                        log::debug!("shared memory link ended: {error}");
                                    }
                                }
                            }
                        });
                    }
                }
            }
            let _ = std::fs::remove_file(&self.path);
        }

        async fn receive(
            &self,
            mut stream: UnixStream,
            deliver: impl Fn(u64, Bytes),
        ) -> io::Result<()> {
            let mut peer = [0u8; 8];
            stream.read_exact(&mut peer).await?;
            let peer = u64::from_le_bytes(peer);
            loop {
                let mut frame = [0u8; FRAME];
                let mut filled = 0;
                let mut fd = None;
                // Never read past this frame: the next descriptor arrives with
                // the first byte of the next one.
                while filled < FRAME {
                    stream.readable().await?;
                    let read = stream.try_io(Interest::READABLE, || {
                        let mut space = [MaybeUninit::uninit(); rustix::cmsg_space!(ScmRights(1))];
                        let mut control = RecvAncillaryBuffer::new(&mut space);
                        #[cfg(target_os = "linux")]
                        let flags = RecvFlags::CMSG_CLOEXEC;
                        #[cfg(not(target_os = "linux"))]
                        let flags = RecvFlags::empty();
                        let read = recvmsg(
                            &stream,
                            &mut [IoSliceMut::new(&mut frame[filled..])],
                            &mut control,
                            flags,
                        )?
                        .bytes;
                        for message in control.drain() {
                            if let RecvAncillaryMessage::ScmRights(fds) = message {
                                fd = fds.last().or(fd.take());
                            }
                        }
                        Ok(read)
                    });
                    match read {
                        Ok(0) => return Ok(()),
                        Ok(read) => filled += read,
                        Err(error) if error.kind() == io::ErrorKind::WouldBlock => {}
                        Err(error) => return Err(error),
                    }
                }
                let token = u64::from_le_bytes(frame[1..9].try_into().unwrap());
                let hash = u64::from_le_bytes(frame[9..17].try_into().unwrap());
                let len = u64::from_le_bytes(frame[17..].try_into().unwrap());
                let len = usize::try_from(len).map_err(|_| other("oversized shared sample"))?;
                let bytes = map(fd.ok_or_else(|| other("frame without descriptor"))?, len)?;
                super::RECEIVED.fetch_add(1, Ordering::Relaxed);
                crate::transfer::shm_received(len);
                match frame[0] {
                    LOSSY => deliver(hash, bytes),
                    ORDERED => {
                        self.expire();
                        self.waiting
                            .lock()
                            .unwrap()
                            .insert((peer, token, hash), (Instant::now(), bytes));
                        self.arrived.notify_waiters();
                    }
                    _ => return Err(other("unknown frame")),
                }
            }
        }

        fn expire(&self) {
            self.waiting
                .lock()
                .unwrap()
                .retain(|_, (since, _)| since.elapsed() < UNCLAIMED);
        }

        /// The ordered sample of topic `hash` that `peer` announced under
        /// `token` on its stream.
        pub(crate) async fn take(
            &self,
            peer: u64,
            token: u64,
            hash: u64,
            budget: Duration,
        ) -> Option<Bytes> {
            let search = async {
                loop {
                    let arrived = self.arrived.notified();
                    tokio::pin!(arrived);
                    arrived.as_mut().enable();
                    let found = self.waiting.lock().unwrap().remove(&(peer, token, hash));
                    if let Some((_, bytes)) = found {
                        return bytes;
                    }
                    arrived.await;
                }
            };
            tokio::time::timeout(budget, search).await.ok()
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;

        fn roundtrip(fd: OwnedFd) {
            let sample: Vec<u8> = (0..300_000u32).map(|i| i as u8).collect();
            let file = fill(fd, &sample).unwrap();
            let received = file.as_fd().try_clone_to_owned().unwrap();
            assert_eq!(map(received, sample.len()).unwrap().as_ref(), &sample[..]);
        }

        #[test]
        fn anonymous_memory_roundtrips() {
            roundtrip(anonymous().unwrap());
        }

        /// The object used on macOS also exists on Linux.
        #[test]
        fn unlinked_memory_fills() {
            let sample = vec![7u8; 100_000];
            let fd = unlinked().unwrap();
            let file = File::from(fd);
            file.set_len(sample.len() as u64).unwrap();
            // The object may be longer than asked for: macOS rounds to pages.
            assert!(file.metadata().unwrap().len() >= sample.len() as u64);
            let exact = || memmap2::MmapOptions::new().len(sample.len()).clone();
            let mut writable = unsafe { exact().map_mut(&file).unwrap() };
            writable.copy_from_slice(&sample);
            drop(writable);
            let readable = unsafe { exact().map(&file).unwrap() };
            assert_eq!(&readable[..], &sample[..]);
        }

        #[test]
        fn growing_or_unsealed_memory_is_refused() {
            let file = fill(anonymous().unwrap(), &[1u8; 4096]).unwrap();
            let fd = file.as_fd().try_clone_to_owned().unwrap();
            assert!(map(fd, 1 << 20).is_err());
            #[cfg(target_os = "linux")]
            {
                let open = File::from(anonymous().unwrap());
                open.set_len(4096).unwrap();
                assert!(map(open.into(), 4096).is_err());
            }
        }
    }
}

/// Samples of at least this many bytes go through shared memory, unless
/// `MINOT_SHM_DISABLED=1`. `MINOT_QUIC_SHM_THRESHOLD` overrides it; it is not
/// the threshold of the coordinated transport, whose pool has other costs.
///
/// 16 KiB is where shared memory is ahead for every delivery mode in
/// `scripts/shm_threshold.py` (publisher plus subscriber CPU per sample, one
/// host, one subscriber): reliable samples cost 10 % less there and half at
/// 64 KiB, but 7 to 15 % more between 2 and 8 KiB, where the descriptor
/// passing outweighs the copy. Best-effort samples that do not fit a datagram
/// would gain from 2 KiB on; they are held to the same value to keep one rule.
fn threshold() -> usize {
    limit("MINOT_QUIC_SHM_THRESHOLD", 16 << 10)
}
/// The transport's own limit for one sample.
const MAX: usize = 1 << 30;

/// Whether a sample of `len` bytes goes through shared memory when its peer
/// shares memory with this process. Smaller samples stay on QUIC.
pub(crate) fn worth_sharing(len: usize) -> bool {
    len >= threshold() && len <= MAX
}

/// Windows, and phones, where apps are sandboxed from each other: everything
/// stays on the network.
#[cfg(not(all(
    feature = "shm",
    unix,
    not(any(target_os = "ios", target_os = "android"))
)))]
mod none {
    use bytes::Bytes;
    use std::{io, sync::Arc, time::Duration};
    use tokio_util::sync::CancellationToken;

    pub(crate) struct Lazy<'a>(&'a [u8]);
    impl<'a> Lazy<'a> {
        pub(crate) fn new(bytes: &'a [u8]) -> Self {
            Self(bytes)
        }
    }
    #[derive(Debug)]
    pub(crate) struct Link;
    impl Link {
        pub(crate) async fn connect(_: &str, _: u64) -> Option<Arc<Self>> {
            None
        }
        pub(crate) fn lossy(&self, _: u64, _: &mut Lazy) -> io::Result<()> {
            Err(io::ErrorKind::Unsupported.into())
        }
        pub(crate) async fn ordered(&self, _: u64, _: &mut Lazy<'_>) -> io::Result<u64> {
            Err(io::ErrorKind::Unsupported.into())
        }
    }
    #[derive(Debug)]
    pub(crate) struct Hub;
    impl Hub {
        pub(crate) fn open(_: u64) -> Option<Arc<Self>> {
            None
        }
        pub(crate) fn path(&self) -> String {
            String::new()
        }
        pub(crate) async fn run(
            self: Arc<Self>,
            _: CancellationToken,
            _: impl Fn(u64, Bytes) + Clone + Send + Sync + 'static,
        ) {
        }
        pub(crate) async fn take(&self, _: u64, _: u64, _: u64, _: Duration) -> Option<Bytes> {
            None
        }
    }
}
