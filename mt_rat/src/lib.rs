use anyhow::anyhow;
use log::{error, info};
use rkyv::{
    api::high::{HighSerializer, HighValidator},
    bytecheck::CheckBytes,
    de::Pool,
    rancor::Strategy,
    ser::allocator::ArenaHandle,
    util::AlignedVec,
};
use std::sync::{Arc, LazyLock, Mutex};

use mt_sea::{ship::NetworkShipImpl, *};

pub use mt_sea::VariableType;
pub use mt_sea::net::NetArray;
pub use rkyv::{Archive, Deserialize, Serialize};

pub struct Rat {
    name: String,
    ship: Option<Arc<NetworkShipImpl>>,
}

pub fn rfalse() -> NetArray<u8> {
    nalgebra::DMatrix::<u8>::zeros(1, 1).into()
}

pub fn rtrue() -> NetArray<u8> {
    let rf = rfalse();
    let mut rf = nalgebra::DMatrix::<u8>::from(rf);
    unsafe { *rf.get_unchecked_mut((0, 0)) = 1 };
    rf.into()
}

static RT: LazyLock<Mutex<Option<Arc<tokio::runtime::Runtime>>>> =
    LazyLock::new(|| Mutex::new(None));
static RAT: LazyLock<Mutex<Option<Rat>>> = LazyLock::new(|| Mutex::new(None));

impl Rat {
    async fn create(name: &str, timeout: Option<std::time::Duration>) -> anyhow::Result<Self> {
        let init_future = mt_sea::ship::NetworkShipImpl::init(
            ShipKind::Rat(name.to_string()),
            false,
            mt_sea::Qos::Reliable,
        );

        let ship = match timeout {
            None => Some(init_future.await?),
            Some(t) => match tokio::time::timeout(t, init_future).await {
                Err(_) => None,
                Ok(ship) => Some(ship?),
            },
        };

        Ok(Self {
            name: name.to_string(),
            ship: ship.map(Arc::new),
        })
    }
}

fn store_rat(new_rat: Rat) -> anyhow::Result<()> {
    let mut rat = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;
    if rat.is_some() {
        return Err(anyhow::anyhow!("Rat already initialized"));
    }
    rat.replace(new_rat);
    Ok(())
}

/// Initialize a Rat using the caller's active Tokio runtime.
pub async fn init_async(
    node_name: &str,
    timeout: Option<std::time::Duration>,
) -> anyhow::Result<()> {
    let new_rat = Rat::create(node_name, timeout).await?;
    store_rat(new_rat)
}

pub fn init(
    node_name: &str,
    timeout: Option<std::time::Duration>,
    runtime: Option<Arc<tokio::runtime::Runtime>>,
) -> anyhow::Result<()> {
    if tokio::runtime::Handle::try_current().is_ok() {
        return Err(anyhow!(
            "init() cannot be called from a Tokio runtime; use init_async().await"
        ));
    }

    let mut srt = RT.lock().unwrap();
    if let Some(rt) = runtime {
        srt.replace(rt);
    }

    if srt.is_none() {
        srt.replace(Arc::new(
            tokio::runtime::Builder::new_current_thread()
                .enable_all()
                .build()
                .unwrap(),
        ));
    }

    let rt = srt.as_ref().expect("just set").clone();
    drop(srt);
    let new_rat = rt.block_on(Rat::create(node_name, timeout))?;
    store_rat(new_rat)
}

pub fn deinit() -> anyhow::Result<()> {
    let mut rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    if rat_arc.is_none() {
        return Err(anyhow::anyhow!("Rat not initialized"));
    }

    rat_arc.take();
    Ok(())
}

/// When the code reaches a variable that is watched, call this function to communicate synchronously with the link.
/// It syncs with the other rats and gets the action to be taken for the current var.
/// It then applies the action to the variable and returns.
pub fn bacon<T>(
    variable_name: &str,
    data: &mut T,
    variable_type: VariableType,
) -> anyhow::Result<()>
where
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    T: 'static + Send,
    T: for<'a> Serialize<HighSerializer<AlignedVec, ArenaHandle<'a>, rkyv::rancor::Error>>,
    T: Send + Sync,
{
    if tokio::runtime::Handle::try_current().is_ok() {
        return Err(anyhow!(
            "bacon() cannot be called from a Tokio runtime; use bacon_async().await"
        ));
    }

    let srt = RT.lock().unwrap();
    let rt = srt.as_ref().cloned().ok_or(anyhow!(
        "Async Runtime not initialized. Call init() before calling bacon()."
    ))?;
    drop(srt);

    rt.block_on(bacon_async(variable_name, data, variable_type))
}

/// Synchronize a watched variable using the caller's active Tokio runtime.
pub async fn bacon_async<T>(
    variable_name: &str,
    data: &mut T,
    variable_type: VariableType,
) -> anyhow::Result<()>
where
    T: Archive,
    T::Archived: for<'a> CheckBytes<HighValidator<'a, rkyv::rancor::Error>>
        + Deserialize<T, Strategy<Pool, rkyv::rancor::Error>>,
    T: 'static + Send,
    T: for<'a> Serialize<HighSerializer<AlignedVec, ArenaHandle<'a>, rkyv::rancor::Error>>,
    T: Send + Sync,
{
    let (rat_name, rat_ship) = {
        let rat = RAT
            .lock()
            .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;
        let rat = rat.as_ref().ok_or(anyhow::anyhow!("Rat not initialized"))?;
        (rat.name.clone(), rat.ship.clone())
    };

    if let Some(rat_ship) = rat_ship {
        match rat_ship.ask_for_action(variable_name).await {
            Ok((mt_sea::Action::Sail, lock_until_ack)) => {
                info!("Rat {} sails for variable {}", rat_name, variable_name);
                let receiver = lock_until_ack.then_some({
                    let client = rat_ship.client.lock().await;
                    let sender = client.coordinator_receive.read().unwrap();
                    sender
                        .as_ref()
                        .expect("How are we receiving anything in the client? :)")
                        .subscribe()
                });

                if let Some(mut receiver) = receiver {
                    info!("Locked...");
                    loop {
                        let (packet, _) = receiver.recv().await?;
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            break;
                        }
                    }
                    info!("Unlocked");
                }

                Ok(())
            }
            Ok((mt_sea::Action::Shoot { target, id }, lock_until_ack)) => {
                info!("Rat {} shoots {} at {:?}", rat_name, variable_name, target);

                let receiver = lock_until_ack.then_some({
                    let client = rat_ship.client.lock().await;
                    let sender = client.coordinator_receive.read().unwrap();
                    sender
                        .as_ref()
                        .expect("How are we receiving anything in the client? :)")
                        .subscribe()
                });

                rat_ship
                    .get_cannon()
                    .shoot(&target, id, data, variable_type, variable_name)
                    .await?;

                if let Some(mut receiver) = receiver {
                    info!("Locked...");
                    loop {
                        let (packet, _) = receiver.recv().await?;
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            break;
                        }
                    }
                    info!("Unlocked");
                }

                info!(
                    "Rat {} finished shooting {} at {:?}",
                    rat_name, variable_name, target
                );

                Ok(())
            }
            Ok((mt_sea::Action::Catch { source, id }, lock_until_ack)) => {
                info!(
                    "Rat {} catches {} from {:?}",
                    rat_name, variable_name, source
                );

                let receiver = lock_until_ack.then_some({
                    let client = rat_ship.client.lock().await;
                    let sender = client.coordinator_receive.read().unwrap();
                    sender
                        .as_ref()
                        .expect("How are we receiving anything in the client? :)")
                        .subscribe()
                });

                let mut recv_data = rat_ship.get_cannon().catch::<T>(id).await?;

                info!(
                    "Rat {} finished catching {} from {:?}",
                    rat_name, variable_name, source
                );

                // The first index is the newest
                *data = recv_data.remove(0);

                if let Some(mut receiver) = receiver {
                    info!("Locked...");
                    loop {
                        let (packet, _) = receiver.recv().await?;
                        if matches!(packet.data, net::PacketKind::Acknowledge) {
                            break;
                        }
                    }
                    info!("Unlocked");
                }

                Ok(())
            }
            Err(e) => {
                error!("Failed to get action: {}", e);
                Err(e)
            }
        }
    } else {
        Ok(())
    }
}

// C FFI
#[cfg(all(target_arch = "aarch64", target_vendor = "apple"))]
type CFfiString = i8;

#[cfg(all(
    not(target_arch = "x86"),
    not(target_arch = "x86_64"),
    not(target_vendor = "apple")
))]
type CFfiString = u8;

#[cfg(any(
    target_arch = "x86",
    target_arch = "x86_64",
    all(target_vendor = "apple", not(target_arch = "aarch64"))
))]
type CFfiString = i8;

#[unsafe(no_mangle)]
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_init(node_name: *const CFfiString, timeout_secs: i32) -> i32 {
    let init = || {
        let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
        let node_name = node_name.to_str().unwrap();

        let timeout = if timeout_secs <= 0 {
            None
        } else {
            Some(std::time::Duration::from_secs(timeout_secs as u64))
        };

        init(node_name, timeout, None)
    };

    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(init);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Could not initialize Rat: {e}.");
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }

    #[cfg(not(panic = "unwind"))]
    {
        let d = init();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not initialize Rat: {e}.");
                -1
            }
        }
    }
}

#[unsafe(no_mangle)]
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_deinit() -> i32 {
    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(deinit);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }

    #[cfg(not(panic = "unwind"))]
    {
        let d = deinit();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
        }
    }
}

#[unsafe(no_mangle)]
/// Matrix must be in column-major order.
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_bacon_f32(
    variable_name: *const CFfiString,
    data: *mut f32,
    rows: usize,
    cols: usize,
) -> i32 {
    let f = || {
        let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
        let variable_name = variable_name.to_str().unwrap();

        let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
        let matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

        let mut net_mat = NetArray::from(matrix);
        bacon(variable_name, &mut net_mat, VariableType::F32).map(|_| {
            let matrix: nalgebra::DMatrix<f32> = net_mat.into();
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
        })
    };

    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(f);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Failed to bacon: {}", e);
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }

    #[cfg(not(panic = "unwind"))]
    {
        let d = f();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
        }
    }
}

#[unsafe(no_mangle)]
/// Matrix must be in column-major order.
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_bacon_f64(
    variable_name: *const CFfiString,
    data: *mut f64,
    rows: usize,
    cols: usize,
) -> i32 {
    let f = || {
        let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
        let variable_name = variable_name.to_str().unwrap();

        let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
        let matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

        let mut net_mat = NetArray::from(matrix);
        bacon(variable_name, &mut net_mat, VariableType::F64).map(|_| {
            let matrix: nalgebra::DMatrix<f64> = net_mat.into();
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
        })
    };

    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(f);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Failed to bacon: {}", e);
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }
    #[cfg(not(panic = "unwind"))]
    {
        let d = f();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
        }
    }
}

#[unsafe(no_mangle)]
/// Matrix must be in column-major order.
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_bacon_i32(
    variable_name: *const CFfiString,
    data: *mut i32,
    rows: usize,
    cols: usize,
) -> i32 {
    let f = || {
        let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
        let variable_name = variable_name.to_str().unwrap();

        let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
        let matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

        let mut net_mat = NetArray::from(matrix);
        bacon(variable_name, &mut net_mat, VariableType::I32).map(|_| {
            let matrix: nalgebra::DMatrix<i32> = net_mat.into();
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
        })
    };

    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(f);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Failed to bacon: {}", e);
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }
    #[cfg(not(panic = "unwind"))]
    {
        let d = f();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
        }
    }
}

#[unsafe(no_mangle)]
/// Matrix must be in column-major order.
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_bacon_u8(
    variable_name: *const CFfiString,
    data: *mut u8,
    rows: usize,
    cols: usize,
) -> i32 {
    let f = || {
        let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
        let variable_name = variable_name.to_str().unwrap();

        let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
        let matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

        let mut net_mat = NetArray::from(matrix);
        bacon(variable_name, &mut net_mat, VariableType::U8).map(|_| {
            let matrix: nalgebra::DMatrix<u8> = net_mat.into();
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
        })
    };

    #[cfg(panic = "unwind")]
    {
        let catch = std::panic::catch_unwind(f);

        match catch {
            Ok(Ok(_)) => 0,
            Ok(Err(e)) => {
                error!("Failed to bacon: {}", e);
                -1
            }
            Err(_) => {
                error!("Rust did panic unexpectedly.");
                -2
            }
        }
    }
    #[cfg(not(panic = "unwind"))]
    {
        let d = f();
        match d {
            Ok(_) => 0,
            Err(e) => {
                error!("Could not deinitialize Rat: {e}.");
                -1
            }
        }
    }
}
