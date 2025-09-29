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

use sea::{ship::NetworkShipImpl, *};

pub use rkyv::{Archive, Deserialize, Serialize};
pub use sea::VariableType;
pub use sea::net::NetArray;

pub struct Rat {
    name: String,
    ship: Option<NetworkShipImpl>,
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
    fn create(
        name: &str,
        timeout: Option<std::time::Duration>,
        rt: Arc<tokio::runtime::Runtime>,
    ) -> anyhow::Result<Self> {
        let ship = rt.block_on(async {
            let init_future =
                sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_string()), None, false);

            match timeout {
                None => Ok(Some(init_future.await?)),
                Some(t) => match tokio::time::timeout(t, init_future).await {
                    Err(_) => Ok::<Option<NetworkShipImpl>, anyhow::Error>(None),
                    Ok(t) => Ok(Some(t?)),
                },
            }
        })?;

        Ok(Self {
            name: name.to_string(),
            ship,
        })
    }
}

pub fn init(
    node_name: &str,
    timeout: Option<std::time::Duration>,
    runtime: Option<Arc<tokio::runtime::Runtime>>,
) -> anyhow::Result<()> {
    let mut rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    if rat_arc.is_some() {
        return Err(anyhow::anyhow!("Rat already initialized"));
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
    let new_rat = Rat::create(node_name, timeout, rt)?;
    rat_arc.replace(new_rat);

    Ok(())
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
    let rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    let rat = rat_arc
        .as_ref()
        .ok_or(anyhow::anyhow!("Rat not initialized"))?;

    let srt = RT.lock().unwrap();
    let rt = srt.as_ref().ok_or(anyhow!(
        "Async Runtime not initialized. Call init() before calling bacon()."
    ))?;

    if let Some(rat_ship) = rat.ship.as_ref() {
        rt.block_on(async move {
            match rat_ship.ask_for_action(variable_name).await {
                Ok((sea::Action::Sail, lock_until_ack)) => {
                    info!("Rat {} sails for variable {}", rat.name, variable_name);
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
                Ok((sea::Action::Shoot { target, id }, lock_until_ack)) => {
                    info!("Rat {} shoots {} at {:?}", rat.name, variable_name, target);

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
                        rat.name, variable_name, target
                    );

                    Ok(())
                }
                Ok((sea::Action::Catch { source, id }, lock_until_ack)) => {
                    info!(
                        "Rat {} catches {} from {:?}",
                        rat.name, variable_name, source
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
                        rat.name, variable_name, source
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
        })
    } else {
        Ok(())
    }
}

// C FFI
#[cfg(build_c_interop)]
#[cfg(all(target_arch = "aarch64", target_vendor = "apple"))]
type CFfiString = i8;

#[cfg(build_c_interop)]
#[cfg(all(
    not(target_arch = "x86"),
    not(target_arch = "x86_64"),
    not(target_vendor = "apple")
))]
type CFfiString = u8;

#[cfg(build_c_interop)]
#[cfg(any(
    target_arch = "x86",
    target_arch = "x86_64",
    all(target_vendor = "apple", not(target_arch = "aarch64"))
))]
type CFfiString = i8;

#[cfg(build_c_interop)]
#[unsafe(no_mangle)]
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_init(node_name: *const CFfiString, timeout_secs: i32) -> i32 {
    let catch = std::panic::catch_unwind(|| {
        let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
        let node_name = node_name.to_str().unwrap();

        let timeout = if timeout_secs <= 0 {
            None
        } else {
            Some(std::time::Duration::from_secs(timeout_secs as u64))
        };

        init(node_name, timeout, None)
    });

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

#[cfg(build_c_interop)]
#[unsafe(no_mangle)]
/// # Safety
/// C interop
pub unsafe extern "C" fn rat_deinit() -> i32 {
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

#[cfg(build_c_interop)]
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
    let catch = std::panic::catch_unwind(|| {
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
    });

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

#[cfg(build_c_interop)]
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
    let catch = std::panic::catch_unwind(|| {
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
    });

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

#[cfg(build_c_interop)]
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
    let catch = std::panic::catch_unwind(|| {
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
    });

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

#[cfg(build_c_interop)]
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
    let catch = std::panic::catch_unwind(|| {
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
    });

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
