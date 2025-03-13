use anyhow::anyhow;
use log::{error, info};
use std::sync::{Arc, LazyLock, Mutex};

use sea::{ship::NetworkShipImpl, *};

pub struct Rat {
    name: String,
    ship: Option<NetworkShipImpl>,
}

pub fn rfalse() -> nalgebra::DMatrix<u8> {
    nalgebra::DMatrix::<u8>::zeros(1, 1)
}

pub fn rtrue() -> nalgebra::DMatrix<u8> {
    let mut rf = rfalse();
    unsafe { *rf.get_unchecked_mut((0, 0)) = 1 };
    rf
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
                sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_string()), None);

            match timeout {
                None => {
                    return Ok(Some(init_future.await?));
                }
                Some(t) => match tokio::time::timeout(t, init_future).await {
                    Err(_) => {
                        return Ok::<Option<NetworkShipImpl>, anyhow::Error>(None);
                    }
                    Ok(t) => {
                        return Ok(Some(t?));
                    }
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

/// When the code reaches a variable that is watched, call this function to communitcate synchronously with the link.
/// It syncs with the other rats and gets the action to be taken for the current var.
/// It then applies the action to the variable and returns.
pub fn bacon<T>(
    variable_name: &str,
    data: &mut T,
    variable_type: VariableType,
) -> anyhow::Result<()>
where
    T: sea::net::SeaSendableBuffer,
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
                Ok(sea::Action::Sail) => {
                    info!("Rat {} sails for variable {}", rat.name, variable_name);

                    Ok(())
                }
                Ok(sea::Action::Shoot { target }) => {
                    info!("Rat {} shoots {} at {:?}", rat.name, variable_name, target);
                    rat_ship
                        .get_cannon()
                        .shoot(&target, data.clone(), variable_type)
                        .await?;

                    info!(
                        "Rat {} finished shooting {} at {:?}",
                        rat.name, variable_name, target
                    );

                    Ok(())
                }
                Ok(sea::Action::Catch { source }) => {
                    info!(
                        "Rat {} catches {} from {:?}",
                        rat.name, variable_name, source
                    );

                    let recv_data = rat_ship.get_cannon().catch::<T>(&source).await?;

                    info!(
                        "Rat {} finished catching {} from {:?}",
                        rat.name, variable_name, source
                    );

                    // let deserialized: nalgebra::DMatrix<T> = bincode::deserialize(&recv_data)
                    // .map_err(|e| anyhow::anyhow!("Failed to deserialize data: {}", e))?;
                    *data = recv_data;

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

#[no_mangle]
pub extern "C" fn rat_init(node_name: *const CFfiString, timeout_secs: i32) -> i32 {
    let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
    let node_name = node_name.to_str().unwrap();

    let timeout = if timeout_secs <= 0 {
        None
    } else {
        Some(std::time::Duration::from_secs(timeout_secs as u64))
    };

    match init(node_name, timeout, None) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to init: {}", e);
            -1
        }
    }
}

#[no_mangle]
pub extern "C" fn rat_deinit() -> i32 {
    match deinit() {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to deinit: {}", e);
            -1
        }
    }
}

#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f32(
    variable_name: *const CFfiString,
    data: *mut f32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix, VariableType::F32) {
        Ok(_) => {
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
            0
        }
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f64(
    variable_name: *const CFfiString,
    data: *mut f64,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix, VariableType::F64) {
        Ok(_) => {
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
            0
        }
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_i32(
    variable_name: *const CFfiString,
    data: *mut i32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix, VariableType::I32) {
        Ok(_) => {
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
            0
        }
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_u8(
    variable_name: *const CFfiString,
    data: *mut u8,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix, VariableType::U8) {
        Ok(_) => {
            for c in 0..cols {
                for r in 0..rows {
                    data[c * rows + r] = matrix[(r, c)];
                }
            }
            0
        }
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}
