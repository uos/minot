use log::{error, info};
use std::sync::{LazyLock, Mutex};

use nalgebra::Scalar;
use sea::*;

pub struct Rat {
    name: String,
    ship: Box<dyn Ship>,
}

static RAT: LazyLock<Mutex<Option<Rat>>> = LazyLock::new(|| Mutex::new(None));

impl Rat {
    pub fn create(name: &str) -> anyhow::Result<Self> {
        let mut ship = Box::new(sea::ship::NetworkShipImpl::new());
        let ship_name = tokio::runtime::Runtime::new()
            .unwrap()
            .block_on(async { ship.water(ShipKind::Rat(name.to_string())).await })?;

        info!("Rat {} initialized with ship {}", name, ship_name);

        Ok(Self {
            name: name.to_string(),
            ship,
        })
    }

    pub async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<Action> {
        self.ship
            .ask_for_action(ShipKind::Rat(self.name.clone()), variable_name)
            .await
    }
}

pub fn init(node_name: &str) -> anyhow::Result<()> {
    let mut rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    if rat_arc.is_some() {
        return Err(anyhow::anyhow!("Rat already initialized"));
    }

    let new_rat = Rat::create(node_name)?;

    *rat_arc = Some(new_rat);

    Ok(())
}

pub fn deinit() -> anyhow::Result<()> {
    let mut rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    if rat_arc.is_none() {
        return Err(anyhow::anyhow!("Rat not initialized"));
    }

    *rat_arc = None;
    Ok(())
}

/// When the code reaches a variable that is watched, call this function to communitcate synchronously with the link.
/// It syncs with the other rats and gets the action to be taken for the current var.
/// It then applies the action to the variable and returns.
pub fn bacon<T>(variable_name: &str, data: &mut nalgebra::DMatrix<T>) -> anyhow::Result<()>
where
    T: Scalar + serde::Serialize + for<'a> serde::Deserialize<'a>,
{
    let rat_arc = RAT
        .lock()
        .map_err(|e| anyhow::anyhow!("Failed to lock rat: {}", e))?;

    let rat = rat_arc
        .as_ref()
        .ok_or(anyhow::anyhow!("Rat not initialized"))?;

    let bytes =
        bincode::serialize(data).map_err(|e| anyhow::anyhow!("Failed to serialize data: {}", e))?;

    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();

    runtime.block_on(async move {
        match rat.ask_for_action(variable_name).await {
            Ok(sea::Action::Sail) => {
                info!("Rat {} sails for variable {}", rat.name, variable_name);

                Ok(())
            }
            Ok(sea::Action::Shoot { target }) => {
                info!("Rat {} shoots {} at {}", rat.name, variable_name, target);
                rat.ship.get_cannon().shoot(target, &bytes).await;

                info!(
                    "Rat {} finished shooting {} at {}",
                    rat.name, variable_name, target
                );

                Ok(())
            }
            Ok(sea::Action::Catch { source }) => {
                info!("Rat {} catches {} from {}", rat.name, variable_name, source);

                let recv_data = rat.ship.get_cannon().catch(source).await;

                info!(
                    "Rat {} finished catching {} from {}",
                    rat.name, variable_name, source
                );

                let deserialized: nalgebra::DMatrix<T> = bincode::deserialize(&recv_data)
                    .map_err(|e| anyhow::anyhow!("Failed to deserialize data: {}", e))?;
                *data = deserialized;
                Ok(())
            }
            Err(e) => {
                error!("Failed to get action: {}", e);
                Err(e)
            }
        }
    })
}

// C FFI

#[cfg(target_arch = "x86_64")]
#[no_mangle]
pub extern "C" fn rat_init(node_name: *const i8) -> i32 {
    let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
    let node_name = node_name.to_str().unwrap();

    match init(node_name) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to init: {}", e);
            -1
        }
    }
}

#[cfg(not(target_arch = "x86_64"))]
#[no_mangle]
pub extern "C" fn rat_init(node_name: *const u8) -> i32 {
    let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
    let node_name = node_name.to_str().unwrap();

    match init(node_name) {
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

#[cfg(target_arch = "x86_64")]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f32(
    variable_name: *const i8,
    data: *mut f32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[cfg(not(target_arch = "x86_64"))]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f32(
    variable_name: *const u8,
    data: *mut f32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[cfg(target_arch = "x86_64")]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f64(
    variable_name: *const i8,
    data: *mut f64,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[cfg(not(target_arch = "x86_64"))]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_f64(
    variable_name: *const u8,
    data: *mut f64,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[cfg(target_arch = "x86_64")]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_i32(
    variable_name: *const i8,
    data: *mut i32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}

#[cfg(not(target_arch = "x86_64"))]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_i32(
    variable_name: *const u8,
    data: *mut i32,
    rows: usize,
    cols: usize,
) -> i32 {
    let variable_name = unsafe { std::ffi::CStr::from_ptr(variable_name) };
    let variable_name = variable_name.to_str().unwrap();

    let data = unsafe { std::slice::from_raw_parts_mut(data, rows * cols) };
    let mut matrix = nalgebra::DMatrix::from_column_slice(rows, cols, data);

    match bacon(variable_name, &mut matrix) {
        Ok(_) => 0,
        Err(e) => {
            error!("Failed to bacon: {}", e);
            -1
        }
    }
}
