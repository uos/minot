use anyhow::anyhow;
use log::{error, info};
use std::sync::{Arc, LazyLock, Mutex};

use sea::{ship::NetworkShipImpl, *};

pub struct Rat {
    name: String,
    ship: NetworkShipImpl,
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
    fn create(name: &str, rt: Arc<tokio::runtime::Runtime>) -> anyhow::Result<Self> {
        let ship = rt.block_on(async {
            sea::ship::NetworkShipImpl::init(ShipKind::Rat(name.to_string()), None).await
        })?;

        info!("Rat {} initialized with ship", name);

        Ok(Self {
            name: name.to_string(),
            ship,
        })
    }

    async fn ask_for_action(&self, variable_name: &str) -> anyhow::Result<Action> {
        self.ship
            .ask_for_action(ShipKind::Rat(self.name.clone()), variable_name)
            .await
    }
}

pub fn init(node_name: &str, runtime: Option<Arc<tokio::runtime::Runtime>>) -> anyhow::Result<()> {
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
    let new_rat = Rat::create(node_name, rt)?;
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
pub fn bacon<T>(variable_name: &str, data: &mut T) -> anyhow::Result<()>
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

    rt.block_on(async move {
        match rat.ask_for_action(variable_name).await {
            Ok(sea::Action::Sail) => {
                info!("Rat {} sails for variable {}", rat.name, variable_name);

                Ok(())
            }
            Ok(sea::Action::Shoot { target }) => {
                info!("Rat {} shoots {} at {:?}", rat.name, variable_name, target);
                rat.ship.get_cannon().shoot(&target, data.clone()).await?;

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

                let recv_data = rat.ship.get_cannon().catch::<T>(&source).await?;

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
}

// C FFI

#[cfg(target_arch = "x86_64")]
#[no_mangle]
pub extern "C" fn rat_init(node_name: *const i8) -> i32 {
    let node_name = unsafe { std::ffi::CStr::from_ptr(node_name) };
    let node_name = node_name.to_str().unwrap();

    match init(node_name, None) {
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

    match init(node_name, None) {
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

#[cfg(target_arch = "x86_64")]
#[no_mangle]
/// Matrix must be in column-major order.
pub extern "C" fn rat_bacon_u8(
    variable_name: *const i8,
    data: *mut u8,
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
pub extern "C" fn rat_bacon_u8(
    variable_name: *const u8,
    data: *mut u8,
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
