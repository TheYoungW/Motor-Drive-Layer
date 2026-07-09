use motor_core::dm_device::DmDeviceType;
use motor_vendor_damiao::{ControlMode as DamiaoControlMode, DamiaoController, DamiaoMotor};
use std::cell::RefCell;
use std::ffi::{c_char, CStr, CString};
use std::ptr;
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

thread_local! {
    static LAST_ERROR: RefCell<CString> = RefCell::new(CString::new("ok").expect("static cstring"));
}

static ABI_CAPABILITIES_JSON: OnceLock<CString> = OnceLock::new();

const ABI_CAPABILITIES: &str = r#"{
  "schema": 1,
  "abi": {
    "name": "motor_abi",
    "version": "__MOTORBRIDGE_VERSION__"
  },
  "transports": ["socketcan", "socketcanfd", "dm-serial", "dm-device"],
  "vendors": ["damiao"],
  "features": {
    "state_cache": true,
    "controller_lifecycle": ["shutdown", "close_bus", "poll_feedback_once", "enable_all", "disable_all"],
    "control_modes": ["mit", "pos-vel", "vel", "force-pos"],
    "damiao": ["dm-serial", "dm-device", "register_u32", "register_f32", "param_u32", "param_f32", "set_can_timeout_ms"]
  }
}"#;

fn abi_capabilities_json() -> &'static CString {
    ABI_CAPABILITIES_JSON.get_or_init(|| {
        CString::new(ABI_CAPABILITIES.replace("__MOTORBRIDGE_VERSION__", env!("CARGO_PKG_VERSION")))
            .expect("capabilities json has no nul bytes")
    })
}

fn set_last_error(msg: impl AsRef<str>) {
    let clean = msg.as_ref().replace('\0', " ");
    let cstr =
        CString::new(clean).unwrap_or_else(|_| CString::new("error").expect("fallback cstring"));
    LAST_ERROR.with(|slot| *slot.borrow_mut() = cstr);
}

fn ok_ptr() -> *const c_char {
    LAST_ERROR.with(|slot| slot.borrow().as_ptr())
}

fn to_damiao_mode(mode: u32) -> Result<DamiaoControlMode, &'static str> {
    match mode {
        1 => Ok(DamiaoControlMode::Mit),
        2 => Ok(DamiaoControlMode::PosVel),
        3 => Ok(DamiaoControlMode::Vel),
        4 => Ok(DamiaoControlMode::ForcePos),
        _ => Err("Damiao mode must be 1(MIT) / 2(POS_VEL) / 3(VEL) / 4(FORCE_POS)"),
    }
}

enum ControllerInner {
    Unbound(String),
    Damiao(DamiaoController),
}

enum MotorHandleInner {
    Damiao(Arc<DamiaoMotor>),
}

#[repr(C)]
pub struct MotorController {
    inner: Mutex<ControllerInner>,
}

#[repr(C)]
pub struct MotorHandle {
    inner: Mutex<MotorHandleInner>,
}

#[repr(C)]
pub struct MotorState {
    pub has_value: i32,
    pub can_id: u8,
    pub arbitration_id: u32,
    pub status_code: u8,
    pub pos: f32,
    pub vel: f32,
    pub torq: f32,
    pub t_mos: f32,
    pub t_rotor: f32,
}

impl Default for MotorState {
    fn default() -> Self {
        Self {
            has_value: 0,
            can_id: 0,
            arbitration_id: 0,
            status_code: 0,
            pos: 0.0,
            vel: 0.0,
            torq: 0.0,
            t_mos: 0.0,
            t_rotor: 0.0,
        }
    }
}

fn ffi_rc(result: Result<(), String>) -> i32 {
    match result {
        Ok(()) => 0,
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

macro_rules! lock_motor_inner {
    ($motor_ptr:expr, $null_message:expr) => {{
        if $motor_ptr.is_null() {
            set_last_error($null_message);
            return -1;
        }
        let motor = unsafe { &*$motor_ptr };
        match motor.inner.lock() {
            Ok(inner) => inner,
            Err(_) => {
                set_last_error("motor handle lock poisoned");
                return -1;
            }
        }
    }};
}

macro_rules! ffi_wrap_motor {
    ($motor_ptr:expr, $body:expr) => {{
        let inner = lock_motor_inner!($motor_ptr, "motor is null");
        ffi_rc($body(&*inner))
    }};
}

fn parse_cstr(ptr: *const c_char, name: &str) -> Result<String, String> {
    if ptr.is_null() {
        return Err(format!("{name} is null"));
    }
    let s = unsafe { CStr::from_ptr(ptr) };
    s.to_str()
        .map(|v| v.to_string())
        .map_err(|_| format!("{name} must be valid UTF-8"))
}

fn controller_vendor_name(inner: &ControllerInner) -> &'static str {
    match inner {
        ControllerInner::Damiao(_) => "Damiao",
        ControllerInner::Unbound(_) => "Unbound",
    }
}

macro_rules! ensure_controller {
    ($fn_name:ident, $variant:ident, $ty:ty, $bind_expr:expr) => {
        fn $fn_name(inner: &mut ControllerInner) -> Result<&mut $ty, String> {
            if let ControllerInner::Unbound(channel) = inner {
                *inner = ControllerInner::$variant($bind_expr(channel).map_err(|e| e.to_string())?);
            }
            match inner {
                ControllerInner::$variant(ctrl) => Ok(ctrl),
                ControllerInner::Unbound(_) => Err("controller binding failed".to_string()),
                current => Err(format!(
                    "controller already bound to {}; use a separate controller",
                    controller_vendor_name(current)
                )),
            }
        }
    };
}

ensure_controller!(
    ensure_damiao_controller,
    Damiao,
    DamiaoController,
    DamiaoController::new_socketcan
);
mod controller_add_motor_ffi;
mod controller_lifecycle_ffi;
mod motor_control_ffi;
mod motor_lifecycle_ffi;
mod motor_register_ffi;
mod param_ffi;
mod state_ffi;
mod vendor_params;
