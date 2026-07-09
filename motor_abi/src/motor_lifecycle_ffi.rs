use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_free(motor: *mut MotorHandle) {
    if motor.is_null() {
        return;
    }
    let _ = unsafe { Box::from_raw(motor) };
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_enable(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.enable().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_disable(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.disable().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_clear_error(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.clear_error().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_zero_position(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.set_zero_position().map_err(|e| e.to_string()),
        }
    })
}
