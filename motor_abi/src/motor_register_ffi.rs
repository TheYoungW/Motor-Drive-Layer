use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_can_timeout_ms(motor: *mut MotorHandle, timeout_ms: u32) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let reg_value = timeout_ms.saturating_mul(20);
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .write_register_u32(9, reg_value)
            .map_err(|e| e.to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    value: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_f32(rid, value).map_err(|e| e.to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    value: u32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_u32(rid, value).map_err(|e| e.to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_f32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut u32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_u32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
    };
    ffi_rc(rc)
}
