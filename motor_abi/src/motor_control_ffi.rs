use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_ensure_mode(
    motor: *mut MotorHandle,
    mode: u32,
    timeout_ms: u32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                let dm_mode = to_damiao_mode(mode).map_err(|e| e.to_string())?;
                m.ensure_control_mode(dm_mode, Duration::from_millis(timeout_ms as u64))
                    .map_err(|e| e.to_string())
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_mit(
    motor: *mut MotorHandle,
    target_position: f32,
    target_velocity: f32,
    stiffness: f32,
    damping: f32,
    feedforward_torque: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_pos_vel(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_pos_vel(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_vel(motor: *mut MotorHandle, target_velocity: f32) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                m.send_cmd_vel(target_velocity).map_err(|e| e.to_string())
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_force_pos(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
    torque_limit_ratio: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_force_pos(target_position, velocity_limit, torque_limit_ratio)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_store_parameters(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.store_parameters().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_request_feedback(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.request_motor_feedback().map_err(|e| e.to_string()),
        }
    })
}
