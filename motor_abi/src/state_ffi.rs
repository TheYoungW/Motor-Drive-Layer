use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
) -> i32 {
    if motor.is_null() || out_state.is_null() {
        set_last_error("motor or out_state is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_state };
    match &*motor {
        MotorHandleInner::Damiao(m) => {
            if let Some(state) = m.latest_state() {
                *out = MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                };
            } else {
                *out = MotorState::default();
            }
        }
    }
    0
}
