pub(super) struct Pid {
    kp: f64, ki: f64, kd: f64, windup_range: f64,
    small_error: f64, small_error_timeout: f64,
    large_error: f64, large_error_timeout: f64,
    slew: f64
}