use std::time::Instant;

pub(crate) struct Pid {
    pub kp: f64, pub ki: f64, pub kd: f64, pub windup_range: f64, pub sign_flip_reset: bool,
    pub small_error: f64, pub small_error_timeout: f64,
    pub large_error: f64, pub large_error_timeout: f64,
    pub slew: f64,

    prev_err: f64, integral: f64, prev_pid_update: Instant, prev_slew_update: Instant,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            kp: 4.0,
            ki: 0.0,
            kd: 20.0,
            windup_range: 4.0,
            sign_flip_reset: true,
            small_error: 1.0,
            small_error_timeout: 1000.0,
            large_error: 5.0,
            large_error_timeout: 4000.0,
            slew: 12.0,
            prev_err: 0.0,
            integral: 0.0,
            prev_pid_update: Instant::now(),
            prev_slew_update: Instant::now(),
        }
    }
}

impl Pid {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(kp: f64, ki: f64, kd: f64, windup_range: f64, sign_flip_reset: bool,
                      small_error: f64, small_error_timeout: f64,
                      large_error: f64, large_error_timeout: f64,
                      slew: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            windup_range,
            sign_flip_reset,
            small_error,
            small_error_timeout,
            large_error,
            large_error_timeout,
            slew,
            prev_err: 0.0,
            integral: 0.0,
            prev_pid_update: Instant::now(),
            prev_slew_update: Instant::now(),
        }
    }

    pub(crate) fn update(&mut self, error: f64) -> f64 {
        let now = Instant::now();
        let dt = self.prev_pid_update.elapsed().as_secs_f64() / 1000.0;
        self.prev_pid_update = now;
        self.integral += error;
        if error.signum() != self.prev_err.signum() && self.sign_flip_reset { self.integral = 0.0; }
        let derivative = (error - self.prev_err) / dt;
        self.prev_err = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub(crate) fn reset(&mut self) {
        self.prev_err = 0.0;
        self.integral = 0.0;
        self.prev_pid_update = Instant::now();
        self.prev_slew_update = Instant::now();
    }

    pub(crate) fn slew(&mut self, target: f64) -> f64 {
        let now = Instant::now();
        let dt = self.prev_slew_update.elapsed().as_secs_f64() / 1000.0;
        self.prev_slew_update = now;

        let mut change = target - self.prev_err;
        if change == 0.0 {
            return target;
        } else if change.abs() / dt > self.slew {
            change = change.signum() * self.slew;
        };
        self.prev_err + change
    }
}

pub(crate) struct ExitCondition {
    pub range: f64,
    pub time: f64,
    start: Option<Instant>,
    done: bool,
}

impl ExitCondition {
    pub(crate) fn new(range: f64, time: f64) -> Self {
        Self {
            range, time,
            start: None,
            done: false,
        }
    }

    pub(crate) fn get_exit(&mut self) -> bool { self.done }

    pub(crate) fn update(&mut self, input: f64) -> bool {
        let now = Instant::now();
        if input.abs() > self.range { self.start = None; }
        else if self.start.is_none() { self.start = Some(now); }
        else if self.start.unwrap().elapsed().as_secs_f64() * 1000.0 > self.time { self.done = true };
        self.done
    }

    pub(crate) fn reset(&mut self) {
        self.start = None;
        self.done = false;
    }
}