use std::{rc::Rc, time::Instant};

use vexide::sync::RwLock;

pub struct Pid {
    pub kp: f64 = 4.0, pub ki: f64 = 0.0, pub kd: f64 = 20.0,
    pub windup_range: f64 = 4.0, pub sign_flip_reset: bool = true,
    pub small_exit: ExitCondition = ExitCondition::new(1.0, 1000.0),
    pub large_exit: ExitCondition = ExitCondition::new(5.0, 4000.0),
    pub slew: f64 = 12.0,

    prev_err: f64 = 0.0, integral: f64 = 0.0, prev_pid_update: Instant, prev_slew_update: Instant,
}

impl Default for Pid {
    fn default() -> Self {
        Self {
            prev_pid_update: Instant::now(),
            prev_slew_update: Instant::now(),
            ..
        }
    }
}

pub struct PidBuilder {
    pub kp: f64 = 4.0, pub ki: f64 = 0.0, pub kd: f64 = 20.0, pub slew: f64 = 12.0,
    pub windup_range: f64 = 4.0, pub sign_flip_reset: bool = false,
    pub small_error: f64 = 1.0, pub small_error_timeout: f64 = 1000.0,
    pub large_error: f64 = 4.0, pub large_error_timeout: f64 = 4000.0,
}

impl From<PidBuilder> for Pid {
    fn from(value: PidBuilder) -> Self {
        Self {
            kp: value.kp, ki: value.kp, kd: value.kd, slew: value.slew,
            windup_range: value.windup_range, sign_flip_reset: value.sign_flip_reset,
            small_exit: ExitCondition::new(value.small_error, value.small_error_timeout),
            large_exit: ExitCondition::new(value.large_error, value.large_error_timeout),
            prev_err: 0.0, integral: 0.0,
            prev_pid_update: Instant::now(), prev_slew_update: Instant::now(),
        }
    }
}

impl From<PidBuilder> for Rc<RwLock<Pid>> {
    fn from(value: PidBuilder) -> Self {
        Rc::new(RwLock::new(value.into()))
    }
}

impl Pid {
    #[allow(clippy::too_many_arguments)]
    pub fn new(kp: f64, ki: f64, kd: f64, windup_range: f64, sign_flip_reset: bool,
                      small_error: f64, small_error_timeout: f64,
                      large_error: f64, large_error_timeout: f64,
                      slew: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            windup_range,
            sign_flip_reset,
            small_exit: ExitCondition::new(small_error, small_error_timeout),
            large_exit: ExitCondition::new(large_error, large_error_timeout),
            slew,
            prev_err: 0.0,
            integral: 0.0,
            prev_pid_update: Instant::now(),
            prev_slew_update: Instant::now(),
        }
    }

    pub fn update(&mut self, error: f64) -> f64 {
        let now = Instant::now();
        let dt = self.prev_pid_update.elapsed().as_secs_f64() / 1000.0;
        self.prev_pid_update = now;
        self.integral += error;
        if (error.signum() != self.prev_err.signum() && self.sign_flip_reset)
            || error.abs() > self.windup_range && self.windup_range != 0.0 { self.integral = 0.0; }
        let derivative = (error - self.prev_err) / dt;
        self.prev_err = error;
        self.kp * error + self.ki * self.integral + self.kd * derivative
    }

    pub fn reset(&mut self) {
        self.prev_err = 0.0;
        self.integral = 0.0;
        self.prev_pid_update = Instant::now();
        self.prev_slew_update = Instant::now();
        self.small_exit.reset();
        self.large_exit.reset();
    }

    pub fn slew(&mut self, target: f64) -> f64 {
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

pub struct ExitCondition {
    pub range: f64,
    pub time: f64,
    start: Option<Instant>,
    done: bool,
}

impl ExitCondition {
    pub const fn new(range: f64, time: f64) -> Self {
        Self {
            range, time,
            start: None,
            done: false,
        }
    }

    pub fn get_exit(&mut self) -> bool { self.done }

    pub fn update(&mut self, input: f64) -> bool {
        let now = Instant::now();
        if input.abs() > self.range { self.start = None; }
        else if self.start.is_none() { self.start = Some(now); }
        else if self.start.unwrap().elapsed().as_secs_f64() * 1000.0 > self.time { self.done = true };
        self.done
    }

    pub fn reset(&mut self) {
        self.start = None;
        self.done = false;
    }
}