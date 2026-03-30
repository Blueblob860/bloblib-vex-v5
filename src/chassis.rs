use std::sync::{Arc};

use vexide::{prelude::{InertialSensor, Motor}, sync::RwLock};

use crate::{odom::OdomLoop, pid::Pid, tracking_wheel::TrackingWheel};

pub(crate) struct Drivetrain {
    pub(crate) left_motors: Vec<Motor>,
    pub(crate) right_motors: Vec<Motor>,
    pub(crate) track_width: f64,
    pub(crate) wheel_size: f64,
    pub(crate) wheel_rpm: f64,
    pub(crate) horizontal_drift: f64,
}

pub(crate) struct Sensors {
    pub(crate) vertical_1: Option<TrackingWheel>,
    pub(crate) vertical_2: Option<TrackingWheel>,
    pub(crate) horizontal_1: Option<TrackingWheel>,
    pub(crate) horizontal_2: Option<TrackingWheel>,
    pub(crate) imu: Option<InertialSensor>,
    pub(crate) imu_scaler: Option<f64>,
}

pub(crate) struct ControllerCurve {
    deadband: f64,
    min_output: f64,
    gain: f64
}

impl ControllerCurve {
    pub(crate) const fn new(deadband: f64, min_output: f64, gain: f64) -> Self {
        Self { deadband, min_output, gain }
    }

    pub(crate) fn curve(&self, input: f64) -> f64 {
        let sius = 1.0 / (self.gain.powf(1.0 - self.deadband - 1.0) * (1.0 - self.deadband));
        let iu = self.gain.powf(input.abs() - self.deadband - 1.0) * (input.abs() - self.deadband) * sius;
        (1.0 - self.min_output) / 1.0 * iu - self.min_output
    }
}

pub(crate) struct Chassis {
    pub(crate) drivetrain: Arc<RwLock<Drivetrain>>,
    pub(crate) linear: Arc<RwLock<Pid>>,
    pub(crate) angular: Arc<RwLock<Pid>>,
    pub(crate) heading: Arc<RwLock<Pid>>,
    pub(crate) sensors: Arc<RwLock<Sensors>>,
    pub(crate) throttle: ControllerCurve,
    pub(crate) steer: ControllerCurve,
    pub(crate) odom: Arc<RwLock<OdomLoop>>,
}