use vexide::prelude::{InertialSensor, Motor};

use crate::{pid::Pid, tracking_wheel::TrackingWheel};

pub(super) struct Drivetrain {
    left_motors: Vec<Motor>,
    right_motors: Vec<Motor>,
    track_width: f64,
    wheel_size: f64,
    wheel_rpm: f64,
    horizontal_drift: f64,
}

pub(super) struct Sensors {
    vertical_1: TrackingWheel,
    vertical_2: TrackingWheel,
    horizontal_1: TrackingWheel,
    horizontal_2: TrackingWheel,
    imu: InertialSensor,
}

pub(super) struct ControllerCurve {

}

pub(super) struct Chassis {
    drivetrain: Drivetrain,
    linear: Pid,
    angular: Pid,
    heading: Pid,
    sensors: Sensors,
    throttle: ControllerCurve,
    steer: ControllerCurve
}