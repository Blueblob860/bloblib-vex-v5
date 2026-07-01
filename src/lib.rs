#![feature(default_field_values)]

pub mod chassis;
pub mod math;
pub mod motions;
pub mod odom;
pub mod opcontrol;
pub mod pid;
pub mod timer;
pub mod tracking_wheel;

pub mod prelude {
    pub use crate::{
        chassis::{
            Chassis, ControllerCurve, Drivetrain, Sensors
        },
        motions::{
            AngularDirection, DriveSide, MoveToPointParams, MoveToPoseParams,
            TurnToHeadingParams, TurnToPointParams
        },
        opcontrol,
        pid::{
            Pid, PidBuilder
        },
        tracking_wheel::{
            TrackingWheel
        }
    };
}