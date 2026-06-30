use std::time::Duration;

use async_trait::async_trait;
use vexide::smart::motor::{BrakeMode, MotorControl};

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::turn_to_heading::AngularDirection::Auto};

#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub(crate) enum AngularDirection {
    #[default]
    Auto,
    Clockwise,
    CounterClockwise
}

#[derive(Default, Debug, PartialEq, Eq)]
pub(crate) enum DriveSide {
    #[default]
    None,
    Left,
    Right
}

#[derive(Debug, Default)]
pub(crate) struct TurnToHeadingParams {
    pub locked_side: DriveSide,
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 1.0
}

#[derive(Default, Debug)]
pub(crate) struct TurnToHeading {
    pub theta: f64, pub params: TurnToHeadingParams,
    brake_mode: BrakeMode = BrakeMode::Coast, prev_power: f64,
    prev_raw_delta: Option<f64>, prev_delta: Option<f64>, settling: bool
}

#[async_trait(?Send)]
impl Motion for TurnToHeading {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.angular.write().await.reset();
        if self.params.locked_side == DriveSide::None { return; }
        let mut dt = chassis.drivetrain.write().await;
        let target = if self.params.locked_side == DriveSide::Left { dt.left_motors[0].target() }
            else { dt.right_motors[0].target() };
        self.brake_mode = match target {
            MotorControl::Brake(brake) => brake,
            _ => BrakeMode::Coast,
        };
        if self.params.locked_side == DriveSide::Left { dt.left_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); }); }
            else { dt.right_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); } ); };
        drop(dt);
    }

    async fn tick(&mut self, chassis: &mut Chassis) -> bool {
        let mut angular = chassis.angular.write().await;
        let mut pose = chassis.get_global_pose(false, false).await;
        if self.params.locked_side != DriveSide::None { pose.theta = pose.theta.rem_euclid(360.0) };

        let raw_delta = angle_error(self.theta, pose.theta, false, Auto);
        if raw_delta.signum() != self.prev_raw_delta.unwrap_or(raw_delta).signum() { self.settling = true };
        self.prev_raw_delta = Some(raw_delta);

        let delta = if self.settling { raw_delta }
            else { angle_error(self.theta, pose.theta, false, self.params.direction) };

        if self.params.min_speed != 0.0 && (delta.abs() < self.params.early_exit_range 
            || delta.signum() != self.prev_delta.unwrap_or(delta).signum()) {
            drop(angular);
            return false;
        }
        self.prev_delta = Some(delta);

        let mut motor_power = angular.update(delta);
        angular.large_exit.update(delta); angular.small_exit.update(delta);

        if motor_power > self.params.max_speed { motor_power = self.params.max_speed; }
        else if motor_power < -self.params.max_speed { motor_power = -self.params.max_speed; }
        if delta.abs() > 20.0 { motor_power = angular.slew(motor_power); }
        if motor_power < 0.0 && motor_power > -self.params.min_speed { motor_power = -self.params.min_speed; }
        else if motor_power > 0.0 && motor_power < self.params.min_speed { motor_power = self.params.min_speed; }
        self.prev_power = motor_power;

        drop(angular);
        chassis.tank(motor_power, -motor_power, true).await;

        if self.params.locked_side == DriveSide::None {
            return true;
        } else if self.params.locked_side == DriveSide::Left {
            let mut dt = chassis.drivetrain.write().await;
            dt.left_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); });
            drop(dt);
        } else {
            let mut dt = chassis.drivetrain.write().await;
            dt.right_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); });
            drop(dt);
        }

        return true;
    }

    async fn cleanup(&mut self, chassis: &mut Chassis) {
        if self.params.locked_side == DriveSide::None { return; }
        chassis.set_brake_mode(self.brake_mode).await;
    }
}

impl Chassis {
    pub(crate) async fn turn_to_heading(&mut self, theta: f64, timeout: u64, params: TurnToHeadingParams) {
        self.run_motion(Box::new(TurnToHeading {
            theta, params, ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }

    pub(crate) async fn swing_to_heading(&mut self, theta: f64, locked_side: DriveSide, timeout: u64, params: TurnToHeadingParams) {
        self.run_motion(Box::new(TurnToHeading {
            theta, params: TurnToHeadingParams { locked_side, ..params }, 
            ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}