use std::time::Duration;

use async_trait::async_trait;
use vexide::smart::{SmartDevice, motor::{BrakeMode, MotorControl}};

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::{swing_to_heading::DriveSide::LEFT, turn_to_heading::AngularDirection::{self, Auto}}};

#[derive(Default, Debug, PartialEq, Eq)]
pub(crate) enum DriveSide {
    #[default]
    LEFT,
    RIGHT
}

#[derive(Default, Debug)]
pub(crate) struct SwingToHeadingParams {
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_range: f64 = 0.0
}

#[derive(Default, Debug)]
pub(crate) struct SwingToHeading {
    pub theta: f64,
    pub locked_side: DriveSide,
    pub params: SwingToHeadingParams,
    target: f64, delta: f64,
    motor_power: f64, prev_power: f64,
    start: f64, settling: bool,
    prev_raw_delta: Option<f64>, prev_delta: Option<f64>,
    brake_mode: BrakeMode = BrakeMode::Brake
}

#[async_trait(?Send)]
impl Motion for SwingToHeading {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.angular.write().await.reset();
        let mut dt = chassis.drivetrain.write().await;
        let target = if self.locked_side == DriveSide::LEFT { dt.left_motors[0].target() }
            else { dt.right_motors[0].target() };
        self.brake_mode = match target {
            MotorControl::Brake(brake) => brake,
            _ => BrakeMode::Coast,
        };
        if self.locked_side == DriveSide::LEFT { dt.left_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); }); }
            else { dt.right_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); } ); };
        drop(dt);
    }

    async fn tick(&mut self, chassis: &mut Chassis) -> bool {
        let mut angular = chassis.angular.write().await;
        if angular.large_exit.get_exit() && angular.small_exit.get_exit() {
            drop(angular);
            return false;
        }

        let mut pose = chassis.get_global_pose(false, false).await;
        pose.theta = pose.theta.rem_euclid(360.0);
        
        let raw_delta = angle_error(self.target, pose.theta, false, Auto);
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
        if motor_power < -self.params.max_speed { motor_power = -self.params.max_speed; }
        if delta.abs() > 20.0 { motor_power = angular.slew(motor_power); }
        if motor_power < 0.0 && motor_power > -self.params.min_speed { motor_power = -self.params.min_speed; }
        else if motor_power > 0.0 && motor_power < self.params.min_speed { motor_power = self.params.min_speed; }
        self.prev_power = motor_power;

        drop(angular);
        let mut dt = chassis.drivetrain.write().await;
        if self.locked_side == LEFT {
            dt.left_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); });
            dt.right_motors.iter_mut().for_each(|m| { m.set_voltage(-motor_power * m.max_voltage()).ok(); });
        } else {
            dt.left_motors.iter_mut().for_each(|m| { m.set_voltage(motor_power * m.max_voltage()).ok(); });
            dt.right_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); });
        }

        drop(dt);
        return true;
    }

    async fn cleanup(&mut self, chassis: &mut Chassis) {
        chassis.set_brake_mode(self.brake_mode).await;
    }
}

impl Chassis {
    pub(crate) async fn swing_to_heading(&mut self, theta: f64, locked_side: DriveSide, timeout: u64, params: SwingToHeadingParams) {
        self.run_motion(Box::new(SwingToHeading {
            theta, locked_side, params, 
            ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}
