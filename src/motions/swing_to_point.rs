use core::f64;
use std::time::Duration;

use async_trait::async_trait;
use vexide::smart::motor::{BrakeMode, MotorControl};

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::{swing_to_heading::DriveSide::{self, LEFT}, turn_to_heading::AngularDirection::{self, Auto}}};

#[derive(Default, Debug)]
pub(crate) struct SwingToPointParams {
    pub forwards: bool = true,
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_range: f64 = 0.0
}

#[derive(Default, Debug)]
pub(crate) struct SwingToPoint {
    pub x: f64, pub y: f64, pub locked_side: DriveSide, pub params: SwingToPointParams,
    target: f64, prev_power: f64, settling: bool,
    prev_raw_delta: Option<f64>, prev_delta: Option<f64>,
    brake: BrakeMode = BrakeMode::Brake
}

#[async_trait(?Send)]
impl Motion for SwingToPoint {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.angular.write().await.reset();
        let mut dt = chassis.drivetrain.write().await;
        let target = if self.locked_side == DriveSide::LEFT { dt.left_motors[0].target() }
            else { dt.right_motors[0].target() };
        self.brake = match target {
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
        pose.theta = if self.params.forwards { pose.theta.rem_euclid(360.0) } else { (pose.theta - 180.0).rem_euclid(360.0) };

        let (delta_x, delta_y) = (self.x - pose.x, self.y - pose.y);
        let target = (f64::consts::TAU - delta_y.atan2(delta_x)).to_degrees().rem_euclid(360.0);
        
        let raw_delta = angle_error(target, pose.theta, false, Auto);
        if raw_delta.signum() != self.prev_raw_delta.unwrap_or(raw_delta).signum() { self.settling = true };
        self.prev_raw_delta = Some(raw_delta);

        let delta = if self.settling { raw_delta }
            else { angle_error(target, pose.theta, false, self.params.direction) };

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
        chassis.set_brake_mode(self.brake).await;
    }
}

impl Chassis {
    pub(crate) async fn swing_to_point(&mut self, x: f64, y: f64, locked_side: DriveSide, timeout: u64, params: SwingToPointParams) {
        self.run_motion(Box::new(SwingToPoint {
            x, y, locked_side, params, 
            ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}