use async_trait::async_trait;

use core::f64;
use std::time::Duration;

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::turn_to_heading::AngularDirection::{self, Auto}};

#[derive(Debug, Default)]
pub(crate) struct TurnToPointParams {
    forwards: bool = false,
    direction: AngularDirection = AngularDirection::Auto,
    max_speed: f64 = 1.0,
    min_speed: f64 = 1.0,
    early_exit_range: f64 = 1.0
}

#[derive(Debug, Default)]
pub(crate) struct TurnToPoint {
    pub x: f64, pub y: f64, pub params: TurnToPointParams,
    prev_power: f64, prev_raw_delta: Option<f64>,
    prev_delta: Option<f64>, settling: bool
}

#[async_trait(?Send)]
impl Motion for TurnToPoint {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.angular.write().await.reset();
    }

    async fn tick(&mut self, chassis: &mut Chassis) -> bool {
        let mut angular = chassis.angular.write().await;
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
        chassis.tank(motor_power, -motor_power, true).await;
        return true;
    }

    async fn cleanup(&mut self, _chassis: &mut Chassis) {}
}

impl Chassis {
    pub(crate) async fn turn_to_point(&mut self, x: f64, y: f64, timeout: u64, params: TurnToPointParams) {
        self.run_motion(Box::new(TurnToPoint {
            x, y, params, ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}