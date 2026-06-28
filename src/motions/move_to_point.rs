use core::f64;
use std::time::Duration;

use async_trait::async_trait;

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::turn_to_heading::AngularDirection, odom::Pose};

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct MoveToPointParams {
    pub forwards: bool = true,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_range: f64 = 0.0
}

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct MoveToPoint {
    pub x: f64 = 0.0,
    pub y: f64 = 0.0,
    pub params: MoveToPointParams,
    close: bool,
    last_pose: Pose,
    prev_linear_out: f64,
    prev_angular_out: f64,
    prev_side: Option<bool>,
    target: Pose
}

#[async_trait(?Send)]
impl Motion for MoveToPoint {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.linear.write().await.reset();
        chassis.angular.write().await.reset();
        self.close = false;
        self.target = Pose::new(self.x, self.y, 0.0);
        self.target.theta = self.last_pose.angle(self.target);
    }

    async fn tick(&mut self, chassis: &mut Chassis) -> bool {
        let mut linear = chassis.linear.write().await;
        let mut angular = chassis.angular.write().await;
        if self.close && (linear.small_exit.get_exit() 
            && linear.large_exit.get_exit()) { drop(linear); drop(angular); return false; }
        
        // Update position + distance to target
        let pose = chassis.get_global_pose(true, true).await;
        let dist_target = pose.distance(self.target);

        // Check if robot is close enough for settling
        if dist_target <= 7.5 && !self.close {
            self.close = true;
            self.params.max_speed = self.prev_linear_out.abs().max(60.0/127.0);
        }

        let side = (pose.y - self.target.y) * -self.target.theta.sin() <= (pose.x - self.target.x) * -self.target.theta.cos();
        if self.prev_side.is_none() { self.prev_side = Some(side); }
        let same_side = side == self.prev_side.unwrap_or_default();
        // exit if close
        if !same_side && self.params.min_speed != 0.0 { drop(linear); drop(angular); return false; }
        self.prev_side = Some(side);

        // calculate error
        let adjusted_robot_theta = if self.params.forwards { pose.theta } else { pose.theta + f64::consts::PI };
        let angular_error = angle_error(adjusted_robot_theta, pose.angle(self.target), true, AngularDirection::Auto);
        let linear_error = pose.distance(self.target) * angle_error(pose.theta, pose.angle(self.target), true, AngularDirection::Auto).cos();

        // update exit conditions
        linear.small_exit.update(linear_error);
        linear.large_exit.update(linear_error);

        // get output from PIDs
        let mut linear_out = linear.update(linear_error);
        let mut angular_out = if self.close { 0.0 } 
            else { angular.update(angular_error.to_degrees()) };

        // apply restrictions on angular speed
        angular_out = angular_out.clamp(-self.params.max_speed, self.params.max_speed);
        angular_out = angular.slew(angular_out);

        // apply restrictions on linear speed
        linear_out = linear_out.clamp(-self.params.max_speed, self.params.max_speed);
        // constrain linear output by max accel
        // but not for decelerating, since that would interfere with settling
        if !self.close { linear_out = linear.slew(linear_out); }

        // prevent moving in the wrong direction
        if self.params.forwards && !self.close { linear_out = linear_out.max(0.0); }
        else if !self.params.forwards && !self.close { linear_out = linear_out.min(0.0); }

        // constrain linear output by the minimum speed
        if self.params.forwards && linear_out < self.params.min_speed.abs() && linear_out > 0.0 { linear_out = self.params.min_speed.abs(); }
        if !self.params.forwards && -linear_out < self.params.min_speed.abs() && linear_out < 0.0
            { linear_out = -self.params.min_speed.abs(); }

        // update previous output
        self.prev_angular_out = angular_out;
        self.prev_linear_out = linear_out;

        // ratio the speeds to respect the max speed
        let mut left_power = linear_out + angular_out;
        let mut right_power = linear_out - angular_out;
        let ratio = left_power.abs().max(right_power.abs()) / self.params.max_speed;
        if ratio > 1.0 {
            left_power /= ratio;
            right_power /= ratio;
        }

        // move the drivetrain
        drop(linear); drop(angular);
        chassis.tank(left_power, right_power, true).await;

        return true;
    }

    async fn cleanup(&mut self, _chassis: &mut Chassis) {

    }
}

impl Chassis {
    pub async fn move_to_point(&mut self, x: f64, y: f64, timeout: u64, params: MoveToPointParams) {
        self.run_motion(Box::new(MoveToPoint {
            x, y, params,
            ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}