use core::f64;
use std::time::Duration;

use async_trait::async_trait;

use crate::{chassis::Chassis, math::angle_error, motion_handler::Motion, motions::turn_to_heading::AngularDirection, odom::Pose};

#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct MoveToPoseParams {
    pub forwards: bool = true,
    pub horizontal_drift: Option<f64> = None,
    pub f_lead: f64 = 0.6,
    // pub g_lead: f64 = 1.0
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 0.0,
}

#[derive(Default)]
pub(crate) struct MoveToPose {
    x: f64, y: f64, theta: f64, params: MoveToPoseParams,
    target: Pose, close: bool, linear_settled: bool,
    prev_same_side: bool, prev_angular_out: f64 = 0.0,
    prev_linear_out: f64 = 0.0
}

#[async_trait(?Send)]
impl Motion for MoveToPose {
    async fn setup(&mut self, chassis: &mut Chassis) {
        chassis.linear.write().await.reset();
        chassis.angular.write().await.reset();
        self.target = Pose::new(self.x, self.y, f64::consts::TAU - self.theta.to_degrees());
        if !self.params.forwards { self.target.theta = (self.target.theta + f64::consts::PI).rem_euclid(f64::consts::TAU); }
        self.params.horizontal_drift = Some(self.params.horizontal_drift.unwrap_or(chassis.drivetrain.read().await.horizontal_drift));

    }

    async fn tick(&mut self, chassis: &mut Chassis) -> bool {
        let mut linear = chassis.linear.write().await;
        let mut angular = chassis.angular.write().await;

        if self.linear_settled && angular.large_exit.get_exit() && angular.small_exit.get_exit()
            || self.close { drop(linear); drop(angular); return false; }
        
        // calculate distance to the target point
        let pose = chassis.get_global_pose(true, true).await;
        let dist_target = pose.distance(self.target);

        // check if the robot is close enough to the target to start settling
        if dist_target < 7.5 && !self.close {
            self.close = true;
            self.params.max_speed = self.prev_linear_out.abs().max(60.0/127.0);
        }

        // check if the linear controller has settled
        if linear.large_exit.get_exit() && linear.small_exit.get_exit() { self.linear_settled = true; }

        // calculate the carrot point
        let carrot = if self.close { self.target } else { self.target - Pose::new(self.target.theta.cos(), self.target.theta.sin(), 0.0) * self.params.f_lead * dist_target };

        // calculate if the robot is on the same side as the carrot point
        let robot_side = (pose.y - self.target.y) * -self.target.theta.sin() <= (pose.x - self.target.x) * self.target.theta.cos() + self.params.early_exit_range;
        let carrot_side = (carrot.y - self.target.y) * -self.target.theta.sin() <=
                                (carrot.x - self.target.x) * self.target.theta.cos() + self.params.early_exit_range;
        let same_side = robot_side == carrot_side;
        // exit if close
        if !same_side && self.prev_same_side && self.close && self.params.min_speed != 0.0 { drop(linear); drop(angular); return false; }
        self.prev_same_side = same_side;

        // calculate error
        let adjusted_robot_theta = if self.params.forwards { pose.theta } else { pose.theta + f64::consts::PI };
        let angular_error = if self.close { angle_error(adjusted_robot_theta, self.target.theta, true, AngularDirection::Auto) } else { angle_error(adjusted_robot_theta, pose.angle(carrot), true, AngularDirection::Auto) };
        let mut linear_error = pose.distance(carrot);
        // only use cos when settling
        // otherwise just multiply by the sign of cos
        // maxSlipSpeed takes care of linearOut
        if !self.close { linear_error *= angle_error(pose.theta, pose.angle(carrot), true, AngularDirection::Auto).cos(); }
        else { linear_error *= angle_error(pose.theta, pose.angle(carrot), true, AngularDirection::Auto).cos().signum(); }

        // update exit conditions
        linear.small_exit.update(linear_error);
        linear.large_exit.update(linear_error);
        angular.small_exit.update(angular_error.to_degrees());
        angular.large_exit.update(angular_error.to_degrees());

        // get output from PIDs
        let mut linear_out = linear.update(linear_error);
        let mut angular_out = angular.update(angular_error.to_degrees());

        // apply restrictions on angular speed
        angular_out = angular_out.clamp(-self.params.max_speed, self.params.max_speed);

        // apply restrictions on linear speed
        linear_out = linear_out.clamp(-self.params.max_speed, self.params.max_speed);

        // constrain linear output by max accel
        if !self.close { linear_out = linear.slew(linear_out) };

        // constrain linear output by the max speed it can travel at without
        // slipping
        let radius = 1.0 / pose.curvature(carrot).abs();
        let max_slip_speed = (self.params.horizontal_drift.unwrap() * radius * 9.8).sqrt();
        linear_out = linear_out.clamp(-max_slip_speed, max_slip_speed);
        // prioritize angular movement over linear movement
        let overturn = angular_out.abs() + linear_out.abs() - self.params.max_speed;
        if overturn > 0.0 { linear_out -= if linear_out > 0.0 { overturn } else { -overturn }; }

        // prevent moving in the wrong direction
        if self.params.forwards && !self.close { linear_out = linear_out.max(0.0); }
        else if !self.close { linear_out = linear_out.min(0.0); }

        // constrain linear output by the minimum speed
        if self.params.forwards && linear_out < self.params.min_speed.abs() && linear_out > 0.0 { linear_out = self.params.min_speed.abs() };
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
    pub async fn move_to_pose(&mut self, x: f64, y: f64, theta: f64, timeout: u64, params: MoveToPoseParams) {
        self.run_motion(Box::new(MoveToPose {
            x, y, theta, params, ..Default::default()
        }), Duration::from_millis(timeout)).await;
    }
}