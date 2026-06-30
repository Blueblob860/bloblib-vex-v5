use core::f64;

use crate::{chassis::Chassis, math::angle_error, motions::turn_to_heading::AngularDirection, odom::Pose};

#[derive(Default, Debug, Clone, Copy)]
pub struct MoveToPointParams {
    pub forwards: bool = true,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_range: f64 = 0.0,
    pub local: bool = false,
    pub reset_local_pose: bool = true,
}

impl Chassis {
    pub async fn move_to_point(&mut self, x: f64, y: f64, timeout: f64, mut params: MoveToPointParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose).await;
        if motion_start.is_none() { return; }

        self.linear.write().await.reset();
        self.angular.write().await.reset();
        let mut close = false;
        let mut last_pose = self.get_pose(params.local, false, false).await;
        let mut prev_linear_out: f64 = 0.0;
        let mut prev_side: Option<bool> = None;
        let mut target = Pose::new(x, y, 0.0);
        target.theta = last_pose.angle(target);
        loop {
            last_pose = self.update_distance(last_pose, params.local).await;
            let mut linear = self.linear.write().await;
            let mut angular = self.angular.write().await;

            if (close && (linear.small_exit.get_exit() && linear.large_exit.get_exit())) ||
              motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout ||
              !self.motion_running.lock().await.0 {
                drop(linear); drop(angular);
                break;
            }

            let dist_target = last_pose.distance(target);

            // Check if robot is close enough for settling
            if dist_target <= 7.5 && !close {
                close = true;
                params.max_speed = prev_linear_out.abs().max(60.0/127.0);
            }

            let side = (last_pose.y - target.y) * -target.theta.sin() <= (last_pose.x - target.x) * -target.theta.cos();
            if prev_side.is_none() { prev_side = Some(side); }
            let same_side = side == prev_side.unwrap_or_default();
            // exit if close
            if !same_side && params.min_speed != 0.0 { drop(linear); drop(angular); break; }
            prev_side = Some(side);

            // calculate error
            let adjusted_robot_theta = if params.forwards { last_pose.theta } else { last_pose.theta + f64::consts::PI };
            let angular_error = angle_error(adjusted_robot_theta, last_pose.angle(target), true, AngularDirection::Auto);
            let linear_error = last_pose.distance(target) * angle_error(last_pose.theta, last_pose.angle(target), true, AngularDirection::Auto).cos();

            // update exit conditions
            linear.small_exit.update(linear_error);
            linear.large_exit.update(linear_error);

            // get output from PIDs
            let mut linear_out = linear.update(linear_error);
            let mut angular_out = if close { 0.0 } 
                else { angular.update(angular_error.to_degrees()) };

            // apply restrictions on angular speed
            angular_out = angular_out.clamp(-params.max_speed, params.max_speed);
            angular_out = angular.slew(angular_out);

            // apply restrictions on linear speed
            linear_out = linear_out.clamp(-params.max_speed, params.max_speed);
            // constrain linear output by max accel
            // but not for decelerating, since that would interfere with settling
            if !close { linear_out = linear.slew(linear_out); }

            // prevent moving in the wrong direction
            if params.forwards && !close { linear_out = linear_out.max(0.0); }
            else if !params.forwards && !close { linear_out = linear_out.min(0.0); }

            // constrain linear output by the minimum speed
            if params.forwards && linear_out < params.min_speed.abs() && linear_out > 0.0 { linear_out = params.min_speed.abs(); }
            if !params.forwards && -linear_out < params.min_speed.abs() && linear_out < 0.0
                { linear_out = -params.min_speed.abs(); }

            // update previous output
            prev_linear_out = linear_out;

            // ratio the speeds to respect the max speed
            let mut left_power = linear_out + angular_out;
            let mut right_power = linear_out - angular_out;
            let ratio = left_power.abs().max(right_power.abs()) / params.max_speed;
            if ratio > 1.0 {
                left_power /= ratio;
                right_power /= ratio;
            }

            // move the drivetrain
            drop(linear); drop(angular);
            self.tank(left_power, right_power, true).await;
        }

        self.end_motion(motion_start).await;
    }
}