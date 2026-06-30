use core::f64;

use crate::{chassis::Chassis, math::angle_error, motions::turn_to_heading::AngularDirection, odom::Pose};

#[derive(Debug, Default, Clone, Copy)]
pub struct MoveToPoseParams {
    pub forwards: bool = true,
    pub horizontal_drift: Option<f64> = None,
    pub f_lead: f64 = 0.6,
    // pub g_lead: f64 = 1.0
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 0.0,
    pub local: bool = false,
    pub reset_local_pose: bool = true,
}

impl Chassis {
    pub async fn move_to_pose(&mut self, x: f64, y: f64, theta: f64, timeout: f64, mut params: MoveToPoseParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose).await;
        if motion_start.is_none() { return; }

        self.linear.write().await.reset();
        self.angular.write().await.reset();

        let mut target = Pose::new(x, y, f64::consts::TAU - theta.to_degrees());
        if !params.forwards { target.theta = (target.theta + f64::consts::PI).rem_euclid(f64::consts::TAU); }

        params.horizontal_drift = Some(params.horizontal_drift.unwrap_or(self.drivetrain.read().await.horizontal_drift));

        let mut close = false;
        let mut linear_settled = false;
        let mut prev_same_side = false;
        let mut prev_linear_out: f64 = 0.0;
        let mut last_pose = self.get_pose(params.local, false, false).await;

        loop {
            last_pose = self.update_distance(last_pose, false).await;

            let mut linear = self.linear.write().await;
            let mut angular = self.angular.write().await;
        
            if (close && (linear_settled && angular.small_exit.get_exit() && angular.large_exit.get_exit())) ||
              motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout ||
              !self.motion_running.lock().await.0 {
                drop(linear); drop(angular); break;
            }
            
            // calculate distance to the target point
            let pose = self.get_pose(params.local, true, true).await;
            let dist_target = pose.distance(target);
        
            // check if the robot is close enough to the target to start settling
            if dist_target < 7.5 && !close {
                close = true;
                params.max_speed = prev_linear_out.abs().max(60.0/127.0);
            }
        
            // check if the linear controller has settled
            if linear.large_exit.get_exit() && linear.small_exit.get_exit() { linear_settled = true; }
        
            // calculate the carrot point
            let carrot = if close { target } else { target - Pose::new(target.theta.cos(), target.theta.sin(), 0.0) * params.f_lead * dist_target };
        
            // calculate if the robot is on the same side as the carrot point
            let robot_side = (pose.y - target.y) * -target.theta.sin() <= (pose.x - target.x) * target.theta.cos() + params.early_exit_range;
            let carrot_side = (carrot.y - target.y) * -target.theta.sin() <=
                                    (carrot.x - target.x) * target.theta.cos() + params.early_exit_range;
            let same_side = robot_side == carrot_side;
            // exit if close
            if !same_side && prev_same_side && close && params.min_speed != 0.0 { drop(linear); drop(angular); break; }
            prev_same_side = same_side;
        
            // calculate error
            let adjusted_robot_theta = if params.forwards { pose.theta } else { pose.theta + f64::consts::PI };
            let angular_error = if close { angle_error(adjusted_robot_theta, target.theta, true, AngularDirection::Auto) } else { angle_error(adjusted_robot_theta, pose.angle(carrot), true, AngularDirection::Auto) };
            let mut linear_error = pose.distance(carrot);
            // only use cos when settling
            // otherwise just multiply by the sign of cos
            // maxSlipSpeed takes care of linearOut
            if !close { linear_error *= angle_error(pose.theta, pose.angle(carrot), true, AngularDirection::Auto).cos(); }
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
            angular_out = angular_out.clamp(-params.max_speed, params.max_speed);
        
            // apply restrictions on linear speed
            linear_out = linear_out.clamp(-params.max_speed, params.max_speed);
        
            // constrain linear output by max accel
            if !close { linear_out = linear.slew(linear_out) };
        
            // constrain linear output by the max speed it can travel at without
            // slipping
            let radius = 1.0 / pose.curvature(carrot).abs();
            let max_slip_speed = (params.horizontal_drift.unwrap() * radius * 9.8).sqrt();
            linear_out = linear_out.clamp(-max_slip_speed, max_slip_speed);
            // prioritize angular movement over linear movement
            let overturn = angular_out.abs() + linear_out.abs() - params.max_speed;
            if overturn > 0.0 { linear_out -= if linear_out > 0.0 { overturn } else { -overturn }; }
        
            // prevent moving in the wrong direction
            if params.forwards && !close { linear_out = linear_out.max(0.0); }
            else if !close { linear_out = linear_out.min(0.0); }
        
            // constrain linear output by the minimum speed
            if params.forwards && linear_out < params.min_speed.abs() && linear_out > 0.0 { linear_out = params.min_speed.abs() };
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