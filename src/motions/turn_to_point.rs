use core::f64;
use vexide::smart::motor::{BrakeMode, MotorControl};

use crate::{chassis::Chassis, motions::turn_to_heading::{AngularDirection, DriveSide, TurnToHeading, TurnToHeadingParams}};

#[derive(Debug, Default, Clone, Copy)]
pub struct TurnToPointParams {
    pub forwards: bool = false,
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 1.0,
    pub local: bool = false,
    pub reset_local_pose: bool = true,
}

impl From<TurnToPointParams> for TurnToHeadingParams {
    fn from(value: TurnToPointParams) -> Self {
        Self { direction: value.direction, max_speed: value.max_speed, min_speed: value.min_speed, early_exit_range: value.early_exit_range, local: value.local, reset_local_pose: value.reset_local_pose }
    }
}

impl Chassis {
    pub async fn turn_to_point(&mut self, x: f64, y: f64, timeout: f64, params: TurnToPointParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose && params.local).await;
        if motion_start.is_none() { return; }

        self.angular.write().await.reset();
        let mut pose = self.get_pose(params.local, false, false).await;
        let mut turn_state = TurnToHeading { params: params.into(), ..Default::default() };

        loop {
            pose = self.update_distance(pose, params.local).await;

            let mut angular = self.angular.write().await;
            let angular_settled = angular.small_exit.get_exit() && angular.large_exit.get_exit();
            drop(angular);
            if angular_settled || motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout
              || !self.motion_running.lock().await.0 {
                break;
            }

            pose.theta = if params.forwards { pose.theta } else { pose.theta - 180.0 }.rem_euclid(360.0);
            let (delta_x, delta_y) = (x - pose.x, y - pose.y);
            let target = (f64::consts::TAU - delta_y.atan2(delta_x)).to_degrees().rem_euclid(360.0);
            let motor_power = turn_state.tick(target, pose, self).await;
            if motor_power.is_none() { break; }
            let motor_power = motor_power.unwrap();
            self.tank(motor_power, -motor_power, true).await;
        }

        self.end_motion(motion_start).await;
    }

    pub async fn swing_to_point(&mut self, x: f64, y: f64, locked_side: DriveSide, timeout: f64, params: TurnToPointParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose && params.local).await;
        if motion_start.is_none() { return; }

        self.angular.write().await.reset();
        
        let mut dt = self.drivetrain.write().await;
        let motor_target = if locked_side == DriveSide::Left { dt.left_motors[0].target() }
            else { dt.right_motors[0].target() };
        let brake_mode = match motor_target {
            MotorControl::Brake(brake) => brake,
            _ => BrakeMode::Coast,
        };
        if locked_side == DriveSide::Left { dt.left_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); }); }
            else { dt.right_motors.iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); } ); };
        drop(dt);
        
        let mut pose = self.get_pose(params.local, false, false).await;
        let mut turn_state = TurnToHeading { params: params.into() , ..Default::default() };

        loop {
            pose = self.update_distance(pose, params.local).await;

            let mut angular = self.angular.write().await;
            let angular_settled = angular.small_exit.get_exit() && angular.large_exit.get_exit();
            drop(angular);
            if angular_settled || motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout
              || !self.motion_running.lock().await.0 {
                break;
            }

            pose.theta = if params.forwards { pose.theta } else { pose.theta - 180.0 }.rem_euclid(360.0);
            let (delta_x, delta_y) = (x - pose.x, y - pose.y);
            let target = (f64::consts::TAU - delta_y.atan2(delta_x)).to_degrees().rem_euclid(360.0);
            let motor_power = turn_state.tick(target, pose, self).await;
            if motor_power.is_none() { break; }
            let motor_power = motor_power.unwrap();

            self.tank(motor_power, -motor_power, true).await;
            let mut dt = self.drivetrain.write().await;
            (if locked_side == DriveSide::Left { &mut dt.left_motors }
                else { &mut dt.right_motors })
                .iter_mut().for_each(|m| { m.brake(BrakeMode::Hold).ok(); });
            drop(dt);
        }

        self.set_brake_mode(brake_mode).await;
        self.end_motion(motion_start).await;
    }
}