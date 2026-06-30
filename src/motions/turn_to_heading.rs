use vexide::smart::motor::{BrakeMode, MotorControl};

use crate::{chassis::Chassis, math::angle_error, motions::turn_to_heading::AngularDirection::Auto, odom::Pose};

#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum AngularDirection {
    #[default]
    Auto,
    Clockwise,
    CounterClockwise
}

#[derive(Debug, PartialEq, Eq)]
pub enum DriveSide {
    Left,
    Right
}

#[derive(Debug, Default, Clone, Copy)]
pub struct TurnToHeadingParams {
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 1.0,
    pub local: bool = false,
    pub reset_local_pose: bool = true,
}

#[derive(Default, Debug)]
pub(crate) struct TurnToHeading {
    pub params: TurnToHeadingParams,
    pub prev_power: f64, pub prev_raw_delta: Option<f64>, 
    pub prev_delta: Option<f64>, pub settling: bool
}

impl TurnToHeading {
    pub(crate) async fn tick(&mut self, theta: f64, pose: Pose, chassis: &mut Chassis) -> Option<f64> {
        let mut angular = chassis.angular.write().await;

        let raw_delta = angle_error(theta, pose.theta, false, Auto);
        if raw_delta.signum() != self.prev_raw_delta.unwrap_or(raw_delta).signum() { self.settling = true };
        self.prev_raw_delta = Some(raw_delta);

        let delta = if self.settling { raw_delta }
            else { angle_error(theta, pose.theta, false, self.params.direction) };

        if self.params.min_speed != 0.0 && (delta.abs() < self.params.early_exit_range 
            || delta.signum() != self.prev_delta.unwrap_or(delta).signum()) {
            drop(angular);
            return None;
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
        Some(motor_power)
    }
}

impl Chassis {
    pub async fn turn_to_heading(&mut self, theta: f64, timeout: f64, params: TurnToHeadingParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose).await;
        if motion_start.is_none() { return; }

        self.angular.write().await.reset();
        let mut pose = self.get_pose(params.local, false, false).await;
        let mut turn_state = TurnToHeading { params, ..Default::default() };

        loop {
            pose = self.update_distance(pose, params.local).await;

            let mut angular = self.angular.write().await;
            let angular_settled = angular.small_exit.get_exit() && angular.large_exit.get_exit();
            drop(angular);
            if angular_settled || motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout
              || !self.motion_running.lock().await.0 {
                break;
            }

            let motor_power = turn_state.tick(theta, pose, self).await;
            if motor_power.is_none() { break; }
            let motor_power = motor_power.unwrap();
            self.tank(motor_power, -motor_power, true).await;
        }

        self.end_motion(motion_start).await;
    }

    pub async fn swing_to_heading(&mut self, theta: f64, locked_side: DriveSide, timeout: f64, params: TurnToHeadingParams) {
        let mut self_clone = self.clone();
        let motion_start = self_clone.start_motion(params.reset_local_pose).await;
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
        let mut turn_state = TurnToHeading { params, ..Default::default() };

        loop {
            pose = self.update_distance(pose, params.local).await;

            let mut angular = self.angular.write().await;
            let angular_settled = angular.small_exit.get_exit() && angular.large_exit.get_exit();
            drop(angular);
            if angular_settled || motion_start.as_ref().unwrap().elapsed().as_secs_f64() * 1000.0 >= timeout
              || !self.motion_running.lock().await.0 {
                break;
            }

            pose.theta = pose.theta.rem_euclid(360.0);
            let motor_power = turn_state.tick(theta, pose, self).await;
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