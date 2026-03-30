use core::f64;
use std::{ops::{Add, Div, Mul, Sub}, sync::Arc, time::{Duration, Instant}};

use vexide::{math::Angle, sync::RwLock, task::Task, time::sleep};

use crate::{chassis::{Chassis, Drivetrain, Sensors}, math::ema, tracking_wheel::Encoder};

#[derive(Debug, Default, Clone, Copy, PartialEq, PartialOrd)]
pub(crate) struct Pose {
    pub x: f64,
    pub y: f64,
    pub theta: f64,
}

impl Add for Pose {
    type Output = Pose;

    fn add(self, rhs: Self) -> Self::Output {
        Self::Output {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            theta: self.theta
        }
    }
}

impl Sub for Pose {
    type Output = Pose;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::Output {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            theta: self.theta
        }
    }
}

impl Mul for Pose {
    type Output = f64;

    fn mul(self, rhs: Pose) -> Self::Output {
        self.x * rhs.x + self.y * rhs.y
    }
}

impl Mul<f64> for Pose {
    type Output = Pose;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::Output {
            x: self.x * rhs,
            y: self.y * rhs,
            theta: self.theta
        }
    }
}

impl Div<f64> for Pose {
    type Output = Pose;

    fn div(self, rhs: f64) -> Self::Output {
        Self::Output {
            x: self.x / rhs,
            y: self.y / rhs,
            theta: self.theta
        }
    }
}

impl Pose {
    pub(crate) fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    pub(crate) fn lerp(&self, other: Pose, t: f64) -> Pose {
        Pose {
            x: t * (other.x - self.x) + self.x,
            y: t * (other.y - self.y) + self.y,
            theta: self.theta
        }
    }

    pub(crate) fn distance(&self, other: Pose) -> f64 {
        (other.x - self.x).hypot(other.y - self.y)
    }

    pub(crate) fn angle(&self, other: Pose) -> f64 {
        (other.y - self.y).atan2(other.x - self.x)
    }

    pub(crate) fn rotate(&self, theta: f64) -> Pose {
        Pose {
            x: self.x * theta.cos() - self.y * theta.sin(),
            y: self.x * theta.sin() + self.y * theta.cos(),
            theta: self.theta
        }
    }

    pub(crate) fn curvature(&self, other: Pose) -> f64 {
        let side = crate::math::sign(self.theta.sin() * (other.x - self.x) - self.theta.cos() * (other.y - self.y));
        let a = self.theta.tan();
        let c = a * self.x - self.y;
        let x = (-a * other.x + other.y + c) / ((a * a) + 1.0).sqrt();
        let d = self.distance(other);
        side * 2.0 * x / (d * d)
    }
}

pub(crate) struct OdomLoop {
    pub(crate) drivetrain: Arc<RwLock<Drivetrain>>,
    pub(crate) sensors: Arc<RwLock<Sensors>>,
    pub(crate) pose: Pose,
    pub(crate) local_pose: Pose,
    pub(crate) speed: Pose,
    pub(crate) local_speed: Pose,
    prev_left_drive: f64,
    prev_right_drive: f64,
    prev_vertical1: f64,
    prev_vertical2: f64,
    prev_horizontal1: f64,
    prev_horizontal2: f64,
    prev_imu: f64,
    prev_update: Instant,
}

impl OdomLoop {
    pub(crate) fn new(drivetrain: Arc<RwLock<Drivetrain>>, sensors: Arc<RwLock<Sensors>>) -> Self {
        Self {
            drivetrain,
            sensors,
            pose: Pose::new(0.0, 0.0, 0.0),
            local_pose: Pose::new(0.0, 0.0, 0.0),
            speed: Pose::new(0.0, 0.0, 0.0),
            local_speed: Pose::new(0.0, 0.0, 0.0),
            prev_left_drive: 0.0,
            prev_right_drive: 0.0,
            prev_vertical1: 0.0,
            prev_vertical2: 0.0,
            prev_horizontal1: 0.0,
            prev_horizontal2: 0.0,
            prev_imu: 0.0,
            prev_update: Instant::now(),
        }
    }

    pub(crate) fn get_pose(&self, radians: bool) -> Pose {
        if radians {
            self.pose
        } else {
            Pose::new(self.pose.x, self.pose.y, self.pose.theta.to_degrees())
        }
    }

    pub(crate) fn set_pose(&mut self, pose: Pose, radians: bool) {
        if radians {
            self.pose = pose;
        } else {
            self.pose = Pose::new(pose.x, pose.y, pose.theta.to_radians());
        }
    }

    pub(crate) fn get_local_pose(&self, radians: bool) -> Pose {
        if radians {
            self.local_pose
        } else {
            Pose::new(self.local_pose.x, self.local_pose.y, self.local_pose.theta.to_degrees())
        }
    }

    pub(crate) fn set_local_pose(&mut self, pose: Pose, radians: bool) {
        if radians {
            self.local_pose = pose;
        } else {
            self.local_pose = Pose::new(pose.x, pose.y, pose.theta.to_radians());
        }
    }

    pub(crate) fn get_speed(&self, radians: bool) -> Pose {
        if radians {
            self.speed
        } else {
            Pose::new(self.speed.x, self.speed.y, self.speed.theta.to_degrees())
        }
    }

    pub(crate) fn get_local_speed(&self, radians: bool) -> Pose {
        if radians {
            self.local_speed
        } else {
            Pose::new(self.local_speed.x, self.local_speed.y, self.local_speed.theta.to_degrees())
        }
    }

    pub(crate) fn estimate_pose(&self, time: f64, radians: bool) -> Pose {
        let current_pose = self.pose;
        let current_speed = self.local_speed;
        let delta_local = current_speed * time;

        let avg_heading = current_pose.theta + delta_local.theta * 0.5;
        let future_pose = current_pose + Pose {
            x: delta_local.x * -avg_heading.cos() + delta_local.y * avg_heading.sin(),
            y: delta_local.x * avg_heading.sin() + delta_local.y * avg_heading.cos(),
            theta: delta_local.theta
        };
        if radians { future_pose }
        else { Pose::new(future_pose.x, future_pose.y, future_pose.theta.to_degrees()) }
    }

    pub(crate) async fn update(&mut self) {
        // Update all sensors
        let sensors = self.sensors.read().await;
        let ld = self.drivetrain.read().await.left_motors.position().unwrap_or(Angle::from_radians(self.prev_left_drive)).as_radians();
        let rd = self.drivetrain.read().await.right_motors.position().unwrap_or(Angle::from_radians(self.prev_right_drive)).as_radians();
        let v1 = if let Some(v1) = &sensors.vertical_1 { v1.get_distance_traveled().unwrap_or(self.prev_vertical1) } else { 0.0 };
        let v2 = if let Some(v2) = &sensors.vertical_2 { v2.get_distance_traveled().unwrap_or(self.prev_vertical2) } else { 0.0 };
        let h1 = if let Some(h1) = &sensors.horizontal_1 { h1.get_distance_traveled().unwrap_or(self.prev_horizontal1) } else { 0.0 };
        let h2 = if let Some(h2) = &sensors.horizontal_2 { h2.get_distance_traveled().unwrap_or(self.prev_horizontal2) } else { 0.0 };
        let imu = if let Some(imu) = &sensors.imu { if let Ok(heading) = imu.rotation() { heading.as_radians().mul(sensors.imu_scaler.unwrap_or(1.0).rem_euclid(f64::consts::TAU)) } else { self.prev_imu } } else { 0.0 };

        // Calculate Deltas
        let delta_ld = ld - self.prev_left_drive;
        let delta_rd = rd - self.prev_right_drive;
        let delta_v1 = v1 - self.prev_vertical1;
        let delta_v2 = v2 - self.prev_vertical2;
        let delta_h1 = h1 - self.prev_horizontal1;
        let delta_h2 = h2 - self.prev_horizontal2;
        let delta_imu = imu - self.prev_imu;

        // Update prev values
        self.prev_left_drive = ld;
        self.prev_right_drive = rd;
        self.prev_vertical1 = v1;
        self.prev_vertical2 = v2;
        self.prev_horizontal1 = h1;
        self.prev_horizontal2 = h2;
        self.prev_imu = imu;

        // Calculate the heading
        // Priority:
        // - Dual Horizontals
        // - Dual Verticals
        // - IMU
        // - Drivetrain Motors
        let mut heading = self.pose.x;
        // Try with the two horizontal sensors first
        if sensors.horizontal_1.is_some() && sensors.horizontal_2.is_some() {
            heading -= (delta_h1 - delta_h2) / (sensors.horizontal_1.as_ref().unwrap().get_offset() - sensors.horizontal_2.as_ref().unwrap().get_offset());
        }
        // Next try with two vertical sensors
        else if sensors.vertical_1.is_some() && sensors.vertical_2.is_some() {
            heading -= (delta_v1 - delta_v2) / (sensors.vertical_1.as_ref().unwrap().get_offset() - sensors.vertical_2.as_ref().unwrap().get_offset());
        }
        // Now try the IMU
        else if sensors.imu.is_some() {
            heading += delta_imu;
        }
        // Fall back to the drive if there aren't any options left
        else  {
            heading += (delta_ld - delta_rd) / self.drivetrain.read().await.track_width;
        }
        let delta_heading = heading - self.pose.theta;
        let avg_heading = self.pose.theta + delta_heading / 2.0;
        let avg_local_heading = self.local_pose.theta + delta_heading / 2.0;

        // Calculate change in X and Y
        let (mut delta_h, mut delta_v) = (0.0, 0.0);
        let (mut h_offset, mut v_offset) = (0.0, 0.0);
        if sensors.horizontal_1.is_some() {
            delta_h = delta_h1;
            h_offset = sensors.horizontal_1.as_ref().unwrap().get_offset();
            if sensors.horizontal_2.is_some() {
                h_offset = (h_offset + sensors.horizontal_2.as_ref().unwrap().get_offset()) / 2.0;
                delta_h = (delta_h + delta_h2) / 2.0;
            }
        }
        if sensors.vertical_1.is_some() {
            delta_v = delta_v1;
            v_offset = sensors.vertical_1.as_ref().unwrap().get_offset();
            if sensors.vertical_2.is_some() {
                v_offset = (v_offset + sensors.vertical_2.as_ref().unwrap().get_offset()) / 2.0;
                delta_v = (delta_v + delta_v2) / 2.0;
            }
        } else {
            delta_v = (delta_ld + delta_rd) / 2.0;
        }

        // Calculate Local X and Y deltas
        let (delta_lx, delta_ly) = if delta_heading == 0.0 {
            (delta_h, delta_v)
        } else {
            ( 2.0 * delta_heading.mul(0.5).sin() * delta_h / (delta_heading + h_offset),
              2.0 * delta_heading.mul(0.5).sin() * delta_v / (delta_heading + v_offset) )
        };

        // Update pose and local (last reset) pose
        let prev_pose = self.pose;
        self.pose = self.pose + Pose {
            x: delta_lx * -avg_heading.cos() + delta_ly * avg_heading.sin(),
            y: delta_lx * avg_heading.sin() + delta_ly * avg_heading.cos(),
            theta: heading
        };
        self.local_pose = self.local_pose + Pose {
            x: delta_lx * -avg_local_heading.cos() + delta_ly * avg_local_heading.sin(),
            y: delta_lx * avg_local_heading.sin() + delta_ly * avg_local_heading.cos(),
            theta: self.local_pose.theta + delta_heading
        };
        
        let now = Instant::now();
        let dt = self.prev_update.elapsed().as_secs_f64() * 0.001;
        self.prev_update = now;

        self.speed.x = ema((self.pose.x - prev_pose.x) / dt, self.speed.x, 0.95);
        self.speed.y = ema((self.pose.y - prev_pose.y) / dt, self.speed.y, 0.95);
        self.speed.theta = ema((self.pose.theta - prev_pose.theta) / dt, self.speed.theta, 0.95);

        self.local_speed.x = ema(delta_lx / dt, self.local_speed.x, 0.95);
        self.local_speed.y = ema(delta_ly / dt, self.local_speed.y, 0.95);
        self.local_speed.theta = ema(delta_heading / dt, self.local_speed.theta, 0.95);
    }

    pub(crate) fn odom_loop(odom_task: Arc<RwLock<OdomLoop>>) -> Task<()> {
        vexide::prelude::spawn(async move {
            odom_task.write().await.update().await;
            sleep(Duration::from_millis(10)).await;
        })
    }
}