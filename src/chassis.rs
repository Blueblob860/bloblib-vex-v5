use core::f64;
use std::{rc::Rc, time::{Duration, Instant}};

use vexide::{prelude::{Controller, InertialSensor, Motor}, smart::motor::BrakeMode, sync::{Mutex, MutexGuard, RwLock}, task::Task};

use crate::{odom::{OdomLoop, Pose}, pid::Pid, tracking_wheel::{Encoder, TrackingWheel}};

pub struct Drivetrain {
    pub left_motors: Vec<Motor>,
    pub right_motors: Vec<Motor>,
    pub track_width: f64,
    pub wheel_size: f64,
    pub drive_gear_ratio: f64,
    pub horizontal_drift: f64,
}

impl Drivetrain {
    pub fn new(left_motors: Vec<Motor>, right_motors: Vec<Motor>, track_width: f64, wheel_size: f64, drive_gear_ratio: f64, horizontal_drift: f64) -> Self {
        Self { left_motors, right_motors, track_width, wheel_size, drive_gear_ratio, horizontal_drift }
    }
}

#[derive(Default)]
pub struct Sensors {
    pub vertical_1: Option<TrackingWheel>,
    pub vertical_2: Option<TrackingWheel>,
    pub horizontal_1: Option<TrackingWheel>,
    pub horizontal_2: Option<TrackingWheel>,
    pub imu: Option<InertialSensor>,
    pub imu_scaler: Option<f64>,
    pub imu_calibrated: bool,
}

impl Sensors {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn vert1(&mut self, vertical_1: TrackingWheel) -> &mut Self {
        self.vertical_1 = Some(vertical_1); self
    }

    pub fn vert2(&mut self, vertical_2: TrackingWheel) -> &mut Self {
        self.vertical_2 = Some(vertical_2); self
    }

    pub fn hori1(&mut self, horizontal_1: TrackingWheel) -> &mut Self {
        self.horizontal_1 = Some(horizontal_1); self
    }

    pub fn hori2(&mut self, horizontal_2: TrackingWheel) -> &mut Self {
        self.horizontal_2 = Some(horizontal_2); self
    }

    pub fn imu(&mut self, imu: InertialSensor, scaler: f64) -> &mut Self {
        self.imu = Some(imu); self.imu_scaler = Some(scaler); self
    }
}

#[derive(Clone)]
pub struct ControllerCurve {
    deadband: f64,
    min_output: f64,
    gain: f64
}

impl ControllerCurve {
    pub const fn new(deadband: f64, min_output: f64, gain: f64) -> Self {
        Self { deadband, min_output, gain }
    }

    pub fn curve(&self, input: f64) -> f64 {
        let sius = 1.0 / (self.gain.powf(1.0 - self.deadband - 1.0) * (1.0 - self.deadband));
        let iu = self.gain.powf(input.abs() - self.deadband - 1.0) * (input.abs() - self.deadband) * sius;
        (1.0 - self.min_output) / 1.0 * iu - self.min_output
    }
}

impl Default for ControllerCurve {
    fn default() -> Self {
        Self {
            deadband: 0.0,
            min_output: 0.0,
            gain: 1.0,
        }
    }
}

#[derive(Clone)]
pub struct Chassis {
    pub drivetrain: Rc<RwLock<Drivetrain>>,
    pub linear: Rc<RwLock<Pid>>,
    pub angular: Rc<RwLock<Pid>>,
    pub sensors: Rc<RwLock<Sensors>>,
    pub throttle: ControllerCurve,
    pub steer: ControllerCurve,
    pub odom: Rc<RwLock<OdomLoop>>,
    pub controller: Rc<RwLock<Controller>>,
    pub dist_travelled: Rc<RwLock<f64>>,
    pub motion_running: Rc<Mutex<(bool, bool)>>,
    pub motion_start: Rc<Mutex<Instant>>,
}

impl Chassis {
    pub fn new(drivetrain: Drivetrain, sensors: Sensors, controller: Controller) -> Self {
        let drivetrain = Rc::new(RwLock::new(drivetrain));
        let sensors = Rc::new(RwLock::new(sensors));
        let odom = Rc::new(RwLock::new(OdomLoop::new(drivetrain.clone(), sensors.clone())));
        Self {
            drivetrain,
            linear: Rc::new(RwLock::new(Pid::default())),
            angular: Rc::new(RwLock::new(Pid::default())),
            sensors,
            throttle: ControllerCurve::default(),
            steer: ControllerCurve::default(),
            odom,
            controller: Rc::new(RwLock::new(controller)),
            dist_travelled: Rc::new(RwLock::new(0.0)),
            motion_running: Rc::new(Mutex::new((false, false))),
            motion_start: Rc::new(Mutex::new(Instant::now()))
        }
    }

    pub async fn calibrate_imu(&mut self) -> bool {
        let mut sensors = self.sensors.write().await;
        let mut controller = self.controller.write().await;
        
        println!("Calibrating IMU");
        controller.try_set_text("Calibrating...", 3, 1).ok();

        let calibration_start = Instant::now();
        if let Err(e) = sensors.imu.as_mut().unwrap().calibrate().await {
            eprintln!("Calibration Failed with Error: {e} in");
            controller.try_set_text("Calibration Failed!", 3, 1).ok();
            drop(sensors);
            drop(controller);
            return false;
        }
        let time = calibration_start.elapsed().as_secs_f64();
        self.sensors.write().await.imu_calibrated = true;
        println!("Calibration Succeeded in {time:.2} Seconds!");
        controller.try_set_text(format!("Calibrated in {time:.2}"), 3, 1).ok();
        
        drop(sensors);
        drop(controller);
        true
    }

    pub async fn calibrate(&mut self, calibrate_imu: bool) -> Task<()> {
        if calibrate_imu {
            for _ in 0..5 {
                if self.calibrate_imu().await {
                    self.controller.write().await.try_rumble(".").ok();
                    break;
                };
            }
            if !self.sensors.read().await.imu_calibrated {
                self.controller.write().await.try_rumble("...").ok();
            }
        }

        let mut drive  = self.drivetrain.write().await;
        drive.left_motors.reset().ok();
        drive.right_motors.reset().ok();
        drop(drive);

        let mut sensors = self.sensors.write().await;
        if let Some(v1) = &mut sensors.vertical_1 { v1.reset().ok(); };
        if let Some(v2) = &mut sensors.vertical_2 { v2.reset().ok(); };
        if let Some(h1) = &mut sensors.horizontal_1 { h1.reset().ok(); };
        if let Some(h2) = &mut sensors.horizontal_2 { h2.reset().ok(); };
        drop(sensors);

        OdomLoop::odom_loop(self.odom.clone()).await
    }

    pub async fn set_pose(&mut self, pose: Pose, local: bool, radians: bool, standard_pos: bool) {
        self.odom.write().await.set_pose(pose, local, radians, standard_pos);
    }

    pub async fn get_pose(&self, local: bool, radians: bool, standard_pos: bool) -> Pose {
        self.odom.read().await.get_pose(local, radians, standard_pos)
    }

    pub async fn get_speed(&self, local: bool, radians: bool, standard_pos: bool) -> Pose {
        self.odom.read().await.get_speed(local, radians, standard_pos)
    }

    pub async fn estimate_pose(&self, time: f64, radians: bool) -> Pose {
        self.odom.read().await.estimate_pose(time, radians)
    }

    pub async fn wait_until_distance(&self, distance: f64) {
        loop {
            if *self.dist_travelled.read().await > distance { return; }
            vexide::prelude::sleep(Duration::from_millis(5)).await;
        }
    }

    pub async fn wait_until_angle(&self, angle: f64, local: bool, radians: bool, standard_pos: bool) {
        loop {
            if (self.get_pose(local, radians, standard_pos).await.theta - angle).abs() < 0.2 {
                return;
            }
            vexide::prelude::sleep(Duration::from_millis(5)).await;
        }
    }

    pub async fn wait_until_done(&self) {
        loop {
            if *self.dist_travelled.read().await != -1.0 { return; }
            vexide::prelude::sleep(Duration::from_millis(5)).await;
        }
    }

    pub async fn start_motion(&mut self, local_reset: bool) -> Option<MutexGuard<'_, Instant>> {
        let mut running = self.motion_running.lock().await;
        if running.0 { running.1 = true; }
        else { running.0 = true; }
        drop(running);
        let mut self_clone = self.clone();
        let mut mutex = self.motion_start.lock().await;
        if !self.motion_running.lock().await.0 { drop(mutex); return None; }
        *self_clone.dist_travelled.write().await = 0.0;
        if local_reset { self_clone.set_pose(Pose::default(), true, true, false).await; }
        *mutex = Instant::now();
        Some(mutex)
    }

    pub async fn update_distance(&mut self, last_pose: Pose, local: bool) -> Pose {
        let pose = self.get_pose(local, false, false).await;
        *self.dist_travelled.write().await += pose.distance(last_pose);
        pose
    }

    pub async fn end_motion(&mut self, mutex: Option<MutexGuard<'_, Instant>>) {
        self.tank(0.0, 0.0, true).await;
        *self.dist_travelled.write().await = -1.0;
        let mut queue = self.motion_running.lock().await;
        queue.0 = queue.1; queue.1 = false;
        drop(mutex);
    }

    pub async fn cancel_motion(&mut self) {
        self.motion_running.lock().await.0 = false;
    }

    pub async fn cancel_all_motions(&mut self) {
        let mut lock = self.motion_running.lock().await;
        lock.0 = false;
        lock.1 = false;
        drop(lock);
    }

    pub async fn set_brake_mode(&mut self, mode: BrakeMode) {
        let mut drive = self.drivetrain.write().await;
        drive.left_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
        drive.right_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
        drop(drive);
    }
}