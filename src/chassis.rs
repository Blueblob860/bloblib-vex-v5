use core::f64;
use std::{rc::Rc, time::{Duration, Instant}};

use vexide::{prelude::{Controller, InertialSensor, Motor}, smart::motor::BrakeMode, sync::{Mutex, MutexGuard, RwLock}, task::Task, time::sleep};

use crate::{motion_handler::{Motion, MotionFlag, MotionHandler}, odom::{OdomLoop, Pose}, pid::Pid, tracking_wheel::{Encoder, TrackingWheel}};

pub(crate) struct Drivetrain {
    pub(crate) left_motors: Vec<Motor>,
    pub(crate) right_motors: Vec<Motor>,
    pub(crate) track_width: f64,
    pub(crate) wheel_size: f64,
    pub(crate) drive_gear_ratio: f64,
    pub(crate) horizontal_drift: f64,
}

impl Drivetrain {
    pub(crate) fn new(left_motors: Vec<Motor>, right_motors: Vec<Motor>, track_width: f64, wheel_size: f64, drive_gear_ratio: f64, horizontal_drift: f64) -> Self {
        Self { left_motors, right_motors, track_width, wheel_size, drive_gear_ratio, horizontal_drift }
    }
}

pub(crate) struct Sensors {
    pub(crate) vertical_1: Option<TrackingWheel>,
    pub(crate) vertical_2: Option<TrackingWheel>,
    pub(crate) horizontal_1: Option<TrackingWheel>,
    pub(crate) horizontal_2: Option<TrackingWheel>,
    pub(crate) imu: Option<InertialSensor>,
    pub(crate) imu_scaler: Option<f64>,

    pub(crate) imu_calibrated: bool,
}

impl Sensors {
    pub(crate) fn new() -> Self {
        Self {
            vertical_1: None,
            vertical_2: None,
            horizontal_1: None,
            horizontal_2: None,
            imu: None,
            imu_scaler: None,

            imu_calibrated: false,
        }
    }

    pub(crate) fn vert1(&mut self, vertical_1: TrackingWheel) -> &mut Self {
        self.vertical_1 = Some(vertical_1); self
    }

    pub(crate) fn vert2(&mut self, vertical_2: TrackingWheel) -> &mut Self {
        self.vertical_2 = Some(vertical_2); self
    }

    pub(crate) fn hori1(&mut self, horizontal_1: TrackingWheel) -> &mut Self {
        self.horizontal_1 = Some(horizontal_1); self
    }

    pub(crate) fn hori2(&mut self, horizontal_2: TrackingWheel) -> &mut Self {
        self.horizontal_2 = Some(horizontal_2); self
    }

    pub(crate) fn imu(&mut self, imu: InertialSensor, scaler: f64) -> &mut Self {
        self.imu = Some(imu); self.imu_scaler = Some(scaler); self
    }
}

#[derive(Clone)]
pub(crate) struct ControllerCurve {
    deadband: f64,
    min_output: f64,
    gain: f64
}

impl ControllerCurve {
    pub(crate) const fn new(deadband: f64, min_output: f64, gain: f64) -> Self {
        Self { deadband, min_output, gain }
    }

    pub(crate) fn curve(&self, input: f64) -> f64 {
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
pub(crate) struct Chassis {
    pub(crate) drivetrain: Rc<RwLock<Drivetrain>>,
    pub(crate) linear: Rc<RwLock<Pid>>,
    pub(crate) angular: Rc<RwLock<Pid>>,
    pub(crate) heading: Rc<RwLock<Pid>>,
    pub(crate) sensors: Rc<RwLock<Sensors>>,
    pub(crate) throttle: ControllerCurve,
    pub(crate) steer: ControllerCurve,
    pub(crate) odom: Rc<RwLock<OdomLoop>>,
    pub(crate) controller: Rc<RwLock<Controller>>,
    pub(crate) dist_travelled: Rc<RwLock<f64>>,
    pub(crate) motion_mutex: Rc<Mutex<Vec<MotionFlag>>>,
}

impl Chassis {
    pub(crate) fn new(drivetrain: Rc<RwLock<Drivetrain>>, sensors: Rc<RwLock<Sensors>>, controller: Rc<RwLock<Controller>>) -> Self {
        let odom = Rc::new(RwLock::new(OdomLoop::new(drivetrain.clone(), sensors.clone())));
        Self {
            drivetrain,
            linear: Rc::new(RwLock::new(Pid::default())),
            angular: Rc::new(RwLock::new(Pid::default())),
            heading: Rc::new(RwLock::new(Pid::default())),
            sensors,
            throttle: ControllerCurve::default(),
            steer: ControllerCurve::default(),
            odom: odom.clone(),
            controller,
            dist_travelled: Rc::new(RwLock::new(0.0)),
            motion_mutex: Rc::new(Mutex::new(vec![]))
        }
    }

    pub(crate) async fn calibrate_imu(&mut self) -> bool {
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

    pub(crate) async fn calibrate(&mut self, calibrate_imu: bool) -> (Task<()>, Task<()>) {
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

        let odom_task = OdomLoop::odom_loop(self.odom.clone());
        let mut motion_handler = MotionHandler::new(self.clone()).await;
        let motion_task = vexide::prelude::spawn(async move { motion_handler.handle().await; });
        (odom_task.await, motion_task)
    }

    pub(crate) async fn set_global_pose(&mut self, pose: Pose, radians: bool) {
        self.odom.write().await.set_pose(pose, radians);
    }

    pub(crate) async fn set_local_pose(&mut self, pose: Pose, radians: bool) {
        self.odom.write().await.set_local_pose(pose, radians);
    }

    pub(crate) async fn get_global_pose(&self, radians: bool, standard_pos: bool) -> Pose {
        let mut pose = self.odom.read().await.get_pose(true);
        if standard_pos {
            pose.theta = f64::consts::PI - pose.theta;
        }
        if !radians {
            pose.theta = pose.theta.to_degrees();
        }
        pose
    }

    pub(crate) async fn get_local_pose(&self, radians: bool, standard_pos: bool) -> Pose {
        let mut pose = self.odom.read().await.get_local_pose(true);
        if standard_pos {
            pose.theta = f64::consts::PI - pose.theta;
        }
        if !radians {
            pose.theta = pose.theta.to_degrees();
        }
        pose
    }

    pub(crate) async fn wait_until(&self, distance: f64) {
        loop {
            if *self.dist_travelled.read().await > distance { return; }
            vexide::prelude::sleep(Duration::from_millis(5)).await;
        }
    }

    pub(crate) async fn wait_until_done(&self) {
        loop {
            if *self.dist_travelled.read().await != -1.0 { return; }
            vexide::prelude::sleep(Duration::from_millis(5)).await;
        }
    }

    pub(crate) async fn run_motion(&mut self, motion: Box<dyn Motion>, timeout: Duration) {
        self.motion_mutex.lock().await.push(MotionFlag::New(motion, timeout));
    }

    pub(crate) async fn cancel_motion(&mut self) {
        self.motion_mutex.lock().await.push(MotionFlag::CancelCurrent);
    }

    pub(crate) async fn cancel_all_motions(&mut self) {
        self.motion_mutex.lock().await.push(MotionFlag::CancelAll);
    }

    pub(crate) async fn set_brake_mode(&mut self, mode: BrakeMode) {
        let mut drive = self.drivetrain.write().await;
        drive.left_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
        drive.right_motors.iter_mut().for_each(|m| { m.brake(mode).ok(); });
        drop(drive);
    }
}