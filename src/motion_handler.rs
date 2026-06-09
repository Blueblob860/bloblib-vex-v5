use std::{rc::Rc, sync::Arc, time::{Duration, Instant}};

use vexide::{sync::Mutex, time::sleep};

use crate::{chassis::Chassis, odom::Pose};

pub(crate) trait Motion {
    fn setup(&mut self, chassis: &mut Chassis);

    fn tick(&mut self, chassis: &mut Chassis) -> bool;

    fn cleanup(&mut self, chassis: &mut Chassis);
}

pub(crate) enum MotionFlag {
    None,
    CancelCurrent,
    CancelAll,
    New(Box<dyn Motion>, Duration)
}

pub(crate) struct MotionHandler {
    pub active: bool,
    queue: Vec<(Box<dyn Motion>, Duration)>,
    chassis: Chassis,
    last_motion_start: Instant,
    pub flag: Rc<Mutex<MotionFlag>>,
}

impl MotionHandler {
    pub async fn new(chassis: Chassis) -> Self {
        Self {
            active: false,
            queue: vec![],
            chassis,
            last_motion_start: Instant::now(),
            flag: Rc::new(Mutex::new(MotionFlag::None))
        }
    }

    pub async fn handle(&mut self) {
        let mut last_pose = self.chassis.get_local_pose(false, false).await;
        loop {
            if self.queue.is_empty() {
                self.active = false;
                sleep(Duration::from_millis(10)).await;
                continue;
            }
            let motion = self.queue.first_mut().unwrap();
            if !self.active {
                self.active = true;
                self.chassis.set_local_pose(Pose::default(), false).await;
                last_pose = Pose::default();
                *self.chassis.dist_travelled.write().await = 0.0;
                motion.0.setup(&mut self.chassis);
                self.last_motion_start = Instant::now();
            }
            if motion.0.tick(&mut self.chassis) || self.last_motion_start.elapsed() >= motion.1 {
                motion.0.cleanup(&mut self.chassis);
                self.chassis.tank(0.0, 0.0, true).await;
                *self.chassis.dist_travelled.write().await = -1.0;
                self.active = false;
            } else {
                let current_pose = self.chassis.get_local_pose(false, false).await;
                *self.chassis.dist_travelled.write().await += current_pose.distance(last_pose);
                last_pose = current_pose;
            }
        }
    }
}