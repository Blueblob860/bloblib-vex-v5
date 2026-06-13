use std::{rc::Rc, sync::Arc, time::{Duration, Instant}};

use async_trait::async_trait;
use vexide::{sync::Mutex, time::sleep};

use crate::{chassis::Chassis, odom::Pose};

#[async_trait(?Send)]
pub(crate) trait Motion {
    async fn setup(&mut self, chassis: &mut Chassis);

    async fn tick(&mut self, chassis: &mut Chassis) -> bool;

    async fn cleanup(&mut self, chassis: &mut Chassis);
}

pub(crate) enum MotionFlag {
    New(Box<dyn Motion>, Duration),
    CancelCurrent,
    CancelAll,
}

pub(crate) struct MotionHandler {
    pub active: bool,
    queue: Vec<(Box<dyn Motion>, Duration)>,
    chassis: Chassis,
    last_motion_start: Instant,
    pub buffer: Rc<Mutex<Vec<MotionFlag>>>,
}

impl MotionHandler {
    pub async fn new(chassis: Chassis) -> Self {
        Self {
            active: false,
            queue: vec![],
            chassis,
            last_motion_start: Instant::now(),
            buffer: Rc::new(Mutex::new(vec![]))
        }
    }

    pub async fn handle(&mut self) {
        let mut last_pose = self.chassis.get_local_pose(false, false).await;
        loop {
            // Sync the Motion Handler's internal queue to the buffer
            let mut buf = self.buffer.lock().await;
            buf.reverse();
            for flag in 0..buf.len() {
                match buf.pop().unwrap() {
                    MotionFlag::New(motion, duration) => {
                        self.queue.push((motion, duration));
                    },
                    MotionFlag::CancelCurrent => {
                        if let Some(motion) = self.queue.first_mut() {
                            motion.0.cleanup(&mut self.chassis).await;
                        }
                        self.chassis.tank(0.0, 0.0, true).await;
                        *self.chassis.dist_travelled.write().await = -1.0;
                        self.active = false;
                        self.queue.remove(0);
                    },
                    MotionFlag::CancelAll => {
                        if let Some(motion) = self.queue.first_mut() {
                            motion.0.cleanup(&mut self.chassis).await;
                        }
                        self.chassis.tank(0.0, 0.0, true).await;
                        *self.chassis.dist_travelled.write().await = -1.0;
                        self.active = false;
                        self.queue.clear();
                    },
                }
            }

            // Update chassis based on current motion
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
                motion.0.setup(&mut self.chassis).await;
                self.last_motion_start = Instant::now();
            }
            if motion.0.tick(&mut self.chassis).await || self.last_motion_start.elapsed() >= motion.1 {
                motion.0.cleanup(&mut self.chassis).await;
                self.chassis.tank(0.0, 0.0, true).await;
                self.queue.remove(0);
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