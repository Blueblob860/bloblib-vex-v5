use std::time::Duration;

use vexide::task::{Task, spawn};

use crate::{chassis::Chassis, motion_handler::Motion};

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct MoveToPointParams {
    pub forwards: bool = true,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_range: f64 = 0.0
}

#[derive(Default, Debug, Clone, Copy)]
pub(crate) struct MoveToPoint {
    pub x: f64 = 0.0,
    pub y: f64 = 0.0,
    pub params: MoveToPointParams
}

impl Motion for MoveToPoint {
    fn setup(&mut self, chassis: &mut Chassis) {
        todo!()
    }

    fn tick(&mut self, chassis: &mut Chassis) -> bool {
        todo!()
    }

    fn cleanup(&mut self, chassis: &mut Chassis) {
        todo!()
    }
}

impl Chassis {
    pub async fn move_to_point(&mut self, x: f64, y: f64, timeout: u64, params: MoveToPointParams) {
        self.run_motion(Box::new(MoveToPoint {
            x, y, params
        }), Duration::from_millis(timeout)).await;
    }
}