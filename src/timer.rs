use std::time::{Duration, Instant};

use vexide::time::sleep;

pub(crate) struct Timer {
    period: f64,
    elapsed: f64,
    start: Instant,
    paused: bool
}

impl Timer {
    pub(crate) fn new(time: f64) -> Self {
        Self {
            period: time,
            elapsed: 0.0,
            start: Instant::now(),
            paused: false,
        }
    }

    fn update(&mut self) {
        if !self.paused {
            self.elapsed += self.start.elapsed().as_secs_f64() * 1000.0;
            self.start = Instant::now();
        }
    }

    pub(crate) fn get_time_set(&mut self) -> f64 {
        self.update();
        self.period
    }

    pub(crate) fn get_time_left(&mut self) -> f64 {
        self.update();
        (self.period - self.elapsed).max(0.0)
    }

    pub(crate) fn get_time_passed(&mut self) -> f64 {
        self.update();
        self.elapsed
    }

    pub(crate) fn is_done(&mut self) -> bool {
        self.update();
        (self.period - self.elapsed) <= 0.0
    }

    pub(crate) fn is_paused(&mut self) -> bool {
        self.update();
        self.paused
    }

    pub(crate) fn set(&mut self, time: f64) {
        self.period = time;
        self.reset();
    }

    pub(crate) fn reset(&mut self) {
        self.elapsed = 0.0;
        self.start = Instant::now();
    }

    pub(crate) fn pause(&mut self) {
        if !self.paused { self.start = Instant::now(); }
        self.paused = true;
    }

    pub(crate) fn resume(&mut self) {
        if self.paused { self.start = Instant::now(); }
        self.paused = false;
    }

    #[must_use = "Sleep futures must be awaited to function"]
    pub(crate) async fn wait_until_done(&mut self) {
        loop {
            if self.is_done() { break; }
            sleep(Duration::from_millis(5)).await;
        }
    }
}