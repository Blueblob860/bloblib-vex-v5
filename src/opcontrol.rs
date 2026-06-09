use crate::chassis::{Chassis, ControllerCurve};

impl Chassis {
    pub(crate) async fn tank(&mut self, left: f64, right: f64, disable_drive_curve: bool) {
        if disable_drive_curve {
            self.drivetrain.write().await.left_motors.iter_mut().for_each(|m| { m.set_voltage(m.max_voltage() * left).ok(); });
            self.drivetrain.write().await.right_motors.iter_mut().for_each(|m| { m.set_voltage(m.max_voltage() * right).ok(); });
        } else {
            self.drivetrain.write().await.left_motors.iter_mut().for_each(|m| { m.set_voltage(self.throttle.curve(m.max_voltage() * left)).ok(); });
            self.drivetrain.write().await.right_motors.iter_mut().for_each(|m| { m.set_voltage(self.throttle.curve(m.max_voltage() * right)).ok(); });
        }
    }

    pub(crate) async fn arcade(&mut self, throttle: f64, turn: f64, disable_drive_curve: bool, desaturation_bias: f64) {
        let mut throttle = if disable_drive_curve { throttle } else { self.throttle.curve(throttle) };
        let mut turn = if disable_drive_curve { turn } else { self.throttle.curve(turn) };

        if throttle.abs() + turn.abs() > 1.0 {
            let old_throttle = throttle;
            let old_turn = turn;
            throttle = 1.0 - desaturation_bias * old_throttle.abs();
            turn = 1.0 - (1.0 - desaturation_bias) * old_turn.abs();
        }

        let left_power = throttle + turn;
        let right_power = throttle - turn;
        self.tank(left_power, right_power, true).await;
    }

    pub(crate) async fn curvature(&mut self, throttle: f64, turn: f64, disable_drive_curve: bool) {
        if throttle == 0.0 {
            self.arcade(throttle, turn, disable_drive_curve, 0.5).await;
            return;
        }

        let throttle = if disable_drive_curve { throttle } else { self.throttle.curve(throttle) };
        let turn = if disable_drive_curve { turn } else { self.throttle.curve(turn) };

        let mut left_power = throttle + throttle.abs() * turn;
        let mut right_power = throttle - throttle.abs() * turn;
        let max = left_power.abs().max(right_power.abs());
        if max > 1.0 {
            left_power /= max;
            right_power /= max;
        }

        self.tank(left_power, right_power, true).await;
    }
}