use core::f64;

use crate::motions::turn_to_heading::AngularDirection;

pub(crate) fn sanitize_angle(angle: f64, radians: bool) -> f64 {
    if radians {
        angle.rem_euclid(f64::consts::TAU)
    } else {
        angle.rem_euclid(360.0)
    }
}

pub(crate) fn angle_error(target: f64, position: f64, radians: bool, direction: AngularDirection) -> f64 {
    let target = sanitize_angle(target, radians);
    let position = sanitize_angle(position, radians);
    let max = if radians { f64::consts:: TAU } else { 360.0 };
    let raw_error = target - position;
    match direction {
        AngularDirection::Auto => raw_error.rem_euclid(max),
        AngularDirection::Clockwise => if raw_error < 0.0 { raw_error + max } else { raw_error },
        AngularDirection::CounterClockwise => if raw_error > 0.0 { raw_error + max } else { raw_error }
    }
}

pub(crate) fn average(vals: Vec<f64>) -> f64 {
    vals.iter().fold(0.0, |a, v| a + v) / vals.len() as f64
}

pub(crate) fn ema(current: f64, previous: f64, smooth: f64) -> f64 {
    smooth * (current - previous) + previous
}

pub(crate) fn sign(val: f64) -> f64 {
    if val < 0.0 { -1.0 } else { 1.0 }
}