use crate::motions::turn_to_heading::AngularDirection;

pub(crate) enum DriveSide {
    LEFT,
    RIGHT
}

pub(crate) struct SwingToHeadingParams {
    pub direction: AngularDirection = AngularDirection::Auto,
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 0.0,
    pub early_exit_speed: f64 = 0.0
}