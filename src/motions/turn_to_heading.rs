#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub(crate) enum AngularDirection {
    #[default]
    Auto,
    Clockwise,
    CounterClockwise
}

pub(crate) struct TurnToHeadingParams {
    direction: AngularDirection = AngularDirection::Auto,
    max_speed: f64 = 1.0,
    min_speed: f64 = 1.0,
    early_exit_speed: f64 = 1.0
}