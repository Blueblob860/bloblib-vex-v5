#[derive(Default, Debug, Clone, Copy, PartialEq, PartialOrd)]
pub(crate) enum AngularDirection {
    #[default]
    Auto,
    Clockwise,
    CounterClockwise
}