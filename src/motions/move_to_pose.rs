#[derive(Debug, Default, Clone, Copy)]
pub(crate) struct MoveToPoseParams {
    pub forwards: bool = true,
    pub horizontal_drift: Option<f64> = None,
    pub f_lead: f64 = 0.6,
    // pub g_lead: f64 = 1.0
    pub max_speed: f64 = 1.0,
    pub min_speed: f64 = 1.0,
    pub early_exit_range: f64 = 0.0,
}