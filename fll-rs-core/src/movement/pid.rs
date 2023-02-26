#[derive(Copy, Clone, Debug)]
pub struct PidConfig {
    /// how strongly to correct current error
    pub kp: f32,
    
    /// how strongly to correct to long term drift
    pub ki: f32,

    /// how strongly to correct to predicted error
    pub kd: f32,
}

/// Implementation of PID algorithm
#[derive(Clone, Debug)]
pub struct PidController {
    pid: PidConfig,

    integral: f32,
    last_error: f32
}

impl PidController {
    pub fn new(pid: PidConfig) -> Self {
        PidController { pid, integral: 0.0, last_error: 0.0 }
    }

    // TODO could this be improved with knowledge of the duration of the time step?
    pub fn update(&mut self, error: f32) -> f32 {
        self.integral += error;

        let cfg = &self.pid;

        let proportional = error;
        let integral = self.integral;
        let derivative = error - self.last_error;

        self.last_error = error;

        cfg.kp * proportional + cfg.ki * integral + cfg.kd + derivative
    }
}
