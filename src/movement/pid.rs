#[derive(Clone, Debug)]
pub struct PidConfig {
    /// how strongly to correct current error
    kp: f64,
    
    /// how strongly to correct to long term drift
    ki: f64,

    /// how strongly to correct to predicted error
    kd: f64,
}

/// Implementation of PID algorithm
#[derive(Clone, Debug)]
pub struct PidController {
    pid: PidConfig,

    integral: f64,
    last_error: f64
}

impl PidController {
    pub fn new(pid: PidConfig) -> Self {
        PidController { pid, integral: 0.0, last_error: 0.0 }
    }

    // TODO could this be improved with knowledge of the duration of the time step?
    pub fn pid(&mut self, error: f64) -> f64 {
        self.integral += error;

        let cfg = &self.pid;

        let proportional = error;
        let integral = self.integral;
        let derivative = error - self.last_error;

        self.last_error = error;

        cfg.kp * proportional + cfg.ki * integral + cfg.kd + derivative
    }
}