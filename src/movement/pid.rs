#[derive(Copy, Clone, Debug)]
pub struct PidConfig {
    /// How strongly to correct current error
    pub kp: f32,

    /// How strongly to correct to long term drift
    pub ki: f32,

    /// How strongly to correct for predicted error
    pub kd: f32,

    /// The max allowed value for the integral
    pub max_i: f32,
}

/// Implementation of PID algorithm
#[derive(Clone, Debug)]
pub struct PidController {
    pid: PidConfig,

    integral: f32,
    last_error: f32,
}

impl PidController {
    pub fn new(pid: PidConfig) -> Self {
        PidController {
            pid,
            integral: 0.0,
            last_error: 0.0,
        }
    }

    pub fn update(&mut self, error: f32) -> (f32, (f32, f32, f32)) {
        let cfg = &self.pid;

        self.integral += error;
        self.integral = self.integral.clamp(-cfg.max_i, cfg.max_i);

        let proportional = error;
        let integral = self.integral;
        let derivative = error - self.last_error;

        self.last_error = error;

        let p = cfg.kp * proportional;
        let i = cfg.ki * integral;
        let d = cfg.kd * derivative;

        (p + i + d, (p, i, d))
    }
}
