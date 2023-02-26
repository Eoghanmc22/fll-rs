use std::f32::consts::PI;

#[derive(Clone)]
pub struct RobotSpec {
    acceleration: f32,
    deceleration: f32,

    gear_ratio: f32,
    wheel_circumference: f32,
    wheel_diameter: f32,
    wheel_right_circumference: f32,
    wheel_left_circumference: f32,
    wheelbase_circumference: f32,
    wheelbase_diameter: f32,

    // TODO how are these calculated?
    error_wheelbase: f32,
    error_wheel: f32,
    error_wheel_right: f32,
    error_wheel_left: f32,

    max_speed: f32,
}

impl RobotSpec {
    pub fn new(
        acceleration: f32,
        deceleration: f32,
        wheel_diameter: f32,
        wheelbase_diameter: f32,
        max_speed: f32,
        gear_ratio: f32,

        error_wheel: f32,
        error_difference: f32,
        error_wheelbase: f32,
    ) -> Self {
        let error_wheel_right = 2.0 / ((1.0 / error_difference) + 1.0);
        let error_wheel_left = 2.0 / (error_difference + 1.0);

        let wheel_circumference = PI * wheel_diameter * error_wheel;
        let wheel_right_circumference = wheel_circumference * error_wheel_right;
        let wheel_left_circumference = wheel_circumference * error_wheel_left;
        let wheelbase_circumference = PI * wheelbase_diameter * error_wheelbase * 2.0;

        RobotSpec {
            acceleration,
            deceleration,
            gear_ratio,
            wheel_circumference,
            wheel_diameter,
            wheel_right_circumference,
            wheel_left_circumference,
            wheelbase_circumference,
            wheelbase_diameter,
            error_wheelbase,
            error_wheel,
            error_wheel_right,
            error_wheel_left,
            max_speed,
        }
    }

    /// The acceleration rate used for movement
    /// In degrees per second per second
    pub fn acceleration(&self) -> f32 {
        self.acceleration
    }

    /// The deceleration rate used for movement
    /// In degrees per second per second
    pub fn deceleration(&self) -> f32 {
        self.deceleration
    }

    /// The gear ratio
    /// Dimensionless
    pub fn gear_ratio(&self) -> f32 {
        self.gear_ratio
    }

    /// The circumference of the main wheels on the robot
    /// In millimeters
    pub fn wheel_circumference(&self) -> f32 {
        self.wheel_circumference
    }

    /// The diameter of the main wheels on the robot
    /// In millimeters
    pub fn wheel_diameter(&self) -> f32 {
        self.wheel_diameter
    }

    /// The turning circumference of the robot's wheelbase
    /// In millimeters
    pub fn wheelbase_circumference(&self) -> f32 {
        self.wheelbase_circumference
    }

    /// The distance between the robot's 2 wheels
    /// In millimeters
    pub fn wheelbase_diameter(&self) -> f32 {
        self.wheelbase_diameter
    }

    /// The circumference of the right wheel
    /// Derived from error_difference
    /// In millimeters
    pub fn wheel_right_circumference(&self) -> f32 {
        self.wheel_right_circumference
    }

    /// The circumference of the left wheel
    /// Derived from error_difference
    /// In millimeters
    pub fn wheel_left_circumference(&self) -> f32 {
        self.wheel_left_circumference
    }

    /// The error in the measured wheel base diameter
    /// Dimensionless
    pub fn error_wheelbase(&self) -> f32 {
        self.error_wheelbase
    }

    /// The error in the measured wheel base diameter
    /// Dimensionless
    pub fn error_wheel(&self) -> f32 {
        self.error_wheel
    }

    /// The error between the adjusted wheel circumference and circumference of the right wheel
    /// Dimensionless
    pub fn error_wheel_right(&self) -> f32 {
        self.error_wheel_right
    }

    /// The error between the adjusted wheel circumference and circumference of the left wheel
    /// Dimensionless
    pub fn error_wheel_left(&self) -> f32 {
        self.error_wheel_left
    }

    /// The max supported driving speed
    /// In degrees per second
    pub fn max_speed(&self) -> f32 {
        self.max_speed
    }

    /// Calculated the amount of degrees a wheel would need to turn to cause the robot to turn
    /// `angle` degrees
    pub fn get_distance_for_turn(&self, angle: f32) -> f32 {
        angle * self.wheelbase_circumference / self.wheel_circumference / self.gear_ratio
    }

    /// Calculated the direction the robot should be facing based on the wheel angles
    pub fn get_approx_angle(&self, left: f32, right: f32) -> f32 {
        (left * self.wheel_left_circumference - right * self.wheel_right_circumference)
            * self.gear_ratio
            / self.wheelbase_circumference
    }

    /// Converts wheel degrees to millimeters
    pub fn deg_to_mm(&self, deg: f32) -> f32 {
        deg / 360.0 * self.wheel_circumference * self.gear_ratio
    }

    /// Converts millimeters to wheel degrees
    pub fn mm_to_deg(&self, mm: f32) -> f32 {
        mm / self.gear_ratio / self.wheel_circumference * 360.0
    }
}
