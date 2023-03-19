use std::f32::consts::PI;

use crate::types::{
    Acceleration, Degrees, DegreesPerSecond, DegreesPerSecondPerSecond, Distance, Heading,
    Milimeters, Speed, UnitsExt,
};

#[derive(Clone)]
pub struct RobotSpec {
    acceleration: DegreesPerSecondPerSecond,
    deceleration: DegreesPerSecondPerSecond,

    wheel_circumference: Milimeters,
    wheel_diameter: Milimeters,
    wheelbase_circumference: Milimeters,
    wheelbase_diameter: Milimeters,

    max_speed: DegreesPerSecond,
}

impl RobotSpec {
    pub fn new(
        acceleration: impl Into<Acceleration>,
        deceleration: impl Into<Acceleration>,
        wheel_diameter: Milimeters,
        wheelbase_diameter: Milimeters,
        max_speed: impl Into<Speed>,
    ) -> Self {
        let wheel_circumference = PI * wheel_diameter.0;
        let wheelbase_circumference = PI * wheelbase_diameter.0 * 2.0;

        let preliminary_spec = RobotSpec {
            acceleration: DegreesPerSecondPerSecond(0.0),
            deceleration: DegreesPerSecondPerSecond(0.0),
            wheel_circumference: wheel_circumference.mm(),
            wheel_diameter,
            wheelbase_circumference: wheelbase_circumference.mm(),
            wheelbase_diameter,
            max_speed: DegreesPerSecond(0.0),
        };

        RobotSpec {
            acceleration: acceleration.into().to_dps2(&preliminary_spec),
            deceleration: deceleration.into().to_dps2(&preliminary_spec),
            max_speed: max_speed.into().to_dps(&preliminary_spec),
            ..preliminary_spec
        }
    }

    /// The acceleration rate used for movement
    /// In degrees per second per second
    pub fn acceleration(&self) -> DegreesPerSecondPerSecond {
        self.acceleration
    }

    /// The deceleration rate used for movement
    /// In degrees per second per second
    pub fn deceleration(&self) -> DegreesPerSecondPerSecond {
        self.deceleration
    }

    /// The circumference of the main wheels on the robot
    /// In millimeters
    pub fn wheel_circumference(&self) -> Milimeters {
        self.wheel_circumference
    }

    /// The diameter of the main wheels on the robot
    /// In millimeters
    pub fn wheel_diameter(&self) -> Milimeters {
        self.wheel_diameter
    }

    /// The turning circumference of the robot's wheelbase
    /// In millimeters
    pub fn wheelbase_circumference(&self) -> Milimeters {
        self.wheelbase_circumference
    }

    /// The distance between the robot's 2 wheels
    /// In millimeters
    pub fn wheelbase_diameter(&self) -> Milimeters {
        self.wheelbase_diameter
    }

    /// The max supported driving speed
    /// In degrees per second
    pub fn max_speed(&self) -> DegreesPerSecond {
        self.max_speed
    }

    /// Calculated the amount of degrees a wheel would need to turn to cause the robot to turn
    /// `angle` degrees
    ///
    /// Assumes the robot has a heading of 0
    /// The actual heading should be subtracted from `angle`
    pub fn get_distance_for_turn(&self, Heading(angle): Heading) -> Degrees {
        (angle * self.wheelbase_circumference.0 / self.wheel_circumference.0).deg()
    }

    /// Calculated the direction the robot should be facing based on the wheel angles
    pub fn get_approx_angle(
        &self,
        left: impl Into<Distance>,
        right: impl Into<Distance>,
    ) -> Heading {
        ((left.into().to_deg(&self).0 - right.into().to_deg(&self).0) * self.wheel_circumference.0
            / self.wheelbase_circumference.0)
            .ang()
    }

    /// Converts wheel degrees to millimeters
    pub fn deg_to_mm(&self, Degrees(deg): Degrees) -> Milimeters {
        (deg / 360.0 * self.wheel_circumference.0).mm()
    }

    /// Converts millimeters to wheel degrees
    pub fn mm_to_deg(&self, Milimeters(mm): Milimeters) -> Degrees {
        (mm / self.wheel_circumference.0 * 360.0).deg()
    }
}
