pub struct TrapezoidalAcceleration {
    acceleration: f64,
    deceleration: f64,

    target_speed: f64,
    total_distance: f64,
    total_duration: f64,

    acceleration_end: f64,
    const_end: f64,
    deceleration_end: f64,

    solved: bool
}

const MIN_SPEED : f64 = 100.0;
const STOPPING_DURATION: f64 = 0.25;
const STOPPING_LENGTH: f64 = STOPPING_DURATION * MIN_SPEED;

impl TrapezoidalAcceleration {
    /// Generates a new acceleration curve
    pub fn new(total_distance: f64, target_speed: f64, acceleration: f64, deceleration: f64) -> Self {
        assert!(total_distance >= 0.0, "total_distance must be greater than 0");
        assert!(target_speed >= 0.0, "target_speed must be greater than 0");
        assert!(acceleration >= 0.0, "acceleration must be greater than 0");
        assert!(deceleration >= 0.0, "deceleration must be greater than 0");

        let target_speed = {
            // Calculate the speed needed to solve the curve if the curve is unsolvable with the provided target_speed
            let special_acceleration_distance = (total_distance - STOPPING_LENGTH) * deceleration / (acceleration + deceleration);
            let special_target_speed = (MIN_SPEED * MIN_SPEED + 2.0 * acceleration * special_acceleration_distance).sqrt();

            // If special_target_speed is smaller than the provided target_speed it must be used instead
            target_speed.min(special_target_speed)
        };

        // Calculate the acceleration and deceleration durations
        let acceleration_duration = (target_speed - MIN_SPEED) / acceleration;
        let deceleration_duration = (target_speed - MIN_SPEED) / deceleration;

        // Derive constant duration
        let constant_duration = {
            let average_acceleration_speed = acceleration_duration * acceleration / 2.0 + MIN_SPEED;
            let average_deceleration_speed = deceleration_duration * deceleration / 2.0 + MIN_SPEED;

            let acceleration_distance = acceleration_duration * average_acceleration_speed;
            let deceleration_distance = deceleration_duration * average_deceleration_speed;

            let constant_distance = total_distance - (acceleration_distance + deceleration_distance + STOPPING_LENGTH);

            constant_distance / target_speed
        };

        let total_duration = acceleration_duration + constant_duration + deceleration_duration + STOPPING_DURATION;

        // Calculate the points at which
        let acceleration_end = acceleration_duration;
        let const_end = acceleration_end + constant_duration;
        let deceleration_end = const_end + deceleration_duration;

        let solved =
            target_speed > MIN_SPEED &&
                total_duration.is_normal() &&
                constant_duration >= 0.0;

        if solved {
            TrapezoidalAcceleration {
                acceleration,
                deceleration,
                target_speed,
                total_distance,
                total_duration,
                acceleration_end,
                const_end,
                deceleration_end,
                solved
            }
        } else {
            // Curve could not be solved and a flat curve at MIN_SPEED will be used
            let duration = total_distance / MIN_SPEED;

            TrapezoidalAcceleration {
                acceleration: 0.0,
                deceleration: 0.0,
                target_speed: MIN_SPEED,
                total_distance,
                total_duration: duration,
                acceleration_end: 0.0,
                const_end: duration,
                deceleration_end: duration,
                solved
            }
        }
    }

    /// Gets the speed form the acceleration curve at the current time
    pub fn get_speed(&self, time: f64) -> f64 {
        if self.target_speed <= MIN_SPEED {
            // The acceleration curve is not defined for the requested parameters
            return MIN_SPEED;
        }

        if time < 0.0 {
            // Backup definition for illegal inputs
            MIN_SPEED
        } else if time < self.acceleration_end {
            // Acceleration
            self.acceleration * time + MIN_SPEED
        } else if time < self.const_end {
            // Constant
            self.target_speed
        } else if time < self.deceleration_end {
            // Deceleration
            self.target_speed - self.deceleration * (time - self.const_end)
        } else {
            // Stopping and other illegal inputs
            MIN_SPEED
        }
    }

    // The distance this acceleration curve covers
    pub fn distance(&self) -> f64 {
        self.total_distance
    }

    // The time this acceleration curve takes to run
    pub fn duration(&self) -> f64 {
        self.total_duration
    }

    // The target_speed of this acceleration curve
    pub fn target_speed(&self) -> f64 {
        self.target_speed
    }

    // Returns true if some acceleration curve was generated
    // Otherwise, a flat curve without acceleration will be used
    pub fn solved(&self) -> bool {
        self.solved
    }

    pub fn min_speed() -> f64 {
        MIN_SPEED
    }

    pub fn stopping_duration() -> f64 {
        STOPPING_DURATION
    }

    pub fn stopping_length() -> f64 {
        STOPPING_LENGTH
    }
}