use std::time::Duration;

use crate::types::{Degrees, DegreesPerSecond, DegreesPerSecondPerSecond};

/// Repersents a precalculated acceleration curve with 4 phases
/// Acceleration, Constant speed, Deceleration, Stopping
pub struct TrapezoidalAcceleration {
    acceleration: f32,
    deceleration: f32,

    target_speed: f32,
    total_distance: f32,
    total_duration: f32,

    acceleration_end: f32,
    const_end: f32,
    deceleration_end: f32,

    solved: bool,
}

/// Models the curent phase of the acceleration curve
pub enum AccelerationPhase {
    RampUp,
    Constant,
    RampDown,
    Stopping,
    Illegal,
}

/// Slowest speed the robot will go in degrees per second
pub const MIN_SPEED: f32 = 100.0;
/// The amount time to spend in the "Stopping"
pub const STOPPING_DURATION: f32 = 0.25;
/// Length of "Stoping" phase
pub const STOPPING_LENGTH: f32 = STOPPING_DURATION * MIN_SPEED;

impl TrapezoidalAcceleration {
    /// Generates a new acceleration curve
    pub fn new(
        Degrees(total_distance): Degrees,
        DegreesPerSecond(target_speed): DegreesPerSecond,
        DegreesPerSecondPerSecond(acceleration): DegreesPerSecondPerSecond,
        DegreesPerSecondPerSecond(deceleration): DegreesPerSecondPerSecond,
    ) -> Self {
        assert!(
            total_distance >= 0.0,
            "total_distance must be greater than 0"
        );
        assert!(target_speed >= 0.0, "target_speed must be greater than 0");
        assert!(acceleration >= 0.0, "acceleration must be greater than 0");
        assert!(deceleration >= 0.0, "deceleration must be greater than 0");

        let target_speed = {
            // Calculate the speed needed to solve the curve if the curve is unsolvable with the provided target_speed
            let special_acceleration_distance =
                (total_distance - STOPPING_LENGTH) * deceleration / (acceleration + deceleration);
            let special_target_speed =
                (MIN_SPEED * MIN_SPEED + 2.0 * acceleration * special_acceleration_distance).sqrt();

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

            let constant_distance =
                total_distance - (acceleration_distance + deceleration_distance + STOPPING_LENGTH);

            constant_distance / target_speed
        };

        let total_duration =
            acceleration_duration + constant_duration + deceleration_duration + STOPPING_DURATION;

        // Calculate the points at which
        let acceleration_end = acceleration_duration;
        let const_end = acceleration_end + constant_duration;
        let deceleration_end = const_end + deceleration_duration;

        let solved =
            target_speed > MIN_SPEED && total_duration.is_normal() && constant_duration > -0.0001;

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
                solved,
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
                solved,
            }
        }
    }

    /// Gets the speed form the acceleration curve at the current time in seconds
    pub fn get_speed(&self, time: Duration) -> (f32, AccelerationPhase) {
        let sec = time.as_secs_f32();

        if self.target_speed <= MIN_SPEED {
            // The acceleration curve is not defined for the requested parameters
            return (MIN_SPEED, AccelerationPhase::Stopping);
        }

        if sec < 0.0 {
            // Backup definition for illegal inputs
            (MIN_SPEED, AccelerationPhase::Illegal)
        } else if sec < self.acceleration_end {
            // Acceleration
            (
                self.acceleration * sec + MIN_SPEED,
                AccelerationPhase::RampUp,
            )
        } else if sec < self.const_end {
            // Constant
            (self.target_speed, AccelerationPhase::Constant)
        } else if sec < self.deceleration_end {
            // Deceleration
            (
                self.target_speed - self.deceleration * (sec - self.const_end),
                AccelerationPhase::RampDown,
            )
        } else if sec < self.total_duration {
            // Stopping
            (MIN_SPEED, AccelerationPhase::Stopping)
        } else {
            // Backup definition for illegal inputs
            (MIN_SPEED, AccelerationPhase::Illegal)
        }
    }

    // The distance this acceleration curve covers
    pub fn distance(&self) -> f32 {
        self.total_distance
    }

    // The time this acceleration curve takes to run
    pub fn duration(&self) -> f32 {
        self.total_duration
    }

    // The target_speed of this acceleration curve
    pub fn target_speed(&self) -> f32 {
        self.target_speed
    }

    // Returns true if some acceleration curve was generated
    // Otherwise, a flat curve without acceleration will be used
    pub fn solved(&self) -> bool {
        self.solved
    }
}
