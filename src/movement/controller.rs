use anyhow::{bail, Context};

use crate::error::Result;
use crate::math;
use crate::movement::acceleration::TrapezoidalAcceleration;
use crate::movement::pid::{PidConfig, PidController};
use crate::robot::{AngleProvider, Command, MotorId, Robot, StopAction, TurnType};
use crate::types::{Degrees, DegreesPerSecond, Heading, UnitsExt};
use std::thread;
use std::time::{Duration, Instant};

/// The standard implementation of movement
pub struct MovementController {
    pid_config: PidConfig,
    target_direction: Heading,
}

impl MovementController {
    pub fn new(pid_config: PidConfig) -> Self {
        MovementController {
            pid_config,
            target_direction: Heading(0.0),
        }
    }

    /// Implementation of the gyro drive algorithm
    pub fn drive<R: Robot + AngleProvider>(
        &mut self,
        robot: &R,
        distance: Degrees,
        speed: DegreesPerSecond,
    ) -> Result<()> {
        self.arc(
            robot,
            distance,
            speed,
            (1.0, 1.0),
            EndingCondition::Distance,
        )
    }

    pub fn turn<R: Robot + AngleProvider>(
        &mut self,
        robot: &R,
        angle: Heading,
        speed: DegreesPerSecond,
    ) -> Result<()> {
        // Use target angle so behaivor is deterministic
        let difference = math::subtract_angles(angle, self.target_direction);

        let turn_type = if difference.0 > 0.0 {
            TurnType::Left
        } else {
            TurnType::Right
        };

        self.turn_named(robot, angle, speed, turn_type)
    }

    // Has a lot in common with drive, could they be merged?
    // todo implement pid turns??
    pub fn turn_named<R: Robot + AngleProvider>(
        &mut self,
        robot: &R,
        angle: Heading,
        speed: DegreesPerSecond,
        turn: TurnType,
    ) -> Result<()> {
        let observered_heading = robot.angle().context("Read heading")?;
        let angle_delta = math::subtract_angles(angle, observered_heading);
        let distance = robot.spec().get_distance_for_turn(angle_delta);

        let ratio = match turn {
            TurnType::Left => (1.0, 0.0),
            TurnType::Right => (0.0, -1.0),
            TurnType::Center => (1.0, -1.0),
        };

        self.arc(robot, distance, speed, ratio, EndingCondition::Heading)
    }

    pub fn arc<R: Robot + AngleProvider>(
        &mut self,
        robot: &R,
        distance: Degrees,
        speed: DegreesPerSecond,
        ratio: (f32, f32),
        ending_condition: EndingCondition,
    ) -> Result<()> {
        if distance.0 == 0.0 {
            // Requested movement of 0 deg
            return Ok(());
        }

        // Determine the requested direction
        let direction = distance.0.signum();

        let distance = distance.0.abs();
        let speed = speed.0;

        assert!(speed > 0.0, "Speed must be greater than 0");

        let spec = robot.spec();

        // Clamp requested speed
        let current_max_speed = spec.max_speed().0 * robot.battery()?.0 - 100.0;
        let speed = f32::clamp(speed, 0.0, current_max_speed);

        // Calculate ratios
        let base = f32::max(ratio.0.abs(), ratio.1.abs());
        let ratio_left = ratio.0 / base * direction;
        let ratio_right = ratio.1 / base * direction;

        // Determine which wheels should move at all
        let move_left = (ratio_left.ceil() as i32).signum().abs() as f32;
        let move_right = (ratio_right.ceil() as i32).signum().abs() as f32;

        // Calculate angles
        let start_angle = self.target_direction;
        let delta_angle = spec.get_approx_angle(
            (distance * ratio_left).deg(),
            (distance * ratio_right).deg(),
        );
        let end_angle = math::add_angles(start_angle, delta_angle);

        // Generate the acceleration curve
        let acceleration = TrapezoidalAcceleration::new(
            distance.deg(),
            speed.dps(),
            spec.acceleration(),
            spec.deceleration(),
        );

        // Setup the PID controller
        let mut pid = PidController::new(self.pid_config);

        // Setup motors
        let mut left = robot.motor(MotorId::DriveLeft).context("Left motor")?;
        let mut right = robot.motor(MotorId::DriveRight).context("Right motor")?;

        left.motor_reset(Some(StopAction::Hold))?;
        right.motor_reset(Some(StopAction::Hold))?;

        // Record the start time for acceleration
        let start = Instant::now();

        loop {
            let iter_start = Instant::now();
            let duration_since_start = iter_start - start;

            // TODO check if this is correct
            let average_distance = {
                let raw_left = left.motor_angle().context("Read left wheel angle")?;
                let raw_right = right.motor_angle().context("Read right wheel angle")?;

                // Due to the ratios, the raw data needs to be corrected before it can be
                // compared with the requested distance
                let adjusted_left = raw_left.to_deg(spec).0 / ratio_left;
                let adjusted_right = raw_right.to_deg(spec).0 / ratio_right;

                // Is it possible that ratio_left or ratio_right could be 0
                // Check that adjusted values are normal before including them
                let combined = match (
                    adjusted_left.is_normal() || adjusted_left == 0.0,
                    adjusted_right.is_normal() || adjusted_right == 0.0,
                ) {
                    (true, true) => (adjusted_left.abs() + adjusted_right.abs()) / 2.0,
                    (true, false) => adjusted_left.abs(),
                    (false, true) => adjusted_right.abs(),
                    (false, false) => bail!("Nether wheel has a normal adjusted angle"),
                };

                // eprintln!("raw_l: {raw_left:.3?}, raw_r: {raw_right:.3?}, combined: {combined:.3}, distance: {distance:.3}");

                combined
            };

            // Calculate headings
            let progress = (acceleration.get_distance(duration_since_start) / distance).min(1.0);
            let observered_heading = robot.angle().context("Read heading")?;
            let target_heading = math::lerp_angles(progress, start_angle, end_angle);

            // Determine if loop should break
            let should_break = match ending_condition {
                EndingCondition::Heading => {
                    math::subtract_angles(end_angle, observered_heading).0.abs() <= 2.0
                }
                EndingCondition::Distance => average_distance >= distance,
            };
            if should_break {
                break;
            }

            // Run PID controller
            let error = math::subtract_angles(target_heading, observered_heading).0;
            let (pid, _internal) = pid.update(error);
            let correction = pid / 100.0;

            // eprintln!("obs: {observered_heading:5.3?}, tar: {target_heading:5.3?}, pid: {internal:5.3?}, cor: {correction:5.3?}, err: {error:5.3?}");

            // Get position on acceleration curve
            let speed_base = acceleration.get_speed(duration_since_start).0;

            // Merge corrections with speeds
            let speed_left = (speed_base * ratio_left + correction * current_max_speed) * move_left;
            let speed_right =
                (speed_base * ratio_right - correction * current_max_speed) * move_right;

            // Re-clamp speeds
            let speed_left = f32::clamp(speed_left, -current_max_speed, current_max_speed);
            let speed_right = f32::clamp(speed_right, -current_max_speed, current_max_speed);

            // Send motor commands
            {
                left.raw(Command::Queue(Command::On(speed_left.dps().into()).into()))?;
                right.raw(Command::Queue(Command::On(speed_right.dps().into()).into()))?;

                left.raw(Command::Execute)?;
                right.raw(Command::Execute)?;
            }

            // Exit if requested
            robot.handle_interrupt()?;

            // TODO tune
            thread::sleep(
                Duration::checked_sub(Duration::from_millis(10), iter_start.elapsed())
                    .unwrap_or_default(),
            )
        }

        // Update state
        self.target_direction = end_angle;

        // Request motor stop
        left.raw(Command::Stop(StopAction::Hold))?;
        right.raw(Command::Stop(StopAction::Hold))?;

        // Wait for motors to stop
        left.wait(None)?;
        right.wait(None)?;

        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub enum EndingCondition {
    Heading,
    Distance,
}
