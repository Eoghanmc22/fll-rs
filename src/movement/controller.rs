use std::thread;
use std::time::{Duration, Instant};
use crate::math;
use crate::Result;
use crate::movement::acceleration::TrapezoidalAcceleration;
use crate::movement::pid::{PidConfig, PidController};
use crate::movement::spec::RobotSpec;
use crate::robot::{Command, Motor, Robot, StopAction, TurnType};

/// The standard implementation of movement
pub struct MovementController {
    pid_config: PidConfig,

    target_direction: f32,
    spec: RobotSpec
}

impl MovementController {
    pub fn new(pid_config: PidConfig, spec: RobotSpec) -> Self {
        MovementController {
            pid_config,
            target_direction: 0.0,
            spec
        }
    }

    pub fn drive(&self, robot: &dyn Robot, distance: i32, speed: i32) -> Result<()> {
        let spec = &self.spec;
        let current_max_speed = spec.max_speed() * robot.battery() - 100.0;

        let sign = (distance.signum() * speed.signum()) as f32 * spec.gear_ratio().signum();
        let distance = distance.abs() as f32;
        let speed = math::clampf(speed.abs() as f32, -current_max_speed, current_max_speed);

        // Would take infinite or zero time to complete
        assert_ne!(sign, 0.0, "Illegal parameters!");

        // Generate the acceleration curve
        let acceleration = TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());
        // Setup the PID controller
        let mut pid = PidController::new(self.pid_config);

        // Setup motors
        robot.motor_reset(Motor::DriveRight, Some(StopAction::Hold));
        robot.motor_reset(Motor::DriveLeft, Some(StopAction::Hold));

        // Record the start time for acceleration
        let start = Instant::now();

        while position(robot, (true, true)) < distance {
            // Update the PID controller
            let error = math::subtract_angles(self.target_direction, robot.facing());
            let correction = pid.update(error) * sign;

            // Get position on acceleration curve
            let speed = acceleration.now(start).0;

            // Use correction from the PID controller to create per wheel speed
            let (speed_left, speed_right) = {
                let speed_right = speed - speed * correction / 50.0;
                let speed_right = math::clampf(speed_right, -current_max_speed, current_max_speed);

                let speed_left = speed + speed * correction / 50.0;
                let speed_left = math::clampf(speed_left, -current_max_speed, current_max_speed);

                (speed_left, speed_right)
            };

            robot.motor(Motor::DriveRight, Command::To((distance * spec.error_wheel_right() * sign) as i32, speed_right as i32));
            robot.motor(Motor::DriveLeft, Command::To((distance * spec.error_wheel_left() * sign) as i32, speed_left as i32));

            robot.handle_interrupt()?;

            // TODO tune
            // Maybe change to yield with temporal adjustments for pid and stuff
            thread::sleep(Duration::from_millis(10))
        }

        robot.wait(Motor::DriveRight);
        robot.wait(Motor::DriveLeft);

        // Implicit Hold stop

        Ok(())
    }

    pub fn turn(&self, robot: &dyn Robot, angle: i32, speed: i32) -> Result<()> {
        let difference = math::subtract_angles(angle as f32, self.target_direction);

        let turn_type = if difference < 0.0 {
            TurnType::Right
        } else {
            TurnType::Left
        };

        self.turn_named(robot, angle, speed, turn_type)
    }

    // Has a lot in common with drive, could they be merged?
    pub fn turn_named(&self, robot: &dyn Robot, angle: i32, speed: i32, turn: TurnType) -> Result<()> {
        assert!(speed > 0, "Speed must be greater than 0");

        let spec = &self.spec;
        let current_max_speed = spec.max_speed() * robot.battery() - 100.0;

        // Calculate how much the wheels need to move for this turn
        // Abs angle -> Rel angle -> distance -> left and right degrees
        let difference = math::subtract_angles(angle as f32, robot.facing());
        let distance = spec.get_distance_for_turn(difference);
        let (dist_left, dist_right) = turn_split(&turn, distance, spec);

        // No turn is necessary
        if difference.abs() < 3.0 {
            return Ok(());
        }

        let sign = distance.signum() * spec.gear_ratio().signum();
        let distance = distance.abs();
        let speed = math::clampf(speed as f32, -current_max_speed, current_max_speed);

        // Generate the acceleration curve
        let acceleration = TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());

        // Setup motors
        robot.motor_reset(Motor::DriveRight, Some(StopAction::Hold));
        robot.motor_reset(Motor::DriveLeft, Some(StopAction::Hold));

        // Record the start time for acceleration
        let start = Instant::now();

        while position(robot, turn.wheels()) < distance {
            // Get position on acceleration curve
            let speed = acceleration.now(start).0;

            let (speed_left, speed_right) = turn_split(&turn, speed, spec);

            if dist_right != 0.0 && speed_right != 0.0 { robot.motor(Motor::DriveRight, Command::To((dist_right * sign) as i32, speed_right as i32)); }
            if dist_left != 0.0 && speed_left != 0.0 { robot.motor(Motor::DriveLeft, Command::To((dist_left * sign) as i32, speed_left as i32)); }

            robot.handle_interrupt()?;

            // TODO tune
            thread::sleep(Duration::from_millis(10))
        }

        // Implicit Hold stop

        Ok(())
    }
}

fn position(robot: &dyn Robot, wheels: (bool, bool)) -> f32 {
    let mut sum = 0.0;
    let mut count = 0.0;

    if wheels.0 {
        let left_pos = robot.motor_angle(Motor::DriveLeft).abs();

        sum += left_pos as f32;
        count += 1.0;
    }
    if wheels.1 {
        let right_pos = robot.motor_angle(Motor::DriveRight).abs();

        sum += right_pos as f32;
        count += 1.0;
    }

    sum / count
}

fn turn_split(turn_type: &TurnType, val: f32, spec: &RobotSpec) -> (f32, f32) {
    match turn_type {
        TurnType::Right => {
            (0.0, val * spec.error_wheel_right())
        }
        TurnType::Left => {
            (val * spec.error_wheel_left(), 0.0)
        }
        TurnType::Center => {
            (val * spec.error_wheel_left() / 2.0, val * spec.error_wheel_right() / 2.0)
        }
    }
}