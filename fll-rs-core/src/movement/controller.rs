use crate::error::Result;
use crate::math;
use crate::movement::acceleration::TrapezoidalAcceleration;
use crate::movement::pid::{PidConfig, PidController};
use crate::movement::spec::RobotSpec;
use crate::robot::{AngleProvider, Command, Motor, MotorId, Robot, StopAction, TurnType};
use std::thread;
use std::time::{Duration, Instant};

// TODO only have a single method that interfaces with the motors thats versatile enough to
// implement everything else. ie has end condition, control loop, gyro/color as input

/// The standard implementation of movement
pub struct MovementController {
    pid_config: PidConfig,
    target_direction: f32,
}

impl MovementController {
    pub fn new(pid_config: PidConfig) -> Self {
        MovementController {
            pid_config,
            target_direction: 0.0,
        }
    }

    pub fn drive<R: Robot + AngleProvider>(
        &self,
        robot: &R,
        distance: i32,
        speed: i32,
    ) -> Result<()> {
        let spec = robot.spec();
        let current_max_speed = spec.max_speed() * robot.battery()? - 100.0;

        let sign = (distance.signum() * speed.signum()) as f32 * spec.gear_ratio().signum();
        let distance = distance.abs() as f32;
        let speed = f32::clamp(speed.abs() as f32, -current_max_speed, current_max_speed);

        // Would take infinite or zero time to complete
        assert_ne!(sign, 0.0, "Illegal parameters!");

        // Generate the acceleration curve
        let acceleration =
            TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());
        // Setup the PID controller
        let mut pid = PidController::new(self.pid_config);

        // Setup motors
        let right = robot.motor(MotorId::DriveRight);
        let left = robot.motor(MotorId::DriveLeft);

        right.motor_reset(Some(StopAction::Hold))?;
        left.motor_reset(Some(StopAction::Hold))?;

        // Record the start time for acceleration
        let start = Instant::now();

        while position(&[right, left])? < distance {
            // Update the PID controller
            let error = math::subtract_angles(self.target_direction, robot.angle()?);
            let correction = pid.update(error) * sign;

            // Get position on acceleration curve
            let speed = acceleration.get_speed(start.elapsed().as_secs_f32()).0;

            // Use correction from the PID controller to create per wheel speed
            let (speed_left, speed_right) = {
                let speed_right = speed - speed * correction / 50.0;
                let speed_right = f32::clamp(speed_right, -current_max_speed, current_max_speed);

                let speed_left = speed + speed * correction / 50.0;
                let speed_left = f32::clamp(speed_left, -current_max_speed, current_max_speed);

                (speed_left, speed_right)
            };

            right.command(Command::Queue(
                Command::To(
                    (distance * spec.error_wheel_right() * sign) as i32,
                    speed_right as i32,
                )
                .into(),
            ))?;
            left.command(Command::Queue(
                Command::To(
                    (distance * spec.error_wheel_left() * sign) as i32,
                    speed_left as i32,
                )
                .into(),
            ))?;

            right.command(Command::Execute)?;
            left.command(Command::Execute)?;

            robot.handle_interrupt()?;

            // TODO tune
            // Maybe change to yield with temporal adjustments for pid and stuff
            thread::sleep(Duration::from_millis(10))
        }

        right.command(Command::Stop(StopAction::Hold))?;
        left.command(Command::Stop(StopAction::Hold))?;

        right.wait()?;
        left.wait()?;

        Ok(())
    }

    pub fn turn<R: Robot + AngleProvider>(&self, robot: &R, angle: i32, speed: i32) -> Result<()> {
        let difference = math::subtract_angles(angle as f32, self.target_direction);

        let turn_type = if difference < 0.0 {
            TurnType::Right
        } else {
            TurnType::Left
        };

        self.turn_named(robot, angle, speed, turn_type)
    }

    // Has a lot in common with drive, could they be merged?
    // todo implement pid turns??
    pub fn turn_named<R: Robot + AngleProvider>(
        &self,
        robot: &R,
        angle: i32,
        speed: i32,
        turn: TurnType,
    ) -> Result<()> {
        assert!(speed > 0, "Speed must be greater than 0");

        let spec = robot.spec();
        let current_max_speed = spec.max_speed() * robot.battery()? - 100.0;

        // Calculate how much the wheels need to move for this turn
        // Abs angle -> Rel angle -> distance -> left and right degrees
        let difference = math::subtract_angles(angle as f32, robot.angle()?);
        let distance = spec.get_distance_for_turn(difference);
        let (dist_left, dist_right) = turn_split(&turn, distance, spec);

        // No turn is necessary
        if difference.abs() < 3.0 {
            return Ok(());
        }

        let sign = distance.signum() * spec.gear_ratio().signum();
        let distance = distance.abs();
        let speed = f32::clamp(speed as f32, -current_max_speed, current_max_speed);

        // Generate the acceleration curve
        let acceleration =
            TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());

        // Setup motors
        let right = robot.motor(MotorId::DriveRight);
        let left = robot.motor(MotorId::DriveLeft);

        right.motor_reset(Some(StopAction::Hold))?;
        left.motor_reset(Some(StopAction::Hold))?;

        // Record the start time for acceleration
        let start = Instant::now();

        while position(&[right, left])? < distance {
            // Get position on acceleration curve
            let speed = acceleration.get_speed(start.elapsed().as_secs_f32()).0;

            let (speed_left, speed_right) = turn_split(&turn, speed, spec);

            right.command(Command::Queue(
                Command::To((dist_right * sign) as i32, speed_right as i32).into(),
            ))?;
            left.command(Command::Queue(
                Command::To((dist_left * sign) as i32, speed_left as i32).into(),
            ))?;

            right.command(Command::Execute)?;
            left.command(Command::Execute)?;

            robot.handle_interrupt()?;

            // TODO tune
            thread::sleep(Duration::from_millis(10))
        }

        right.command(Command::Stop(StopAction::Hold))?;
        left.command(Command::Stop(StopAction::Hold))?;

        right.wait()?;
        left.wait()?;

        Ok(())
    }
}

fn position(motors: &[&dyn Motor]) -> Result<f32> {
    debug_assert!(!motors.is_empty());

    let mut sum = 0.0;
    let mut count = 0.0;

    for motor in motors {
        let left_pos = motor.motor_angle()?.abs();

        sum += left_pos as f32;
        count += 1.0;
    }

    Ok(sum / count)
}

fn turn_split(turn_type: &TurnType, val: f32, spec: &RobotSpec) -> (f32, f32) {
    match turn_type {
        TurnType::Right => (0.0, val * spec.error_wheel_right()),
        TurnType::Left => (val * spec.error_wheel_left(), 0.0),
        TurnType::Center => (
            val * spec.error_wheel_left() / 2.0,
            val * spec.error_wheel_right() / 2.0,
        ),
    }
}
