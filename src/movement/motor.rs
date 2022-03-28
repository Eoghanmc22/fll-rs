use std::f32::consts::PI;
use std::thread;
use std::time::{Duration, Instant};
use crate::math;
use crate::movement::acceleration::{AccelerationPhase, TrapezoidalAcceleration};
use crate::movement::pid::{PidConfig, PidController};
use crate::robot::{Command, Motor, Robot, RobotError, StopAction, TurnType};

/// The standard implementation of movement
pub struct MovementController {
    pid_config: PidConfig,

    target_direction: f32,
    spec: RobotSpec
}

impl<> MovementController<> {
    pub fn new(pid_config: PidConfig, spec: RobotSpec) -> Self {
        MovementController {
            pid_config,
            target_direction: 0.0,
            spec
        }
    }

    pub fn drive(&mut self, robot: &dyn Robot, distance: i32, speed: i32) -> Result<(), RobotError> {
        let spec = &self.spec;
        let current_max_speed = spec.max_speed() * robot.battery() - 100.0;

        let sign = (distance.signum() * speed.signum()) as f32 * spec.gear_ratio().signum();
        let distance = distance.abs() as f32;
        let speed = math::clampf(speed.abs() as f32, current_max_speed, -current_max_speed);

        // Would take infinite or zero time to complete
        assert_ne!(sign, 0.0, "Illegal parameters!");

        // Generate the acceleration curve
        let acceleration = TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());
        // Setup the PID controller
        let mut pid = PidController::new(self.pid_config);

        // Setup motors
        robot.motor_reset(Motor::DriveRight, None);
        robot.motor_reset(Motor::DriveLeft, None);

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
                let speed_right = math::clampf(speed_right, current_max_speed, -current_max_speed);

                let speed_left = speed + speed * correction / 50.0;
                let speed_left = math::clampf(speed_left, current_max_speed, -current_max_speed);

                (speed_left, speed_right)
            };

            robot.motor(Motor::DriveRight, Command::Direct((speed_right * sign) as i32), false);
            robot.motor(Motor::DriveLeft, Command::Direct((speed_left * sign) as i32), false);

            robot.handle_interrupt()?;

            // TODO tune
            thread::sleep(Duration::from_millis(10))
        }

        robot.motor(Motor::DriveRight, Command::Stop(StopAction::Hold), false);
        robot.motor(Motor::DriveLeft, Command::Stop(StopAction::Hold), false);

        Ok(())
    }

    pub fn turn(&self, robot: &dyn Robot, angle: f32, speed: i32) -> Result<(), RobotError> {
        let difference = math::subtract_angles(angle, self.target_direction);

        let turn_type = if difference < 0.0 {
            TurnType::Right
        } else {
            TurnType::Left
        };

        self.turn_named(robot, angle, speed, turn_type)
    }

    // Has a lot in common with drive, could they be merged?
    pub fn turn_named(&self, robot: &dyn Robot, angle: f32, speed: i32, turn: TurnType) -> Result<(), RobotError> {
        assert!(speed > 0, "Speed must be greater than 0");

        let spec = &self.spec;
        let current_max_speed = spec.max_speed() * robot.battery() - 100.0;

        // Calculate how much the wheels need to move for this turn
        // Abs angle -> Rel angle -> distance -> left and right degrees
        let difference = math::subtract_angles(angle, robot.facing());
        let distance = spec.get_distance_for_turn(difference);
        let (dist_left, dist_right) = turn_split(&turn, distance, spec);

        // No turn is necessary
        if difference.abs() < 3.0 {
            return Ok(());
        }

        let sign = distance.signum() * spec.gear_ratio().signum();
        let distance = distance.abs();
        let speed = math::clampf(speed as f32, current_max_speed, -current_max_speed);

        // Generate the acceleration curve
        let acceleration = TrapezoidalAcceleration::new(distance, speed, spec.acceleration(), spec.deceleration());

        // Setup motors
        robot.motor_reset(Motor::DriveRight, Some(StopAction::Hold));
        robot.motor_reset(Motor::DriveLeft, Some(StopAction::Hold));

        // Record the start time for acceleration
        let start = Instant::now();

        while position(robot, turn.wheels()) < distance {
            // Get position on acceleration curve
            let (speed, phase) = acceleration.now(start);

            // run_abs is more reliable position wise
            // However run_direct is more reliable speed wise
            match phase {
                AccelerationPhase::Stopping |
                AccelerationPhase::Illegal => {
                    break;
                }
                _ => {}
            }

            let (speed_left, speed_right) = turn_split(&turn, speed, spec);

            // Move the length of the acceleration curve using run_direct for less lateral movement
            if dist_right != 0.0 && speed_right != 0.0 { robot.motor(Motor::DriveRight, Command::Direct((speed_right * sign) as i32), false); }
            if dist_left != 0.0 && speed_left != 0.0 { robot.motor(Motor::DriveLeft, Command::Direct((speed_left * sign) as i32), false); }

            robot.handle_interrupt()?;

            // TODO tune
            thread::sleep(Duration::from_millis(10))
        }

        let (speed_left, speed_right) = turn_split(&turn, TrapezoidalAcceleration::min_speed(), spec);

        // Move the length of the stopping phase using run_abs for better direction accuracy
        if dist_right != 0.0 && speed_right != 0.0 { robot.motor(Motor::DriveRight, Command::Distance(dist_right as i32, speed_right as i32), false); }
        if dist_left != 0.0 && speed_left != 0.0 { robot.motor(Motor::DriveLeft, Command::Distance(dist_left as i32, speed_left as i32), false); }

        robot.wait(Motor::DriveRight);
        robot.wait(Motor::DriveLeft);

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
            (0.0, val * spec.error_wheel_right)
        }
        TurnType::Left => {
            (val * spec.error_wheel_left, 0.0)
        }
        TurnType::Center => {
            (val * spec.error_wheel_left / 2.0, val * spec.error_wheel_right / 2.0)
        }
    }
}

pub struct RobotSpec {
    acceleration: f32,
    deceleration: f32,

    gear_ratio: f32,
    wheel_circumference: f32,
    wheel_right_circumference: f32,
    wheel_left_circumference: f32,
    wheelbase_circumference: f32,
    wheelbase_diameter: f32,

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
            wheel_right_circumference,
            wheel_left_circumference,
            wheelbase_circumference,
            wheelbase_diameter,
            error_wheelbase,
            error_wheel,
            error_wheel_right,
            error_wheel_left,
            max_speed
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

    pub fn get_distance_for_turn(&self, angle: f32) -> f32 {
        angle * self.wheelbase_circumference / self.wheel_circumference / self.gear_ratio
    }

    pub fn get_approx_angle(&self, left: f32, right: f32) -> f32 {
        (left * self.wheel_left_circumference - right * self.wheel_right_circumference) * self.gear_ratio / self.wheelbase_circumference
    }

    pub fn deg_to_mm(&self, deg: f32) -> f32 {
        deg / 360.0 * self.wheel_circumference * self.gear_ratio
    }

    pub fn mm_to_deg(&self, mm: f32) -> f32 {
        mm / self.gear_ratio / self.wheel_circumference * 360.0
    }
}