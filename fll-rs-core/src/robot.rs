use crate::error::Result;
use crate::input::Input;
use crate::movement::spec::RobotSpec;
use crate::types::{Distance, Heading, Percent, Speed};
use std::time::Duration;

/// How the robot should turn
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum TurnType {
    /// Centered on right wheel
    Right,

    /// Centered on left wheel
    Left,

    /// Turn using both wheels
    Center,
}

impl TurnType {
    pub fn wheels(&self) -> (bool, bool) {
        match self {
            TurnType::Right => (false, true),
            TurnType::Left => (true, false),
            TurnType::Center => (true, true),
        }
    }
}

/// Identifies a motor
#[derive(Eq, PartialEq, Copy, Clone, Hash, Debug)]
pub enum MotorId {
    DriveRight,
    DriveLeft,
    AttachmentA,
    AttachmentD,
}
// todo support set-points
/// A motor movement
#[derive(Clone, Debug, PartialEq)]
pub enum Command {
    /// Begin spinning forever
    /// Takes speed param
    On(Speed),

    /// Stop spinning
    Stop(StopAction),

    /// Spin for a set distance (relative)
    /// Takes distance and speed params
    Distance(Distance, Speed),

    /// Spin to a set angle (absolute)
    /// Takes position and speed params
    ///
    /// Position is in distance from last reset
    To(Distance, Speed),

    /// Spin for a set time
    /// Takes time (seconds) and speed params
    Time(Duration, Speed),

    /// Sets the motor's target speed
    /// Useful for algorithms that need to dynamically adjust the motors speed
    /// Takes duty cycle param (-100 to 100 percent power)
    Direct(Percent),

    /// Queues a command to be ran when a `Execute` command is sent
    /// Queuing a command could allow it to be executed faster
    /// This could minimize "kick" when using 2 motors
    ///
    /// Not guranteed to be kept if additional commands are sent to this motor
    /// between this command and the next `Execute` command
    Queue(Box<Command>),

    /// Executes the queued command
    Execute,
}

/// A Stop Action
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StopAction {
    /// Freely coast to a stop
    Coast,
    /// Cause motors to stop more quickly than `Coast` but without holding its position
    Break,
    /// Actively holds a motor's position
    Hold,
}

/// Identifies a sensor
#[derive(Eq, PartialEq, Copy, Clone, Hash, Debug)]
pub enum SensorId {
    DriveLeft,
    AttachmentRight,
    AttachmentLeft,
}

/// Represents a simple robot with 2 wheels to move and a method to sense direction
pub trait Robot: AngleProvider {
    /// Drive the robot using the gyro to correct for errors
    ///
    /// # Returns
    ///
    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    ///
    /// # Panics
    ///
    /// This function may panic if `distance` or `speed` equal 0
    fn drive(&self, distance: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()>;

    /// Turns the robot using the gyro sensor
    /// Guesses the turn type from the turn direction
    ///
    /// # Returns
    ///
    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    ///
    /// # Panics
    ///
    /// This function may panic if `speed` is less than or equal to 0
    fn turn(&self, angle: Heading, speed: impl Into<Speed>) -> Result<()>;

    /// Turns the robot
    /// Uses specified turn type using the gyro sensor
    ///
    /// # Returns
    ///
    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    ///
    /// # Panics
    ///
    /// This function may panic if `speed` is less than or equal to 0
    fn turn_named(&self, angle: Heading, speed: impl Into<Speed>, turn: TurnType) -> Result<()>;

    /// Retereives a motor
    fn motor(&self, motor: MotorId) -> &dyn Motor;

    /// Retrieves the battery percentage
    /// Ranges from 0.0 -> 1.0
    fn battery(&self) -> Result<Percent>;

    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    fn handle_interrupt(&self) -> Result<()>;

    /// Returns the status of the robot's buttons
    fn process_buttons(&self) -> Result<Input>;

    /// Waits for the status of the robot's buttons and returns that new status
    fn await_input(&self) -> Result<Input>;

    /// Resets the robot's state
    fn reset(&mut self) -> Result<()>;

    /// Returns information about the robot
    fn spec(&self) -> &RobotSpec;
}

pub trait ColorSensor {
    /// Retrieves the light reflected into the color sensor
    fn reflected_light(&self) -> Result<Percent>;

    // TODO color getter

    /// Sets the white (1.0) definition
    fn cal_white(&self) -> Result<()>;

    /// Sets the black (0.0) definition
    fn cal_black(&self) -> Result<()>;
}

pub trait Motor {
    /// Start a motor movement
    fn command(&self, command: Command) -> Result<()>;

    /// Waits until a motor is finished moving
    fn wait(&self) -> Result<()>;

    /// Retrieves the speed of a motor
    fn speed(&self) -> Result<Speed>;

    /// Retrieves the angle of a motor in distance from the last reset
    fn motor_angle(&self) -> Result<Distance>;

    /// Resets the angle of a motor to 0
    /// this method sets the stopping action for implicit stops
    fn motor_reset(&self, stopping_action: Option<StopAction>) -> Result<()>;
}

pub trait AngleProvider {
    /// Retrieves the robot's current heading
    fn angle(&self) -> Result<Heading>;
}
