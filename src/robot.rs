use std::time::Duration;
use anyhow::Result;

/// How the robot should turn
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
pub enum TurnType {
    /// Centered on right wheel
    Right,

    /// Centered on left wheel
    Left,

    /// Turn using both wheels
    Center
}

impl TurnType {
    pub fn wheels(&self) -> (bool, bool) {
        match self {
            TurnType::Right => {
                (false, true)
            }
            TurnType::Left => {
                (true, false)
            }
            TurnType::Center => {
                (true, true)
            }
        }
    }
}

/// Identifies a motor
#[derive(Eq, PartialEq, Copy, Clone, Hash, Debug)]
pub enum Motor {
    DriveRight,
    DriveLeft,
    AttachmentRight,
    AttachmentLeft
}
// todo support set-points
/// A motor movement
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum Command {
    /// Begin spinning forever
    /// Takes speed param
    On(i32),

    /// Stop spinning
    Stop(StopAction),

    /// Spin for a set distance (relative)
    /// Takes distance and speed params
    Distance(i32, i32),

    /// Spin to a set angle (absolute)
    /// Takes distance and speed params
    To(i32, i32),

    /// Spin for a set time
    /// Takes time (seconds) and speed params
    Time(Duration, i32),

    /// Sets the motor's target speed
    /// Useful for algorithms that need to dynamically adjust the motors speed
    /// Takes duty cycle param (-100 - 100 percent power)
    Direct(i32),

    /// Queues a command to be ran in the future
    /// Queuing a command could allow it to be executed faster
    /// This could minimize "kick" when using 2 motors
    Queue(Box<Command>),
    
    /// Executes the queued command
    Execute
}

/// A Stop Action
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum StopAction {
    /// Freely coast to a stop
    Coast,
    /// Cause motors to stop more quickly than `Coast` but without holding its position
    Break,
    /// Actively holds a motor's position
    Hold
}

/// Represents a simple robot with 2 wheels to move and a method to sense direction
pub trait Robot {
    /// Retrieve the robot's current heading.
    fn facing(&self) -> Result<f32>;

    /// Drive the robot straight
    ///
    /// # Returns
    ///
    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    ///
    /// # Panics
    ///
    /// This function may panic if `distance` or `speed` equal 0
    fn drive(&self, distance: i32, speed: i32) -> Result<()>;

    /// Turns the robot
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
    fn turn(&self, angle: i32, speed: i32) -> Result<()>;

    /// Turns the robot
    /// Uses specified turn type
    fn turn_named(&self, angle: i32, speed: i32, turn: TurnType) -> Result<()>;

    /// Moves a motor
    fn motor(&self, motor: Motor, movement: Command) -> Result<()>;

    /// Waits until a motor is finished moving
    fn wait(&self, motor: Motor) -> Result<()>;

    /// Retrieves the speed of a motor
    fn speed(&self, motor: Motor) -> Result<i32>;

    /// Retrieves the angle of a motor
    fn motor_angle(&self, motor: Motor) -> Result<i32>;

    /// Resets the angle of a motor to 0
    /// this method sets the stopping action for implicit stops
    fn motor_reset(&self, motor: Motor, stopping_action: Option<StopAction>) -> Result<()>;

    /// Resets the robot
    /// Should panic if called during a mission
    // Todo Should this even be exposed here?
    fn reset(&self) -> Result<()>;

    /// Retrieves the battery percentage
    /// Ranges from 0.0 -> 1.0
    fn battery(&self) -> Result<f32>;

    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    fn handle_interrupt(&self) -> Result<()>;
}