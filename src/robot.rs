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

/// Identifies a motor
#[derive(Eq, PartialEq, Copy, Clone, Hash, Debug)]
pub enum Motor {
    DriveRight,
    DriveLeft,
    AttachmentRight,
    AttachmentLeft,
}

/// A motor movement
#[derive(Copy, Clone, Debug)]
pub enum Command {
    /// Begin spinning forever
    /// Takes speed param
    On(i32),

    /// Stop spinning
    Off,

    /// Spin for a set distance
    /// Takes distance and speed params
    Distance(i32, i32),

    /// Spin for a set time
    /// Takes time (seconds) and speed params
    Time(f32, i32)
}

/// Represents a simple robot with 2 wheels to move and a method to sense direction
pub trait Robot {
    /// Retrieve the robot's current heading.
    fn facing(&self) -> f32;

    /// Drive the robot straight
    fn drive(&self, distance: i32, speed: i32);

    /// Turns the robot
    /// Guesses the turn type from the new target angle
    fn turn(&self, angle: f32, speed: i32);

    /// Turns the robot
    /// Uses specified turn type
    fn turn_named(&self, angle: f32, speed: i32, turn: TurnType);

    /// Moves a motor
    fn motor(&self, motor: Motor, movement: Command, wait: bool);

    /// Waits until a motor is finished moving
    fn wait(&self, motor: Motor);

    /// Retrieves the speed of a motor
    fn speed(&self, motor: Motor) -> i32;

    /// Retrieves the angle of a motor
    fn motor_angle(&self, motor: Motor, actual: bool) -> i32;

    /// Resets the angle of a motor to 0
    fn motor_reset(&self, motor: Motor) -> i32;

    /// Resets the robot
    /// Should panic if called during a mission
    fn reset(&self);
}