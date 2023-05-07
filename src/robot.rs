use anyhow::{bail, Context};
use ev3dev_lang_rust::motors::TachoMotor;
use ev3dev_lang_rust::sensors::SensorPort;

use crate::error::Result;
use crate::input::Input;
use crate::movement::spec::RobotSpec;
use crate::types::{Distance, Heading, Percent, Speed};
use std::cell::RefMut;
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
    Attachment1,
    Attachment2,
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
    /// Takes time and speed params
    Time(Duration, Speed),

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

impl StopAction {
    pub fn to_str(&self) -> &'static str {
        match self {
            StopAction::Coast => TachoMotor::STOP_ACTION_COAST,
            StopAction::Break => TachoMotor::STOP_ACTION_BRAKE,
            StopAction::Hold => TachoMotor::STOP_ACTION_HOLD,
        }
    }
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
    fn drive(&self, distance: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()>;

    /// Turns the robot using the gyro sensor
    /// Guesses the turn type from the turn direction
    ///
    /// # Inputs
    ///
    /// `speed` should be greater than `0.0`
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
    /// # Inputs
    ///
    /// `speed` should be greater than `0.0`
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

    fn set_heading(&self, angle: Heading) -> Result<()>;

    /// Retereives a motor
    fn motor(&self, motor: MotorId) -> Option<RefMut<dyn Motor>>;

    fn color_sensor(&self, port: SensorPort) -> Option<RefMut<dyn ColorSensor>>;

    /// Returns the status of the robot's buttons
    fn process_buttons(&self) -> Result<Input>;

    /// Returns an error if an interrupt has been requested
    /// Otherwise, returns Ok(())
    fn handle_interrupt(&self) -> Result<()> {
        let input = self.process_buttons().context("Process buttons")?;

        if input.is_left() {
            bail!("Interupt requested")
        }

        Ok(())
    }

    /// Retrieves the battery percentage
    /// Ranges from 0.0 -> 1.0
    fn battery(&self) -> Result<Percent>;

    /// Waits for the status of the robot's buttons and returns that new status
    fn await_input(&self, timeout: Option<Duration>) -> Result<Input>;

    /// Stops the robot's motors
    fn stop(&self) -> Result<()>;

    /// Resets the robot's state
    fn reset(&self) -> Result<()>;

    /// Returns information about the robot
    fn spec(&self) -> &RobotSpec;
}

pub trait ColorSensor {
    /// Retrieves the light reflected into the color sensor
    fn reflected_light(&self) -> Result<Percent>;

    /// Sets the white (1.0) definition
    fn cal_white(&mut self) -> Result<()>;

    /// Sets the black (0.0) definition
    fn cal_black(&mut self) -> Result<()>;

    /// Resets calibration data
    fn reset(&mut self) -> Result<()>;
}

pub trait Motor {
    /// Start a motor movement
    fn raw(&mut self, command: Command) -> Result<()>;

    /// Resets the angle of a motor to 0
    /// this method sets the stopping action for implicit stops
    fn motor_reset(&mut self, stop_action: Option<StopAction>) -> Result<()>;

    /// Waits until a motor is finished moving
    fn wait(&self, timeout: Option<Duration>) -> Result<()>;

    /// Retrieves the speed of a motor
    fn speed(&self) -> Result<Speed>;

    /// Retrieves the angle of a motor in distance from the last reset
    fn motor_angle(&self) -> Result<Distance>;
}

pub trait MotorExt {
    /// Spin for a set distance (relative)
    fn dist(&mut self, dist: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()>;

    /// Spin to a set angle (absolute)
    ///
    /// Position is in distance from last reset
    fn to_pos(&mut self, position: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()>;

    /// Spin for a set time
    fn time(&mut self, duration: Duration, speed: impl Into<Speed>) -> Result<()>;
}

impl MotorExt for dyn Motor + '_ {
    fn dist(&mut self, dist: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::Distance(dist.into(), speed.into()))?;

        self.wait(None)
    }

    fn to_pos(&mut self, position: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::To(position.into(), speed.into()))?;

        self.wait(None)
    }

    fn time(&mut self, duration: Duration, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::Time(duration, speed.into()))?;

        self.wait(None)
    }
}

impl<M: Motor> MotorExt for M {
    fn dist(&mut self, dist: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::Distance(dist.into(), speed.into()))?;

        self.wait(None)
    }

    fn to_pos(&mut self, position: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::To(position.into(), speed.into()))?;

        self.wait(None)
    }

    fn time(&mut self, duration: Duration, speed: impl Into<Speed>) -> Result<()> {
        self.raw(Command::Time(duration, speed.into()))?;

        self.wait(None)
    }
}

pub trait AngleProvider {
    /// Retrieves the robot's current heading
    fn angle(&self) -> Result<Heading>;
}
