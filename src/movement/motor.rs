use std::collections::HashMap;
use crate::movement::pid::PidConfig;
use crate::robot::{Motor, Robot};

/// The standard implementation of movement
pub struct MovementController<'a> {
    robot: &'a mut dyn Robot,
    tracker: &'a mut MotorTracker,
    
    pid: PidConfig
}

impl<'a> MovementController<'a> {
    pub fn new(robot: &'a mut dyn Robot, tracker: &'a mut MotorTracker, pid: PidConfig) -> Self {
        MovementController { robot, tracker, pid }
    }
    
    
}

/// A system to track motor offsets
pub struct MotorTracker {
    motors: HashMap<Motor, MotorData>
}

impl MotorTracker {
    pub fn new() -> Self {
        MotorTracker { motors: HashMap::new() }
    }

    /// Get a motor's offset
    pub fn motor_offset(&mut self, motor: Motor) -> i32 {
        let motor_data = self.motors.get(&motor).cloned().unwrap_or_default();

        motor_data.offset
    }

    /// Get a motor's angle
    pub fn motor_angle(&mut self, robot: &dyn Robot, motor: Motor) -> i32 {
        let motor_angle = robot.motor_angle(motor, true);
        
        motor_angle + self.motor_offset(motor)
    }

    /// Virtually zero a motor
    pub fn motor_zero(&mut self, robot: &dyn Robot, motor: Motor) {
        let motor_angle = robot.motor_angle(motor, true);
        
        self.motors.insert(motor, MotorData { offset: -motor_angle });
    }
    
    /// Clear a motor's offset
    pub fn motor_clear(&mut self, motor: Motor) {
        self.motors.remove(&motor);
    }
}

#[derive(Clone, Default)]
pub struct MotorData {
    offset: i32
}