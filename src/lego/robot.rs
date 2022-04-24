use ev3dev_lang_rust::motors::{LargeMotor, MediumMotor};
use ev3dev_lang_rust::{Ev3Button, PowerSupply};
use ev3dev_lang_rust::sensors::GyroSensor;
use crate::movement::controller::MovementController;
use crate::robot::{Command, Motor, Robot, Result, StopAction, TurnType};

pub struct LegoRobot {
    gyro_sensor: GyroSensor,
    battery: PowerSupply,
    buttons: Ev3Button,

    controller: MovementController,
    right_drive: LargeMotor,
    left_drive: LargeMotor,

    right_attachment: MediumMotor,
    left_attachment: MediumMotor,
}

impl Robot for LegoRobot {
    fn facing(&self) -> f32 {
        todo!()
    }

    fn drive(&self, distance: i32, speed: i32) -> Result<()> {
        todo!()
    }

    fn turn(&self, angle: i32, speed: i32) -> Result<()> {
        todo!()
    }

    fn turn_named(&self, angle: i32, speed: i32, turn: TurnType) -> Result<()> {
        todo!()
    }

    fn motor(&self, motor: Motor, movement: Command) {
        todo!()
    }

    fn wait(&self, motor: Motor) {
        todo!()
    }

    fn speed(&self, motor: Motor) -> i32 {
        todo!()
    }

    fn motor_angle(&self, motor: Motor) -> i32 {
        todo!()
    }

    fn motor_reset(&self, motor: Motor, stopping_action: Option<StopAction>) {
        todo!()
    }

    fn reset(&self) {
        todo!()
    }

    fn battery(&self) -> f32 {
        todo!()
    }

    fn handle_interrupt(&self) -> Result<()> {
        todo!()
    }
}