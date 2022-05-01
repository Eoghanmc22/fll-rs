use std::cell::RefCell;
use std::collections::HashMap;
use std::thread;
use std::time::Duration;
use ev3dev_lang_rust::motors::TachoMotor;
use ev3dev_lang_rust::{Ev3Button, PowerSupply};
use ev3dev_lang_rust::sensors::GyroSensor;
use crate::movement::controller::MovementController;
use crate::robot::{Command, Motor, Robot, StopAction, TurnType};
use anyhow::{bail, Context, Result};
use crate::error::Ev3ErrorWrapper;

struct MotorHandler {
    queued_command: Option<Command>,

    stopping_action: Option<StopAction>,

    speed_sp: Option<i32>,
    duty_cycle_sp: Option<i32>,
    time_sp: Option<Duration>,
    position_sp: Option<i32>,

    direct : bool,
}

pub struct LegoRobot {
    gyro_sensor: GyroSensor,
    battery: PowerSupply,
    // todo
    buttons: Ev3Button,

    controller: MovementController,

    motors: HashMap<Motor, (TachoMotor, RefCell<MotorHandler>)>,
}

impl Robot for LegoRobot {
    fn facing(&self) -> Result<f32> {
        if let Ok(angle) = self.gyro_sensor.get_angle() {
            Ok(angle as f32)
        } else {
            // todo log this event?
            fn try_fix_sensor(gyro_sensor: &GyroSensor) -> Result<i32> {
                gyro_sensor.set_mode_gyro_ang().map_err(Ev3ErrorWrapper)?;

                while !gyro_sensor.is_mode_gyro_ang().map_err(Ev3ErrorWrapper)? {
                    thread::sleep(Duration::from_millis(2));
                }

                gyro_sensor.get_angle().map_err(Ev3ErrorWrapper).context("Couldn't read gyro")
            }

            Ok(try_fix_sensor(&self.gyro_sensor)? as f32)
        }
    }

    fn drive(&self, distance: i32, speed: i32) -> Result<()> {
        self.controller.drive(self, distance, speed)
    }

    fn turn(&self, angle: i32, speed: i32) -> Result<()> {
        self.controller.turn(self, angle, speed)
    }

    fn turn_named(&self, angle: i32, speed: i32, turn: TurnType) -> Result<()> {
        self.controller.turn_named(self, angle, speed, turn)
    }

    fn motor(&self, motor_id: Motor, cmd: Command) -> Result<()> {
        let (motor, motor_handler) = self.motors.get(&motor_id).with_context(|| format!("No {:?} motor", motor_id))?;
        let motor_handler = &mut *motor_handler.borrow_mut();

        let (sp, run) = match cmd {
            Command::Queue(_) => { (true, false) }
            Command::Execute => { (false, true) }
            _ => {
                motor_handler.queued_command = None;
                (true, true)
            }
        };

        match cmd {
            Command::On(speed) => {
                if sp && Some(speed) != motor_handler.speed_sp {
                    motor.set_speed_sp(speed).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;

                    motor_handler.speed_sp = Some(speed);
                }
                if run {
                    motor.run_forever().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Stop(stopping_action) => {
                if sp && Some(stopping_action) != motor_handler.stopping_action {
                    let name = stop_action_name(stopping_action);

                    motor.set_stop_action(name).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                }
                if run {
                    motor.stop().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Distance(distance, speed) => {
                if sp {
                    if Some(distance) != motor_handler.position_sp {
                        motor.set_position_sp(distance).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }
                }
                if run {
                    motor.run_to_rel_pos(None).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::To(position, speed) => {
                if sp {
                    if Some(position) != motor_handler.position_sp {
                        motor.set_position_sp(position).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }
                }
                if run {
                    motor.run_to_abs_pos(None).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Time(duration, speed) => {
                if sp {
                    if Some(duration) != motor_handler.time_sp {
                        motor.set_time_sp(duration.as_millis() as i32).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                    }
                }
                if run {
                    motor.run_timed(None).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Direct(duty_cycle) => {
                if sp && Some(duty_cycle) != motor_handler.duty_cycle_sp {
                    motor.set_duty_cycle_sp(duty_cycle).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor_id))?;
                }

                if run && !motor_handler.direct {
                    motor.run_direct().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = true;
                }
            }
            _ => {}
        }

        Ok(())
    }

    fn wait(&self, motor_id: Motor) -> Result<()> {
        let (motor, _) = self.motors.get(&motor_id).with_context(|| format!("No {:?} motor", motor_id))?;

        motor.wait_until_not_moving(None);

        Ok(())
    }

    fn speed(&self, motor_id: Motor) -> Result<i32> {
        let (motor, _) = self.motors.get(&motor_id).with_context(|| format!("No {:?} motor", motor_id))?;

        motor.get_speed().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't read property, motor {:?}", motor_id))
    }

    fn motor_angle(&self, motor_id: Motor) -> Result<i32> {
        let (motor, _) = self.motors.get(&motor_id).with_context(|| format!("No {:?} motor", motor_id))?;

        motor.get_position().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't read property, motor {:?}", motor_id))
    }

    // Todo should this be a "virtual" reset or a real one?
    fn motor_reset(&self, motor_id: Motor, stopping_action: Option<StopAction>) -> Result<()> {
        let (motor, _) = self.motors.get(&motor_id).with_context(|| format!("No {:?} motor", motor_id))?;

        motor.reset().map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't reset motor {:?}", motor))?;

        if let Some(stopping_action) = stopping_action {
            motor.set_stop_action(stop_action_name(stopping_action)).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't write property, motor {:?}", motor))?;
        }

        Ok(())
    }

    // Todo should this be a "virtual" reset or a real one?
    fn reset(&self) -> Result<()> {
        for motor in self.motors.keys() {
            self.motor_reset(*motor, None).with_context(|| format!("Couldn't reset motor {:?}", motor))?;
        }

        //todo how to reset gyro
        //todo should this panic in a mission?

        Ok(())
    }

    fn battery(&self) -> Result<f32> {
        //todo should this adjust for min voltage???
        let max = self.battery.get_voltage_max_design().map_err(Ev3ErrorWrapper).context("Couldn't read battery")?;
        let now = self.battery.get_voltage_now().map_err(Ev3ErrorWrapper).context("Couldn't read battery")?;

        Ok(now as f32 / max as f32)
    }

    fn handle_interrupt(&self) -> Result<()> {
        self.buttons.process();

        if self.buttons.is_left() {
            bail!("Interrupt requested");
        }

        Ok(())
    }
}

fn stop_action_name(stopping_action: StopAction) -> &'static str {
    match stopping_action {
        StopAction::Coast => { TachoMotor::STOP_ACTION_COAST }
        StopAction::Break => { TachoMotor::STOP_ACTION_BRAKE }
        StopAction::Hold => { TachoMotor::STOP_ACTION_HOLD }
    }
}