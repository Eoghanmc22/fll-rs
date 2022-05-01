use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use std::thread;
use std::time::Duration;
use ev3dev_lang_rust::motors::{MotorPort, TachoMotor};
use ev3dev_lang_rust::{Ev3Button, PowerSupply};
use ev3dev_lang_rust::sensors::SensorPort;
use ev3dev_lang_rust::sensors::GyroSensor as Ev3GyroSensor;
use ev3dev_lang_rust::sensors::ColorSensor as Ev3ColorSensor;
use crate::movement::controller::MovementController;
use crate::robot::{AngleProvider, ColorSensor, Command, DualColorSensor, Motor, Robot, StopAction, TurnType};
use anyhow::{bail, Context, Result};
use crate::error::Ev3ErrorWrapper;
use crate::movement::pid::PidConfig;
use crate::movement::spec::RobotSpec;

pub struct LegoRobot<Gyro: GyroSensorType, Color: ColorSensorType> {
    gyro_sensor: Gyro,
    color_sensor: Color,

    battery: PowerSupply,
    // todo
    buttons: Ev3Button,

    controller: MovementController,

    motors: HashMap<Motor, (TachoMotor, RefCell<MotorHandler>)>,

    spec: Rc<RobotSpec>,
}

#[derive(Default)]
struct MotorHandler {
    queued_command: Option<Command>,

    stopping_action: Option<StopAction>,

    speed_sp: Option<i32>,
    duty_cycle_sp: Option<i32>,
    time_sp: Option<Duration>,
    position_sp: Option<i32>,

    direct : bool,
}

impl<Gyro: GyroSensorType + Default, Color: ColorSensorType + Default> LegoRobot<Gyro, Color> {
    pub fn new(motor_definitions: &[(Motor, MotorPort)], pid_config: PidConfig, spec: RobotSpec) -> Result<Self> {
        let gyro_sensor = Default::default();
        let color_sensor = Default::default();

        let battery = PowerSupply::new().map_err(Ev3ErrorWrapper).context("Couldn't find power supply")?;
        let buttons = Ev3Button::new().map_err(Ev3ErrorWrapper).context("Couldn't find buttons")?;

        let controller = MovementController::new(pid_config);

        let mut motors = HashMap::new();

        for (motor_id, port) in motor_definitions {
            let motor = TachoMotor::get(*port).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't find motor {:?}, port {:?}", motor_id, port))?;
            let handler = RefCell::new(MotorHandler::default());

            motors.insert(*motor_id, (motor, handler));
        }

        let spec = Rc::new(spec);

        Ok(Self {
            gyro_sensor,
            color_sensor,
            battery,
            buttons,
            controller,
            motors,
            spec
        })
    }
}

impl<Gyro: GyroSensorType + Default> LegoRobot<Gyro, HasDualColorSensor> {
    pub fn new_dual_color_sensor(left_color_sensor: SensorPort, right_color_sensor: SensorPort, motor_definitions: &[(Motor, MotorPort)], pid_config: PidConfig, spec: RobotSpec) -> Result<Self> {
        let gyro_sensor = Default::default();
        let color_sensor = HasDualColorSensor {
            left: Rc::new(HasColorSensor {
                port: Some(left_color_sensor),
                ..Default::default()
            }),
            right: Rc::new(HasColorSensor {
                port: Some(right_color_sensor),
                ..Default::default()
            })
        };

        let battery = PowerSupply::new().map_err(Ev3ErrorWrapper).context("Couldn't find power supply")?;
        let buttons = Ev3Button::new().map_err(Ev3ErrorWrapper).context("Couldn't find buttons")?;

        let controller = MovementController::new(pid_config);

        let mut motors = HashMap::new();

        for (motor_id, port) in motor_definitions {
            let motor = TachoMotor::get(*port).map_err(Ev3ErrorWrapper).with_context(|| format!("Couldn't find motor {:?}, port {:?}", motor_id, port))?;
            let handler = RefCell::new(MotorHandler::default());

            motors.insert(*motor_id, (motor, handler));
        }

        let spec = Rc::new(spec);

        Ok(Self {
            gyro_sensor,
            color_sensor,
            battery,
            buttons,
            controller,
            motors,
            spec
        })
    }
}

// LegoRobot always implements AngleProvider but the compiler doesnt know that
impl<Gyro: GyroSensorType, Color: ColorSensorType> LegoRobot<Gyro, Color> where LegoRobot<Gyro, Color>: AngleProvider {
    pub fn reset(&self) -> Result<()> {
        for motor in self.motors.keys() {
            self.motor_reset(*motor, None).with_context(|| format!("Couldn't reset motor {:?}", motor))?;
        }

        // todo how to reset gyro
        // todo should this panic in a mission?

        Ok(())
    }

    // todo setters for pid, etc
}

// LegoRobot always implements AngleProvider but the compiler doesnt know that
impl<Gyro: GyroSensorType, Color: ColorSensorType> Robot for LegoRobot<Gyro, Color> where LegoRobot<Gyro, Color>: AngleProvider {
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

    fn battery(&self) -> Result<f32> {
        // todo should this adjust for min voltage???
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

    fn spec(&self) -> Rc<RobotSpec> {
        self.spec.clone()
    }
}

fn stop_action_name(stopping_action: StopAction) -> &'static str {
    match stopping_action {
        StopAction::Coast => { TachoMotor::STOP_ACTION_COAST }
        StopAction::Break => { TachoMotor::STOP_ACTION_BRAKE }
        StopAction::Hold => { TachoMotor::STOP_ACTION_HOLD }
    }
}


// todo better naming
#[derive(Default)]
struct HasGyroSensor {
    sensor: RefCell<Option<Ev3GyroSensor>>,
    port: Option<SensorPort>,
}

#[derive(Default)]
struct NoGyroSensor;

pub trait GyroSensorType {}
impl GyroSensorType for HasGyroSensor {}
impl GyroSensorType for NoGyroSensor {}

impl<C: ColorSensorType> AngleProvider for LegoRobot<NoGyroSensor, C> {}

impl<C: ColorSensorType> AngleProvider for LegoRobot<HasGyroSensor, C> {
    fn angle(&self) -> Result<f32> {
        fn create_gyro_sensor(port: Option<SensorPort>) -> Result<Ev3GyroSensor> {
            let gyro = if let Some(port) = port {
                Ev3GyroSensor::get(port).map_err(Ev3ErrorWrapper).with_context(|| format!("No gyro sensor on port {:?}", port))?
            } else {
                Ev3GyroSensor::find().map_err(Ev3ErrorWrapper).context("No gyro sensor connected")?
            };

            gyro.set_mode_gyro_ang().map_err(Ev3ErrorWrapper).context("Couldn't set gyro mode")?;

            // todo is this needed?
            while !gyro.is_mode_gyro_ang().map_err(Ev3ErrorWrapper).context("Couldn't check gyro mode")? {
                thread::sleep(Duration::from_millis(2));
            }

            Ok(gyro)
        }

        let cached_sensor = &mut *self.gyro_sensor.sensor.borrow_mut();

        if let Some(inner) = cached_sensor {
            if let Ok(angle) = inner.get_angle() {
                return Ok(angle as f32)
            }
        }

        let gyro = create_gyro_sensor(self.gyro_sensor.port)?;
        let angle = gyro.get_angle().map_err(Ev3ErrorWrapper).context("Couldn't read gyro")?;

        *cached_sensor = Some(gyro);

        Ok(angle as f32)
    }
}


struct HasDualColorSensor { left: Rc<HasColorSensor>, right: Rc<HasColorSensor> }

#[derive(Default)]
struct HasColorSensor {
    sensor: RefCell<Option<Ev3ColorSensor>>,
    port: Option<SensorPort>,

    white: RefCell<f32>,
    black: RefCell<f32>,
}

#[derive(Default)]
struct NoColorSensor;

pub trait ColorSensorType {}
impl ColorSensorType for NoColorSensor {}
impl ColorSensorType for HasColorSensor {}
impl ColorSensorType for HasDualColorSensor {}

impl ColorSensor for HasColorSensor {
    fn reflected_light(&self) -> Result<f32> {
        fn create_color_sensor(port: Option<SensorPort>) -> Result<Ev3ColorSensor> {
            let color_sensor = if let Some(port) = port {
                Ev3ColorSensor::get(port).map_err(Ev3ErrorWrapper).with_context(|| format!("No color sensor on port {:?}", port))?
            } else {
                Ev3ColorSensor::find().map_err(Ev3ErrorWrapper).context("No color sensor connected")?
            };

            color_sensor.set_mode_col_reflect().map_err(Ev3ErrorWrapper).context("Couldn't set color sensor mode")?;

            // todo is this needed?
            while !color_sensor.is_mode_col_color().map_err(Ev3ErrorWrapper).context("Couldn't check color sensor mode")? {
                thread::sleep(Duration::from_millis(2));
            }

            Ok(color_sensor)
        }

        let cached_sensor = &mut *self.sensor.borrow_mut();

        if let Some(inner) = cached_sensor {
            if let Ok(reflected_light) = inner.get_color() {
                return Ok(reflected_light as f32)
            }
        }

        let color_sensor = create_color_sensor(self.port)?;
        let angle = color_sensor.get_color().map_err(Ev3ErrorWrapper).context("Couldn't read color sensor")?;

        *cached_sensor = Some(color_sensor);

        Ok(angle as f32)
    }

    fn cal_white(&self) -> Result<()> {
        *self.white.borrow_mut() = self.reflected_light()?;

        Ok(())
    }

    fn cal_black(&self) -> Result<()> {
        *self.black.borrow_mut() = self.reflected_light()?;

        Ok(())
    }

    fn follow_line(&self, distance: i32, speed: i32) -> Result<()> {
        todo!()
    }
}

impl<G: GyroSensorType> ColorSensor for LegoRobot<G, HasColorSensor> {
    fn reflected_light(&self) -> Result<f32> {
        self.color_sensor.reflected_light()
    }

    fn cal_white(&self) -> Result<()> {
        self.color_sensor.cal_white()
    }

    fn cal_black(&self) -> Result<()> {
        self.color_sensor.cal_black()
    }

    fn follow_line(&self, distance: i32, speed: i32) -> Result<()> {
        self.color_sensor.follow_line(distance, speed)
    }
}

impl<G: GyroSensorType> DualColorSensor<HasColorSensor> for LegoRobot<G, HasDualColorSensor> {
    fn color_right(&self) -> Rc<HasColorSensor> {
        self.color_sensor.right.clone()
    }

    fn color_left(&self) -> Rc<HasColorSensor> {
        self.color_sensor.left.clone()
    }

    fn align(&self, max_distance: i32, speed: i32) -> Result<()> {
        todo!()
    }
}