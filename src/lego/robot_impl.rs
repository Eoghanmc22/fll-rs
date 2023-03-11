use anyhow::{bail, Context};
use ev3dev_lang_rust::motors::{MotorPort, TachoMotor};
use ev3dev_lang_rust::sensors::ColorSensor as Ev3ColorSensor;
use ev3dev_lang_rust::sensors::GyroSensor as Ev3GyroSensor;
use ev3dev_lang_rust::sensors::SensorPort;
use ev3dev_lang_rust::{wait, Ev3Button, PowerSupply};
use fll_rs::error::Result;
use fll_rs::input::Input;
use fll_rs::movement::controller::MovementController;
use fll_rs::movement::pid::PidConfig;
use fll_rs::movement::spec::RobotSpec;
use fll_rs::robot::{
    AngleProvider, ColorSensor, Command, DualColorSensor, MotorId, Robot, StopAction, TurnType,
};
use std::cell::RefCell;
use std::collections::HashMap;
use std::fs::File;
use std::os::unix::io::AsRawFd;
use std::thread;
use std::time::Duration;

pub struct LegoRobot {
    battery: PowerSupply,

    buttons: Ev3Button,
    input: RefCell<Input>,
    input_raw: File,

    controller: MovementController,

    motors: HashMap<MotorId, (TachoMotor, RefCell<MotorHandler>)>,

    spec: RobotSpec,
}

#[derive(Default)]
struct MotorHandler {
    queued_command: Option<Command>,

    stopping_action: Option<StopAction>,

    speed_sp: Option<i32>,
    duty_cycle_sp: Option<i32>,
    time_sp: Option<Duration>,
    position_sp: Option<i32>,

    direct: bool,
}

impl LegoRobot {
    pub fn new(
        motor_definitions: &[(MotorId, MotorPort)],
        pid_config: PidConfig,
        spec: RobotSpec,
    ) -> Result<Self> {
        let gyro_sensor = Default::default();
        let color_sensor = Default::default();

        let battery = PowerSupply::new().context("Couldn't find power supply")?;

        let buttons = Ev3Button::new().context("Couldn't find buttons")?;
        let input = Default::default();
        let input_raw = File::open("/dev/input/by-path/platform-gpio_keys-event")?;

        let controller = MovementController::new(pid_config);

        let mut motors = HashMap::new();

        for (motor_id, port) in motor_definitions {
            let motor = TachoMotor::get(*port)
                .with_context(|| format!("Couldn't find motor {:?}, port {:?}", motor_id, port))?;
            let handler = RefCell::new(MotorHandler::default());

            motors.insert(*motor_id, (motor, handler));
        }

        Ok(Self {
            gyro_sensor,
            color_sensor,
            battery,
            buttons,
            input,
            input_raw,
            controller,
            motors,
            spec,
        })
    }
}

impl<Gyro: GyroSensorType + Default> LegoRobot<Gyro, HasDualColorSensor> {
    pub fn new_dual_color_sensor(
        left_color_sensor: SensorPort,
        right_color_sensor: SensorPort,
        motor_definitions: &[(MotorId, MotorPort)],
        pid_config: PidConfig,
        spec: RobotSpec,
    ) -> Result<Self> {
        let gyro_sensor = Default::default();
        let color_sensor = HasDualColorSensor {
            left: HasColorSensor {
                port: Some(left_color_sensor),
                ..Default::default()
            },
            right: HasColorSensor {
                port: Some(right_color_sensor),
                ..Default::default()
            },
            _private: Private,
        };

        let battery = PowerSupply::new().context("Couldn't find power supply")?;

        let buttons = Ev3Button::new().context("Couldn't find buttons")?;
        let input = Default::default();
        let wait_file = File::open("/dev/input/by-path/platform-gpio_keys-event")?;

        let controller = MovementController::new(pid_config);

        let mut motors = HashMap::new();

        for (motor_id, port) in motor_definitions {
            let motor = TachoMotor::get(*port)
                .with_context(|| format!("Couldn't find motor {:?}, port {:?}", motor_id, port))?;
            let handler = RefCell::new(MotorHandler::default());

            motors.insert(*motor_id, (motor, handler));
        }

        Ok(Self {
            gyro_sensor,
            color_sensor,
            battery,
            buttons,
            input,
            input_raw: wait_file,
            controller,
            motors,
            spec,
        })
    }
}

// LegoRobot always implements AngleProvider but the compiler doesnt know that
impl<Gyro: GyroSensorType, Color: ColorSensorType> LegoRobot<Gyro, Color>
where
    LegoRobot<Gyro, Color>: AngleProvider,
{
    pub fn reset(&self) -> Result<()> {
        for motor in self.motors.keys() {
            self.motor_reset(*motor, None)
                .with_context(|| format!("Couldn't reset motor {:?}", motor))?;
        }

        // todo how to reset gyro
        // todo should this panic in a mission?

        Ok(())
    }

    // todo setters for pid, etc
}

// LegoRobot always implements AngleProvider but the compiler doesnt know that
impl<Gyro: GyroSensorType, Color: ColorSensorType> Robot for LegoRobot<Gyro, Color>
where
    LegoRobot<Gyro, Color>: AngleProvider,
{
    fn drive(&self, distance: i32, speed: i32) -> Result<()> {
        self.controller.drive(self, distance, speed)
    }

    fn turn(&self, angle: i32, speed: i32) -> Result<()> {
        self.controller.turn(self, angle, speed)
    }

    fn turn_named(&self, angle: i32, speed: i32, turn: TurnType) -> Result<()> {
        self.controller.turn_named(self, angle, speed, turn)
    }

    fn motor(&self, motor_id: MotorId, cmd: Command) -> Result<()> {
        let (motor, motor_handler) = self
            .motors
            .get(&motor_id)
            .with_context(|| format!("No {:?} motor", motor_id))?;
        let motor_handler = &mut *motor_handler.borrow_mut();

        let (sp, run) = match cmd {
            Command::Queue(_) => (true, false),
            Command::Execute => (false, true),
            _ => {
                motor_handler.queued_command = None;
                (true, true)
            }
        };

        match cmd {
            Command::On(speed) => {
                if sp && Some(speed) != motor_handler.speed_sp {
                    motor.set_speed_sp(speed).with_context(|| {
                        format!("Couldn't write property, motor {:?}", motor_id)
                    })?;

                    motor_handler.speed_sp = Some(speed);
                }
                if run {
                    motor
                        .run_forever()
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Stop(stopping_action) => {
                if sp && Some(stopping_action) != motor_handler.stopping_action {
                    let name = stop_action_name(stopping_action);

                    motor.set_stop_action(name).with_context(|| {
                        format!("Couldn't write property, motor {:?}", motor_id)
                    })?;
                }
                if run {
                    motor
                        .stop()
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Distance(distance, speed) => {
                if sp {
                    if Some(distance) != motor_handler.position_sp {
                        motor.set_position_sp(distance).with_context(|| {
                            format!("Couldn't write property, motor {:?}", motor_id)
                        })?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).with_context(|| {
                            format!("Couldn't write property, motor {:?}", motor_id)
                        })?;
                    }
                }
                if run {
                    motor
                        .run_to_rel_pos(None)
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::To(position, speed) => {
                if sp {
                    if Some(position) != motor_handler.position_sp {
                        motor.set_position_sp(position).with_context(|| {
                            format!("Couldn't write property, motor {:?}", motor_id)
                        })?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).with_context(|| {
                            format!("Couldn't write property, motor {:?}", motor_id)
                        })?;
                    }
                }
                if run {
                    motor
                        .run_to_abs_pos(None)
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Time(duration, speed) => {
                if sp {
                    if Some(duration) != motor_handler.time_sp {
                        motor
                            .set_time_sp(duration.as_millis() as i32)
                            .with_context(|| {
                                format!("Couldn't write property, motor {:?}", motor_id)
                            })?;
                    }

                    if Some(speed) != motor_handler.speed_sp {
                        motor.set_speed_sp(speed).with_context(|| {
                            format!("Couldn't write property, motor {:?}", motor_id)
                        })?;
                    }
                }
                if run {
                    motor
                        .run_timed(None)
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = false;
                }
            }
            Command::Direct(duty_cycle) => {
                if sp && Some(duty_cycle) != motor_handler.duty_cycle_sp {
                    motor.set_duty_cycle_sp(duty_cycle).with_context(|| {
                        format!("Couldn't write property, motor {:?}", motor_id)
                    })?;
                }

                if run && !motor_handler.direct {
                    motor
                        .run_direct()
                        .with_context(|| format!("Couldn't send command, motor {:?}", motor_id))?;

                    motor_handler.direct = true;
                }
            }
            _ => {}
        }

        Ok(())
    }

    fn wait(&self, motor_id: MotorId) -> Result<()> {
        let (motor, _) = self
            .motors
            .get(&motor_id)
            .with_context(|| format!("No {:?} motor", motor_id))?;

        motor.wait_until_not_moving(None);

        Ok(())
    }

    fn speed(&self, motor_id: MotorId) -> Result<i32> {
        let (motor, _) = self
            .motors
            .get(&motor_id)
            .with_context(|| format!("No {:?} motor", motor_id))?;

        motor
            .get_speed()
            .with_context(|| format!("Couldn't read property, motor {:?}", motor_id))
    }

    fn motor_angle(&self, motor_id: MotorId) -> Result<i32> {
        let (motor, _) = self
            .motors
            .get(&motor_id)
            .with_context(|| format!("No {:?} motor", motor_id))?;

        motor
            .get_position()
            .with_context(|| format!("Couldn't read property, motor {:?}", motor_id))
    }

    // Todo should this be a "virtual" reset or a real one?
    fn motor_reset(&self, motor_id: MotorId, stopping_action: Option<StopAction>) -> Result<()> {
        let (motor, _) = self
            .motors
            .get(&motor_id)
            .with_context(|| format!("No {:?} motor", motor_id))?;

        motor
            .reset()
            .with_context(|| format!("Couldn't reset motor {:?}", motor))?;

        if let Some(stopping_action) = stopping_action {
            motor
                .set_stop_action(stop_action_name(stopping_action))
                .with_context(|| format!("Couldn't write property, motor {:?}", motor))?;
        }

        Ok(())
    }

    fn battery(&self) -> Result<f32> {
        // todo should this adjust for min voltage???
        let max = self
            .battery
            .get_voltage_max_design()
            .context("Couldn't read battery")?;
        let now = self
            .battery
            .get_voltage_now()
            .context("Couldn't read battery")?;

        Ok(now as f32 / max as f32)
    }

    fn handle_interrupt(&self) -> Result<()> {
        self.buttons.process();

        if self.buttons.is_left() {
            bail!("Interrupt requested");
        }

        Ok(())
    }

    fn process_buttons(&self) -> Result<Input> {
        {
            self.buttons.process();

            let mut state = [false; 6];
            state[Input::UP] = self.buttons.is_up();
            state[Input::DOWN] = self.buttons.is_down();
            state[Input::LEFT] = self.buttons.is_left();
            state[Input::RIGHT] = self.buttons.is_right();
            state[Input::ENTER] = self.buttons.is_enter();
            state[Input::BACKSPACE] = self.buttons.is_backspace();

            self.input.borrow_mut().update(state);
        }

        Ok(self.input.borrow().clone())
    }

    fn await_input(&self) -> Result<Input> {
        wait::wait(self.input_raw.as_raw_fd(), || true, None);

        self.process_buttons()
    }

    fn reset(&mut self) -> Result<()> {
        todo!()
    }

    fn spec(&self) -> &RobotSpec {
        &self.spec
    }
}

fn stop_action_name(stopping_action: StopAction) -> &'static str {
    match stopping_action {
        StopAction::Coast => TachoMotor::STOP_ACTION_COAST,
        StopAction::Break => TachoMotor::STOP_ACTION_BRAKE,
        StopAction::Hold => TachoMotor::STOP_ACTION_HOLD,
    }
}

#[derive(Default)]
struct Private;

// todo better naming
#[derive(Default)]
pub struct HasGyroSensor {
    sensor: RefCell<Option<Ev3GyroSensor>>,
    port: Option<SensorPort>,

    _private: Private,
}

#[derive(Default)]
pub struct NoGyroSensor(Private);

pub trait GyroSensorType {}
impl GyroSensorType for HasGyroSensor {}
impl GyroSensorType for NoGyroSensor {}

impl<C: ColorSensorType> AngleProvider for LegoRobot<NoGyroSensor, C> {
    fn angle(&self) -> Result<f32> {
        let left = self.motor_angle(MotorId::DriveLeft)? as f32;
        let right = self.motor_angle(MotorId::DriveRight)? as f32;

        Ok(self.spec().get_approx_angle(left, right))
    }
}

impl<C: ColorSensorType> AngleProvider for LegoRobot<HasGyroSensor, C> {
    fn angle(&self) -> Result<f32> {
        fn create_gyro_sensor(port: Option<SensorPort>) -> Result<Ev3GyroSensor> {
            let gyro = if let Some(port) = port {
                Ev3GyroSensor::get(port)
                    .with_context(|| format!("No gyro sensor on port {:?}", port))?
            } else {
                Ev3GyroSensor::find().context("No gyro sensor connected")?
            };

            gyro.set_mode_gyro_ang().context("Couldn't set gyro mode")?;

            // todo is this needed?
            while !gyro
                .is_mode_gyro_ang()
                .context("Couldn't check gyro mode")?
            {
                thread::sleep(Duration::from_millis(2));
            }

            Ok(gyro)
        }

        let cached_sensor = &mut *self.gyro_sensor.sensor.borrow_mut();

        if let Some(inner) = cached_sensor {
            if let Ok(angle) = inner.get_angle() {
                return Ok(angle as f32);
            }
        }

        let gyro = create_gyro_sensor(self.gyro_sensor.port)?;
        let angle = gyro.get_angle().context("Couldn't read gyro")?;

        *cached_sensor = Some(gyro);

        Ok(angle as f32)
    }
}

pub struct HasDualColorSensor {
    left: HasColorSensor,
    right: HasColorSensor,
    _private: Private,
}

#[derive(Default)]
pub struct HasColorSensor {
    sensor: RefCell<Option<Ev3ColorSensor>>,
    port: Option<SensorPort>,

    white: RefCell<f32>,
    black: RefCell<f32>,

    _private: Private,
}

#[derive(Default)]
pub struct NoColorSensor(Private);

pub trait ColorSensorType {}
impl ColorSensorType for NoColorSensor {}
impl ColorSensorType for HasColorSensor {}
impl ColorSensorType for HasDualColorSensor {}

impl ColorSensor for HasColorSensor {
    fn reflected_light(&self) -> Result<f32> {
        fn create_color_sensor(port: Option<SensorPort>) -> Result<Ev3ColorSensor> {
            let color_sensor = if let Some(port) = port {
                Ev3ColorSensor::get(port)
                    .with_context(|| format!("No color sensor on port {:?}", port))?
            } else {
                Ev3ColorSensor::find().context("No color sensor connected")?
            };

            color_sensor
                .set_mode_col_reflect()
                .context("Couldn't set color sensor mode")?;

            // todo is this needed?
            while !color_sensor
                .is_mode_col_color()
                .context("Couldn't check color sensor mode")?
            {
                thread::sleep(Duration::from_millis(2));
            }

            Ok(color_sensor)
        }

        let cached_sensor = &mut *self.sensor.borrow_mut();

        if let Some(inner) = cached_sensor {
            if let Ok(reflected_light) = inner.get_color() {
                return Ok(reflected_light as f32);
            }
        }

        let color_sensor = create_color_sensor(self.port)?;
        let angle = color_sensor
            .get_color()
            .context("Couldn't read color sensor")?;

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

    fn follow_line(&self, _distance: i32, _speed: i32) -> Result<()> {
        unimplemented!()
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
        todo!()
    }
}

impl<G: GyroSensorType> DualColorSensor for LegoRobot<G, HasDualColorSensor> {
    type Sensor = HasColorSensor;

    fn color_right(&self) -> &Self::Sensor {
        &self.color_sensor.right
    }

    fn color_left(&self) -> &Self::Sensor {
        &self.color_sensor.left
    }

    fn align(&self, max_distance: i32, speed: i32) -> Result<()> {
        todo!()
    }

    fn follow_line_left(&self, distance: i32, speed: i32) -> Result<()> {
        todo!()
    }

    fn follow_line_right(&self, distance: i32, speed: i32) -> Result<()> {
        todo!()
    }
}
