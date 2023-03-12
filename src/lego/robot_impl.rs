use crate::error::Result;
use crate::input::Input;
use crate::movement::controller::MovementController;
use crate::movement::pid::PidConfig;
use crate::movement::spec::RobotSpec;
use crate::robot::{
    AngleProvider, ColorSensor, Command, Motor, MotorId, Robot, StopAction, TurnType,
};
use crate::types::{Degrees, DegreesPerSecond, Distance, Heading, Percent, Speed};
use anyhow::{bail, Context};
use ev3dev_lang_rust::motors::{MotorPort, TachoMotor as Ev3TachoMotor};
use ev3dev_lang_rust::sensors::ColorSensor as Ev3ColorSensor;
use ev3dev_lang_rust::sensors::GyroSensor as Ev3GyroSensor;
use ev3dev_lang_rust::sensors::SensorPort;
use ev3dev_lang_rust::Device;
use ev3dev_lang_rust::{wait, Ev3Button, Port, PowerSupply};
use fxhash::FxHashMap as HashMap;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::os::unix::io::AsRawFd;
use std::rc::Rc;
use std::time::Duration;

pub struct LegoRobot {
    battery: PowerSupply,

    buttons: Ev3Button,
    buttons_raw: File,
    input: RefCell<Input>,

    controller: MovementController,
    angle_algorithm: AngleAlgorithm,

    motors: HashMap<MotorId, RefCell<LegoMotor>>,
    sensors: HashMap<String, LegoSensor>,

    spec: Rc<RobotSpec>,
}

pub enum AngleAlgorithm {
    Wheel,
    Gyro(SensorPort),
}

enum LegoSensor {
    Gyro(RefCell<LegoGyroSensor>),
    Color(RefCell<LegoColorSensor>),
}

struct LegoMotor {
    motor: Ev3TachoMotor,

    queued_command: Option<Command>,

    stopping_action: Option<StopAction>,

    speed_sp: Option<i32>,
    time_sp: Option<Duration>,
    position_sp: Option<i32>,

    spec: Rc<RobotSpec>,
}

struct LegoGyroSensor {
    gyro: Ev3GyroSensor,
}

struct LegoColorSensor {
    color: Ev3ColorSensor,
    white: f32,
    black: f32,
}

impl LegoRobot {
    pub fn new(
        motor_definitions: &[(MotorId, MotorPort)],
        pid_config: PidConfig,
        spec: RobotSpec,
        angle_algorithm: AngleAlgorithm,
    ) -> Result<Self> {
        let battery = PowerSupply::new().context("Couldn't find power supply")?;

        let buttons = Ev3Button::new().context("Couldn't find buttons")?;
        let input = Default::default();
        let input_raw = File::open("/dev/input/by-path/platform-gpio_keys-event")?;

        let controller = MovementController::new(pid_config);

        let spec = Rc::new(spec);

        let mut motors = HashMap::default();
        for (motor_id, port) in motor_definitions {
            let motor = Ev3TachoMotor::get(*port)
                .with_context(|| format!("Couldn't find motor {:?}, port {:?}", motor_id, port))?;
            motors.insert(
                *motor_id,
                LegoMotor {
                    motor,
                    queued_command: None,
                    stopping_action: None,
                    speed_sp: None,
                    time_sp: None,
                    position_sp: None,
                    spec: spec.clone(),
                }
                .into(),
            );
        }

        let mut sensors = HashMap::default();
        for color_sensor in Ev3ColorSensor::list().context("Find color sensors")? {
            let address = color_sensor.get_address()?;
            sensors.insert(
                address,
                LegoSensor::Color(
                    init_color_sensor(color_sensor)
                        .context("Init color sensor")?
                        .into(),
                ),
            );
        }
        for gyro_sensor in Ev3GyroSensor::list().context("Find gyro sensors")? {
            let address = gyro_sensor.get_address()?;
            sensors.insert(
                address,
                LegoSensor::Gyro(
                    init_gyro_sensor(gyro_sensor)
                        .context("Init gyro sensor")?
                        .into(),
                ),
            );
        }

        Ok(Self {
            battery,
            buttons,
            input,
            buttons_raw: input_raw,
            controller,
            motors,
            spec,
            angle_algorithm,
            sensors,
        })
    }
}

impl Robot for LegoRobot {
    fn drive(&self, distance: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.controller.drive(
            self,
            distance.into().to_deg(&self.spec),
            speed.into().to_dps(&self.spec),
        )
    }

    fn turn(&self, angle: Heading, speed: impl Into<Speed>) -> Result<()> {
        self.controller
            .turn(self, angle, speed.into().to_dps(&self.spec))
    }

    fn turn_named(&self, angle: Heading, speed: impl Into<Speed>, turn: TurnType) -> Result<()> {
        self.controller
            .turn_named(self, angle, speed.into().to_dps(&self.spec), turn)
    }

    fn motor(&self, motor_id: MotorId) -> Option<RefMut<dyn Motor>> {
        let motor = self.motors.get(&motor_id);

        if let Some(motor) = motor {
            Some(motor.borrow_mut())
        } else {
            None
        }
    }

    fn color_sensor(&self, port: SensorPort) -> Option<RefMut<dyn ColorSensor>> {
        let sensor = self.sensors.get(&port.address());

        if let Some(LegoSensor::Color(color)) = sensor {
            Some(color.borrow_mut())
        } else {
            None
        }
    }

    fn process_buttons(&self) -> Result<Input> {
        self.buttons.process();

        let mut state = [false; 6];

        state[Input::UP] = self.buttons.is_up();
        state[Input::DOWN] = self.buttons.is_down();
        state[Input::LEFT] = self.buttons.is_left();
        state[Input::RIGHT] = self.buttons.is_right();
        state[Input::ENTER] = self.buttons.is_enter();
        state[Input::BACKSPACE] = self.buttons.is_backspace();

        self.input.borrow_mut().update(state);

        Ok(self.input.borrow().clone())
    }

    fn battery(&self) -> Result<Percent> {
        let min = self
            .battery
            .get_voltage_min_design()
            .context("Get min voltage")? as f32;
        let max = self
            .battery
            .get_voltage_max_design()
            .context("Get max voltage")? as f32;
        let current = self.battery.get_voltage_now().context("Get voltage")? as f32;

        Ok(Percent((current - min) / (max - min)))
    }

    fn await_input(&self) -> Result<Input> {
        wait::wait(self.buttons_raw.as_raw_fd(), || true, None);

        self.process_buttons()
    }

    fn reset(&self) -> Result<()> {
        for motor in self.motors.values() {
            motor
                .borrow_mut()
                .motor_reset(None)
                .context("Reset motor")?;
        }
        for sensor in self.sensors.values() {
            match sensor {
                LegoSensor::Gyro(gyro) => {
                    reset_gyro(&mut *gyro.borrow_mut())?;
                }
                LegoSensor::Color(_) => {}
            }
        }
        *self.input.borrow_mut() = Default::default();

        Ok(())
    }

    fn spec(&self) -> &RobotSpec {
        &self.spec
    }
}

impl AngleProvider for LegoRobot {
    fn angle(&self) -> Result<Heading> {
        match self.angle_algorithm {
            AngleAlgorithm::Wheel => {
                let left = self
                    .motor(MotorId::DriveLeft)
                    .context("Left motor")?
                    .motor_angle()?;
                let right = self
                    .motor(MotorId::DriveRight)
                    .context("Right motor")?
                    .motor_angle()?;

                Ok(self.spec().get_approx_angle(left, right))
            }
            AngleAlgorithm::Gyro(port) => {
                let sensor = self.sensors.get(&port.address()).context("No gyro found")?;

                if let LegoSensor::Gyro(gyro) = sensor {
                    Ok(Heading(gyro.borrow().gyro.get_angle()? as f32))
                } else {
                    bail!("Sensor in {port:?} is not a gyro sensor");
                }
            }
        }
    }
}

impl LegoMotor {
    fn handle_command(&mut self, command: &Command, execute: bool) -> Result<()> {
        match command {
            Command::On(speed) => {
                let dps = speed.to_dps(&self.spec).0 as i32;

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                }

                if execute {
                    self.motor.run_forever().context("Run forever")?;
                }
            }
            Command::Stop(action) => {
                if Some(*action) != self.stopping_action {
                    self.motor
                        .set_stop_action(action.to_str())
                        .context("Set stop action")?;
                }

                if execute {
                    self.motor.stop().context("Stop motor")?;
                }
            }
            Command::Distance(distance, speed) => {
                let deg = distance.to_deg(&self.spec).0 as i32;
                let dps = speed.to_dps(&self.spec).0 as i32;

                if Some(deg) != self.position_sp {
                    self.motor
                        .set_position_sp(deg)
                        .context("Set position setpoint")?;
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                }

                if execute {
                    self.motor.run_to_rel_pos(None).context("Run relative")?;
                }
            }
            Command::To(position, speed) => {
                let deg = position.to_deg(&self.spec).0 as i32;
                let dps = speed.to_dps(&self.spec).0 as i32;

                if Some(deg) != self.position_sp {
                    self.motor
                        .set_position_sp(deg)
                        .context("Set position setpoint")?;
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                }

                if execute {
                    self.motor.run_to_abs_pos(None).context("Run absloute")?;
                }
            }
            Command::Time(duration, speed) => {
                let dps = speed.to_dps(&self.spec).0 as i32;

                if Some(*duration) != self.time_sp {
                    self.motor
                        .set_time_sp(duration.as_millis() as i32)
                        .context("Set time setpoint")?;
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                }

                if execute {
                    self.motor.run_timed(None).context("Run timed")?;
                }
            }
            command => panic!("Got bad command {command:?}"),
        }

        Ok(())
    }
}

impl Motor for LegoMotor {
    fn raw(&mut self, command: Command) -> Result<()> {
        // Clear queued action if needed
        match command {
            Command::Queue(_) | Command::Execute => {}
            _ => {
                self.queued_command = None;
            }
        }

        match command {
            Command::Queue(queued_command) => {
                self.handle_command(&queued_command, false)?;

                self.queued_command = Some(*queued_command);
            }
            Command::Execute => {
                if let Some(queued_command) = self.queued_command.take() {
                    self.handle_command(&queued_command, true)?;
                } else {
                    bail!("Reveived `Command::Execute` when no command was queued");
                }
            }
            command => {
                self.handle_command(&command, true)?;
            }
        }

        Ok(())
    }

    fn motor_reset(&mut self, stop_action: Option<StopAction>) -> Result<()> {
        self.motor.reset().context("Reset motor")?;

        self.queued_command = None;
        self.stopping_action = None;
        self.speed_sp = None;
        self.time_sp = None;
        self.position_sp = None;

        if let Some(stop_action) = stop_action {
            self.motor
                .set_stop_action(stop_action.to_str())
                .context("Set stop action")?;

            self.stopping_action = Some(stop_action);
        }

        Ok(())
    }

    fn wait(&self, timeout: Option<Duration>) -> Result<()> {
        self.motor.wait_until_not_moving(timeout);

        Ok(())
    }

    fn speed(&self) -> Result<Speed> {
        self.motor
            .get_speed()
            .map(|it| DegreesPerSecond(it as f32).into())
            .context("Read motor speed")
    }

    fn motor_angle(&self) -> Result<Distance> {
        self.motor
            .get_position()
            .map(|it| Degrees(it as f32).into())
            .context("Read motor position")
    }
}

impl ColorSensor for LegoColorSensor {
    fn reflected_light(&self) -> Result<Percent> {
        let reflected = self.color.get_color().context("Read color sensor")?;
        let percent = reflected as f32 / 100.0;
        let adjusted = (percent - self.black) / (self.white - self.black);

        Ok(Percent(adjusted))
    }

    fn cal_white(&mut self) -> Result<()> {
        let reflected = self.color.get_color().context("Read color sensor")?;
        self.white = reflected as f32 / 100.0;

        Ok(())
    }

    fn cal_black(&mut self) -> Result<()> {
        let reflected = self.color.get_color().context("Read color sensor")?;
        self.black = reflected as f32 / 100.0;

        Ok(())
    }

    fn reset(&mut self) -> Result<()> {
        self.white = 1.0;
        self.black = 0.0;

        Ok(())
    }
}

fn init_color_sensor(color: Ev3ColorSensor) -> Result<LegoColorSensor> {
    color
        .set_mode_col_reflect()
        .context("Set color sensor mode")?;

    Ok(LegoColorSensor {
        color,
        white: 1.0,
        black: 0.0,
    })
}

fn init_gyro_sensor(gyro: Ev3GyroSensor) -> Result<LegoGyroSensor> {
    gyro.set_mode_gyro_ang().context("Set gyro sensor mode")?;

    let mut gyro = LegoGyroSensor { gyro };
    reset_gyro(&mut gyro).context("Reset gyro")?;

    Ok(gyro)
}

fn reset_gyro(gyro: &mut LegoGyroSensor) -> Result<()> {
    todo!("Proper reset it");
}
