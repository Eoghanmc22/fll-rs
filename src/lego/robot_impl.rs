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
use ev3dev_lang_rust::sensors::Sensor;
use ev3dev_lang_rust::sensors::SensorPort;
use ev3dev_lang_rust::{wait, Ev3Button, Port, PowerSupply};
use ev3dev_lang_rust::{Attribute, Device};
use fxhash::FxHashMap as HashMap;
use std::cell::{RefCell, RefMut};
use std::fs::File;
use std::os::unix::io::AsRawFd;
use std::rc::Rc;
use std::thread;
use std::time::Duration;

pub struct LegoRobot {
    battery: PowerSupply,

    buttons: Ev3Button,
    buttons_raw: File,
    input: RefCell<Input>,

    pid_config: PidConfig,
    controller: RefCell<MovementController>,
    angle_algorithm: AngleAlgorithm,

    motors: HashMap<MotorId, RefCell<LegoMotor>>,
    motor_definitions: Vec<(MotorId, MotorPort)>,
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
        // Wait for any drivers to start up
        thread::sleep(Duration::from_millis(300));

        let battery = PowerSupply::new().context("Couldn't find power supply")?;

        let controller = MovementController::new(pid_config).into();

        let spec = Rc::new(spec);

        // Setup input
        let buttons = Ev3Button::new().context("Couldn't find buttons")?;
        let input = Default::default();
        let buttons_raw = File::open("/dev/input/by-path/platform-gpio_keys-event")?;

        // Discover sensors and motors
        let motor_definitions: Vec<_> = motor_definitions.into();
        let motors =
            discover_motors(&motor_definitions, spec.clone()).context("Discover motors")?;
        let sensors = discover_sensors().context("Discover sensors")?;

        Ok(Self {
            battery,
            buttons,
            input,
            buttons_raw,
            controller,
            motors,
            spec,
            angle_algorithm,
            sensors,
            motor_definitions,
            pid_config,
        })
    }

    pub fn rediscover_motors(&mut self) -> Result<()> {
        self.motors = discover_motors(&self.motor_definitions, self.spec.clone())
            .context("Discover motors")?;

        Ok(())
    }

    pub fn rediscover_sensors(&mut self) -> Result<()> {
        self.sensors = discover_sensors().context("Discover sensors")?;

        Ok(())
    }

    fn update_button_state(&self) -> bool {
        self.buttons.process();

        let mut state = [false; 6];

        state[Input::UP] = self.buttons.is_up();
        state[Input::DOWN] = self.buttons.is_down();
        state[Input::LEFT] = self.buttons.is_left();
        state[Input::RIGHT] = self.buttons.is_right();
        state[Input::ENTER] = self.buttons.is_enter();
        state[Input::BACKSPACE] = self.buttons.is_backspace();

        self.input.borrow_mut().update(state)
    }
}

impl Robot for LegoRobot {
    fn drive(&self, distance: impl Into<Distance>, speed: impl Into<Speed>) -> Result<()> {
        self.controller.borrow_mut().drive(
            self,
            distance.into().to_deg(&self.spec),
            speed.into().to_dps(&self.spec),
        )
    }

    fn turn(&self, angle: Heading, speed: impl Into<Speed>) -> Result<()> {
        self.controller
            .borrow_mut()
            .turn(self, angle, speed.into().to_dps(&self.spec))
    }

    fn turn_named(&self, angle: Heading, speed: impl Into<Speed>, turn: TurnType) -> Result<()> {
        self.controller
            .borrow_mut()
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
        self.update_button_state();

        Ok(self.input.borrow().clone())
    }

    fn battery(&self) -> Result<Percent> {
        // let min = self
        //     .battery
        //     .get_voltage_min_design()
        //     .context("Get min voltage")? as f32;

        let max = self
            .battery
            .get_voltage_max_design()
            .context("Get max voltage")? as f32;
        let current = self.battery.get_voltage_now().context("Get voltage")? as f32;

        // Ok(Percent((dbg!(current) - dbg!(min)) / (dbg!(max) - min)))

        // Seems to be a bug in ev3dev
        Ok(Percent((current * 10.0 / max).min(1.0).max(0.0)))
    }

    fn await_input(&self) -> Result<Input> {
        wait::wait(
            self.buttons_raw.as_raw_fd(),
            || self.update_button_state(),
            None,
        );

        Ok(self.input.borrow().clone())
    }

    fn stop(&self) -> Result<()> {
        for motor in self.motors.values() {
            motor
                .borrow_mut()
                .motor_reset(None)
                .context("Reset motor")?;
        }
        Ok(())
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
                    reset_gyro_soft(&mut gyro.borrow_mut())?;
                }
                LegoSensor::Color(_) => {}
            }
        }
        *self.input.borrow_mut() = Default::default();
        *self.controller.borrow_mut() = MovementController::new(self.pid_config);

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
                    Ok(Heading(gyro.borrow().gyro.get_value0()? as f32))
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
                    self.speed_sp = Some(dps);
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
                    self.stopping_action = Some(*action);
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
                    self.position_sp = Some(deg);
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                    self.speed_sp = Some(dps);
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
                    self.position_sp = Some(deg);
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                    self.speed_sp = Some(dps);
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
                    self.time_sp = Some(*duration);
                }

                if Some(dps) != self.speed_sp {
                    self.motor.set_speed_sp(dps).context("Set speed setpoint")?;
                    self.speed_sp = Some(dps)
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

        self.motor.stop().context("Stop motor")?;

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

fn discover_motors(
    motor_definitions: &[(MotorId, MotorPort)],
    spec: Rc<RobotSpec>,
) -> Result<HashMap<MotorId, RefCell<LegoMotor>>> {
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

    Ok(motors)
}

fn discover_sensors() -> Result<HashMap<String, LegoSensor>> {
    let mut sensors = HashMap::default();

    for color_sensor in Ev3ColorSensor::list().context("Find color sensors")? {
        let address = color_sensor
            .get_address()?
            .strip_prefix("ev3-ports:")
            .context("Strip prefix")?
            .to_owned();
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
        let address = gyro_sensor
            .get_address()?
            .strip_prefix("ev3-ports:")
            .context("Strip prefix")?
            .to_owned();
        sensors.insert(
            address,
            LegoSensor::Gyro(
                init_gyro_sensor(gyro_sensor)
                    .context("Init gyro sensor")?
                    .into(),
            ),
        );
    }

    Ok(sensors)
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

    reset_gyro_hard(&mut gyro).context("Reset gyro hard")?;
    reset_gyro_soft(&mut gyro).context("Reset gyro soft")?;

    Ok(gyro)
}

fn reset_gyro_hard(gyro: &mut LegoGyroSensor) -> Result<()> {
    let address = gyro.gyro.get_address().context("Get sensor address")?;

    let mode = Attribute::from_path_with_discriminator(
        "/sys/class/lego-port",
        "/mode",
        "/address",
        address.as_str(),
    )
    .context("Get mode attribute")?;

    mode.set_str_slice("other-uart")
        .context("Hard reset gyro")?;
    thread::sleep(Duration::from_millis(300));
    mode.set_str_slice("auto").context("Hard reset gyro")?;

    for _ in 0..100 {
        let gyros = Ev3GyroSensor::list().context("Find gyro sensors")?;
        for new_gyro in gyros {
            if new_gyro
                .get_address()
                .context("Get new sensor address")?
                .contains(&address)
            {
                gyro.gyro = new_gyro;

                // Wait for driver to initalize
                thread::sleep(Duration::from_millis(300));

                return Ok(());
            }
        }

        // Busy wait the gyro to come back online
        thread::sleep(Duration::from_millis(100));
    }

    bail!("Gyro did not reconnect within 10s");
}

fn reset_gyro_soft(gyro: &mut LegoGyroSensor) -> Result<()> {
    gyro.gyro
        .get_attribute("direct")
        .set_str_slice("\x11")
        .context("Write reset command")?;

    thread::sleep(Duration::from_millis(25));

    Ok(())
}
