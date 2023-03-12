use std::{cell::RefCell, rc::Rc};

use anyhow::Context;
use ev3dev_lang_rust::{
    motors::{LargeMotor, MediumMotor, MotorPort},
    sensors::SensorPort,
    Device,
};
use fll_rs::{
    graphics::{
        display::{self, Renderer},
        menu::Menu,
    },
    lego::{
        graphics_impl::LegoDisplay,
        robot_impl::{AngleAlgorithm, LegoRobot},
    },
    movement::{pid::PidConfig, spec::RobotSpec},
    robot::{MotorId, Robot},
    types::UnitsExt,
};

macro_rules! handle_errors {
    ($robot:ident, $renderer:ident, $code:expr) => {{
        let robot = $robot.clone();
        let renderer = $renderer.clone();
        let code = $code;
        move || {
            let rst = (code)();

            if let Err(err) = rst {
                eprintln!("{err:?}");

                let robot = robot.borrow();
                let mut renderer = renderer.borrow_mut();

                renderer.clear();
                renderer.draw_text(&format!("{err}"), 0, 0, 5.0, display::BLACK);
                renderer.update();

                let _ = robot.await_input();
            }
        }
    }};
}

macro_rules! add_mission {
    ($menu:ident, $robot:ident, $renderer:ident, $name:expr, $mission:expr) => {
        $menu.push(
            $name,
            handle_errors!($robot, $renderer, {
                let robot = $robot.clone();
                move || {
                    let robot = robot.borrow();

                    ($mission)(&*robot)
                }
            }),
        );
    };
}

fn main() -> anyhow::Result<()> {
    {
        let motors = LargeMotor::list().context("List motors")?;
        for motor in motors {
            let address = motor.get_address().context("Get address")?;
            let max_speed = motor.get_max_speed().context("Get max speed")?;

            eprintln!("{address}:{max_speed}");
        }
        let motors = MediumMotor::list().context("List motors")?;
        for motor in motors {
            let address = motor.get_address().context("Get address")?;
            let max_speed = motor.get_max_speed().context("Get max speed")?;

            eprintln!("{address}:{max_speed}");
        }
    }

    let motor_definitions = &[
        (MotorId::DriveLeft, MotorPort::OutB),
        (MotorId::DriveRight, MotorPort::OutC),
    ];
    let pid_config = PidConfig {
        kp: 2.0,
        ki: 0.01,
        kd: 7.0,
    };
    let spec = RobotSpec::new(
        1000.dps2(),
        300.dps2(),
        96.mm(),
        150.mm(),
        960.dps(),
        1.0,
        1.0,
        1.0,
        1.0,
    );
    let angle_algorithm = AngleAlgorithm::Gyro(SensorPort::In3);

    let robot = Rc::new(RefCell::new(
        LegoRobot::new(motor_definitions, pid_config, spec, angle_algorithm)
            .context("Create robot")?,
    ));
    let display = LegoDisplay::new().context("Create display")?;
    let renderer = Rc::new(RefCell::new(Renderer::new(display)));

    let mut main_menu = Menu::new(5);

    add_mission!(main_menu, robot, renderer, "Test 1", missions::test);

    main_menu.push(
        "Test 1",
        handle_errors!(robot, renderer, {
            let robot = robot.clone();
            move || {
                let robot = robot.borrow();

                missions::test(&*robot)
            }
        }),
    );

    loop {
        main_menu.render(&mut renderer.borrow_mut());

        let new_input = robot.borrow().await_input().context("Await imput")?;
        if !main_menu.notify_input(&new_input) {
            return Ok(());
        }
    }
}

mod missions {
    use fll_rs::robot::Robot;

    pub fn test<R: Robot>(robot: &R) -> anyhow::Result<()> {
        Ok(())
    }
}
