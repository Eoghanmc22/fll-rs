use std::cell::RefCell;

use anyhow::Context;
use ev3dev_lang_rust::{motors::MotorPort, sensors::SensorPort};
use fll_rs::{
    graphics::{
        display::{self, Display, Renderer},
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

fn main() -> anyhow::Result<()> {
    // Define constants for robot
    let motor_definitions = &[
        (MotorId::Attachment1, MotorPort::OutA),
        (MotorId::DriveLeft, MotorPort::OutB),
        (MotorId::DriveRight, MotorPort::OutC),
        (MotorId::Attachment2, MotorPort::OutD),
    ];
    let pid_config = PidConfig {
        kp: 0.3,
        ki: 0.02,
        kd: 0.6,

        max_i: 200.0,
    };
    let spec = RobotSpec::new(1000.dps2(), 500.dps2(), 88.mm(), 120.mm(), 1000.dps());
    let angle_algorithm = AngleAlgorithm::Gyro(SensorPort::In3);

    // Construct Robot
    let robot = RefCell::new(
        LegoRobot::new(motor_definitions, pid_config, spec, angle_algorithm)
            .context("Create robot")?,
    );

    // Construct Renderer
    let display = LegoDisplay::new().context("Create display")?;
    let renderer = RefCell::new(Renderer::new(display));

    let mut main_menu = Menu::new(5);

    // Define menu entries
    let reset_attachments = wrap_mission(&robot, &renderer, missions::reset_attachments);
    main_menu.push("Reset attachments", &reset_attachments);
    let mission_test = wrap_mission(&robot, &renderer, missions::test);
    main_menu.push("Test", &mission_test);
    let snake_game = || {
        games::snake(&robot, &renderer);
    };
    main_menu.push("Snake Game", &snake_game);

    loop {
        main_menu.render(&mut renderer.borrow_mut());

        let new_input = robot.borrow().await_input(None).context("Await imput")?;
        if !main_menu.notify_input(&new_input) {
            return Ok(());
        }
    }
}

#[allow(clippy::precedence)]
mod missions {
    use anyhow::{Context, Result};
    use fll_rs::robot::{Command, MotorExt, MotorId, Robot, StopAction, TurnType};
    use fll_rs::types::UnitsExt;
    use std::thread;
    use std::time::Duration;

    pub fn test<R: Robot>(robot: &R) -> Result<()> {
        // Retrieve attachment motors
        let mut attachment1 = robot.motor(MotorId::Attachment1).context("Attachment 1")?;
        let mut attachment2 = robot.motor(MotorId::Attachment2).context("Attachment 2")?;

        // Set attachment motors to hold position when stopped
        attachment1.motor_reset(Some(StopAction::Hold))?;
        attachment2.motor_reset(Some(StopAction::Hold))?;

        robot.drive(100.deg(), 80.pct())?;
        robot.turn(90.ang(), 80.pct())?;
        robot.drive(100.deg(), 80.pct())?;
        robot.turn(180.ang(), 80.pct())?;
        robot.drive(100.deg(), 80.pct())?;
        robot.turn(-90.ang(), 80.pct())?;
        robot.drive(100.deg(), 80.pct())?;
        robot.turn(0.ang(), 80.pct())?;

        Ok(())
    }

    pub fn reset_attachments<R: Robot>(robot: &R) -> Result<()> {
        // Retrieve attachment motors
        let mut attachment1 = robot.motor(MotorId::Attachment1).context("Attachment 1")?;
        let mut attachment2 = robot.motor(MotorId::Attachment2).context("Attachment 2")?;

        // Start moving both attachment motors at the same time
        attachment1.raw(Command::Time(
            Duration::from_secs_f64(1.0),
            100.pct().into(),
        ))?;
        attachment2.raw(Command::Time(
            Duration::from_secs_f64(0.5),
            100.pct().into(),
        ))?;

        // Wait for both motors to finish moving
        attachment1.wait(None)?;
        attachment2.wait(None)?;

        Ok(())
    }
}

mod games {
    use std::{
        cell::RefCell,
        collections::HashSet,
        thread,
        time::{Duration, Instant},
    };

    use fll_rs::{
        graphics::display::{self, Display, Renderer},
        math,
        robot::Robot,
    };
    use rand::Rng;

    pub fn snake<R: Robot, D: Display>(robot: &RefCell<R>, renderer: &RefCell<Renderer<D>>) {
        let board_size = (40i32, 30i32);
        let apple_count = 5;

        let start = Instant::now();
        let robot = robot.borrow_mut();
        let mut renderer = renderer.borrow_mut();
        let mut random = rand::thread_rng();
        let mut snake = vec![(board_size.0 / 2, board_size.1 / 2)];
        let mut apples = HashSet::new();
        let mut direction = (0, 0);

        // Place initial apples
        while apples.len() < apple_count {
            apples.insert((
                random.gen_range(0..board_size.0),
                random.gen_range(0..board_size.1),
            ));
        }

        let mut deadline = Instant::now();
        let game_loop_duration = Duration::from_millis(400);

        loop {
            // Process input and wait until next tick
            loop {
                let remaining = deadline - Instant::now();

                let Ok(input) = robot.await_input(Some(remaining)) else { return; };

                if input.is_up() {
                    direction = (0, -1);
                }
                if input.is_down() {
                    direction = (0, 1);
                }
                if input.is_right() {
                    direction = (1, 0);
                }
                if input.is_left() {
                    direction = (-1, 0);
                }

                // Exit when enter is pressed
                if input.is_enter() && start.elapsed() > Duration::from_millis(500) {
                    return;
                }

                if remaining.is_zero() {
                    break;
                }
            }

            // Handle eatting an apple
            {
                let Some(head) = snake.first() else { return; };
                let Some(tail) = snake.last() else { return; };

                if apples.remove(head) {
                    // Replace apple
                    while apples.len() < apple_count {
                        apples.insert((
                            random.gen_range(0..board_size.0),
                            random.gen_range(0..board_size.1),
                        ));
                    }

                    // Make snake longer
                    snake.push(*tail);
                }
            }

            // Move the snake forward
            {
                let last_snake = snake.clone();
                for (idx, part) in snake.iter_mut().enumerate().skip(1) {
                    *part = last_snake[idx - 1];
                }
            }

            // Move the head forward
            {
                let Some(head) = snake.first_mut() else { return; };
                // Add direction to position
                head.0 += direction.0;
                head.1 += direction.1;

                // Wrap into bounds
                head.0 = math::modi(head.0, board_size.0);
                head.1 = math::modi(head.1, board_size.1);
            }

            // Handle the snake dying
            {
                let Some(head) = snake.first() else { return; };
                if snake[1..].contains(head) {
                    renderer.clear();
                    renderer.draw_text("You died", 0, 0, 20.0, display::BLACK);
                    renderer.update();

                    thread::sleep(Duration::from_millis(1500));

                    return;
                }
            }

            // Draw the game
            {
                let pixels_per_tile_x = (renderer.width() as i32 - board_size.0) / board_size.0;
                let pixels_per_tile_y = (renderer.height() as i32 - board_size.1) / board_size.1;

                renderer.clear();

                // Draw the snake
                for (x, y) in &snake {
                    renderer.draw_rectangle_solid(
                        x * pixels_per_tile_x + x,
                        y * pixels_per_tile_y + y,
                        pixels_per_tile_x as u32,
                        pixels_per_tile_y as u32,
                        display::BLACK,
                    );
                }

                // Draw the apples
                for (x, y) in &apples {
                    renderer.draw_rectangle_solid(
                        x * pixels_per_tile_x + x,
                        y * pixels_per_tile_y + y,
                        pixels_per_tile_x as u32,
                        pixels_per_tile_y as u32,
                        display::BLACK,
                    );
                }

                renderer.update();
            }

            // Set deadline for when the next tick should begin
            {
                deadline += game_loop_duration;
            }
        }
    }
}

fn wrap_mission<'a, R: Robot, D: Display, F: Fn(&R) -> anyhow::Result<()> + 'a>(
    robot: &'a RefCell<R>,
    renderer: &'a RefCell<Renderer<D>>,
    code: F,
) -> impl Fn() + 'a {
    move || {
        let robot = robot.borrow();

        let rst = (|| {
            robot.reset().context("Reset robot")?;

            let rst = (code)(&robot).context("Mission");

            robot.stop().context("Cleanup robot")?;

            rst
        })();

        if let Err(err) = rst {
            eprintln!("{err:?}");

            let mut renderer = renderer.borrow_mut();

            renderer.clear();
            renderer.draw_text(&format!("{err}"), 0, 0, 5.0, display::BLACK);
            renderer.update();

            let _ = robot.await_input(None);
        }
    }
}
