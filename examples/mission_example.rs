use fll_rs::movement::pid::PidConfig;
use fll_rs::movement::spec::RobotSpec;
use fll_rs::robot::{DualColorSensor, MotorId};
use fll_rs::error::Result;
use fll_rs::robot_impl::{LegoRobot, HasGyroSensor, HasDualColorSensor};

fn main() -> Result<()> {
    let motor_definitions = &[(MotorId::DriveLeft, MotorPort::OutA), (MotorId::DriveRight, MotorPort::OutB)];

    let pid_config = PidConfig {
        kp: 1.0,
        ki: 0.0,
        kd: 0.0
    };

    let spec = RobotSpec::new(
        1500.0,
        800.0,
        62.4,
        168.0,
        1000.0,
        1.0,

        1.0,
        1.0,
        1.0
    );

    let robot: LegoRobot<HasGyroSensor, HasDualColorSensor> = LegoRobot::new_dual_color_sensor(SensorPort::In1, SensorPort::In2, motor_definitions, pid_config, spec)?;

    Ok(())
}
