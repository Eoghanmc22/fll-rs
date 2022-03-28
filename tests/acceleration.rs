use fll_rs::movement::acceleration::TrapezoidalAcceleration;

#[test]
fn acceleration_test() {
    let rates = vec![2000.0, 1500.0, 800.0, 500.0, 100.0, 10.0, 0.0];
    let speeds = vec![1000.0, 750.0, 500.0, 100.0, 50.0, 0.0];
    let distances = vec![10_000.0, 5000.0, 1000.0, 800.0, 500.0, 200.0, 100.0, 50.0, 10.0, 0.0];

    for distance in distances.iter() {
        for speed in speeds.iter() {
            for acceleration_rate in rates.iter() {
                for deceleration_rate in rates.iter() {
                    simulate(*distance, *speed, *acceleration_rate, *deceleration_rate)
                }
            }
        }
    }
}

const TIME_STEP: f32 = 0.001;
const ELISION: f32 = 0.001;

fn simulate(distance: f32, speed: f32, acceleration: f32, deceleration: f32) {
    println!("Curve dist: {}, speed: {}, acceleration: {}, deceleration: {}", distance, speed, acceleration, deceleration);

    let acceleration_curve = TrapezoidalAcceleration::new(distance, speed, acceleration, deceleration);

    assert_eq!(acceleration_curve.distance(), distance,
               "Curve runs for ({}) degs but ({}) degs were requested", acceleration_curve.distance(), distance);
    assert!(acceleration_curve.target_speed() <= speed
                || acceleration_curve.target_speed() == TrapezoidalAcceleration::min_speed(),
            "Computed speed ({}) is greater than the provided speed ({})", acceleration_curve.target_speed(), speed);


    let mut data = Vec::new();

    let mut pos = 0.0;
    let mut duration = 0.0;

    let mut last_speed = TrapezoidalAcceleration::min_speed();
    let threshold = f32::max(acceleration * TIME_STEP, deceleration * TIME_STEP) + ELISION;

    while pos < distance {
        let speed = acceleration_curve.eval(duration).0;
        pos += speed * TIME_STEP;

        duration += TIME_STEP;

        assert!((speed - last_speed).abs() < threshold, "Not continuous, difference: {}, allowed difference: {}, time: {}, pos: {}", (speed - last_speed).abs(), threshold, duration, pos);

        last_speed = speed;

        data.push((duration, speed));
    }

    assert!((duration - acceleration_curve.duration()).abs() < TIME_STEP + ELISION,
            "Real ({}) duration differed from expected ({})", duration, acceleration_curve.duration());
    assert!((distance - acceleration_curve.distance()).abs() < acceleration_curve.target_speed() * TIME_STEP + ELISION,
            "Real ({}) distance differed from expected ({})", duration, acceleration_curve.duration());
}