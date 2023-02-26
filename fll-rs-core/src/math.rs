pub fn modf(a: f32, b: f32) -> f32 {
    (a % b + b) % b
}

pub fn modi(a: i32, b: i32) -> i32 {
    (a % b + b) % b
}

/// Normalizes an angle in degrees to a number in (-180, 180)
pub fn normalize_angle(angle: f32) -> f32 {
    let temp = modf(angle, 360.0);
    if temp > 180.0 {
        temp - 360.0
    } else {
        temp
    }
}

/// Caculates the distance in degrees between to angles
pub fn subtract_angles(a: f32, b: f32) -> f32 {
    // TODO check correctness
    normalize_angle(modf(a, 360.0) - modf(b, 360.0))
}
