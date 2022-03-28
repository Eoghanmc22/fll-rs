pub fn modf(a: f32, b: f32) -> f32 {
    (a % b + b) % b
}

pub fn modi(a: i32, b: i32) -> i32 {
    (a % b + b) % b
}

pub fn normalize_angle(angle: f32) -> f32 {
    let temp = modf(angle, 360.0);
    if temp > 180.0 {
        temp - 360.0
    } else {
        temp
    }
}

pub fn subtract_angles(a: f32, b: f32) -> f32 {
    normalize_angle(modf(a, 360.0) - modf(b, 360.0))
}

pub fn clampf(val: f32, max: f32, min: f32) -> f32 {
    debug_assert!(max > min, "Max must be greater then min!");
    val.min(max).max(min)
}

pub fn clampi(val: i32, max: i32, min: i32) -> i32 {
    debug_assert!(max > min, "Max must be greater then min!");
    val.min(max).max(min)
}