use crate::types::{Heading, UnitsExt};

pub fn modf(a: f32, b: f32) -> f32 {
    (a % b + b) % b
}

pub fn modi(a: i32, b: i32) -> i32 {
    (a % b + b) % b
}

/// Normalizes an angle in degrees to a number in (-180, 180)
pub fn normalize_angle(Heading(angle): Heading) -> Heading {
    let temp = modf(angle, 360.0);
    if temp > 180.0 {
        Heading(temp - 360.0)
    } else {
        Heading(temp)
    }
}

/// Caculates the distance in degrees between to angles
pub fn subtract_angles(Heading(a): Heading, Heading(b): Heading) -> Heading {
    // TODO check correctness
    normalize_angle((modf(a, 360.0) - modf(b, 360.0)).angle())
}
