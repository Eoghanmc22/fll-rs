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
    normalize_angle((modf(a, 360.0) - modf(b, 360.0)).ang())
}

pub fn add_angles(Heading(a): Heading, Heading(b): Heading) -> Heading {
    // TODO check correctness
    normalize_angle((modf(a, 360.0) + modf(b, 360.0)).ang())
}

pub fn lerp_angles(alpha: f32, a: Heading, b: Heading) -> Heading {
    let alpha = alpha.min(1.0).max(0.0);

    // TODO check correctness
    let delta = subtract_angles(b, a).0;
    let offset = delta * alpha;
    add_angles(a, Heading(offset))
}
