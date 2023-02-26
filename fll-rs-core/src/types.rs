use crate::movement::spec::RobotSpec;

// TODO validation

/// Repersents distance in milimeters
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Milimeters(pub f32);

/// Repersents distance in wheel degrees
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Degrees(pub f32);

/// Repersents distance in wheel rotations
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Rotations(pub f32);

/// Repersents speed in wheel degrees per second
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct DegreesPerSecond(pub f32);

/// Repersents speed in milimeters per second
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct MilimetersPerSecond(pub f32);

/// Repersents acceleration in wheel degrees per second per second
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct DegreesPerSecondPerSecond(pub f32);

/// Repersents acceleration in milimeters per second per second
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct MilimetersPerSecondPerSecond(pub f32);

/// Repersented as a number from -1.0 -> 1.0
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Percent(pub f32);

/// Repersent a heading, stored as degrees (-180 -> 180 when normalized)
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Heading(pub f32);

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum Distance {
    Milimeters(Milimeters),
    Degrees(Degrees),
    Rotations(Rotations),
}

impl Distance {
    pub fn to_deg(self, spec: &RobotSpec) -> Degrees {
        match self {
            Distance::Milimeters(val) => spec.mm_to_deg(val),
            Distance::Degrees(val) => val,
            Distance::Rotations(val) => Degrees(val.0 * 360.0),
        }
    }

    pub fn to_rot(self, spec: &RobotSpec) -> Rotations {
        match self {
            Distance::Milimeters(val) => Rotations(spec.mm_to_deg(val).0 / 360.0),
            Distance::Rotations(val) => val,
            Distance::Degrees(val) => Rotations(val.0 / 360.0),
        }
    }

    pub fn to_mm(self, spec: &RobotSpec) -> Milimeters {
        match self {
            Distance::Milimeters(val) => val,
            Distance::Rotations(val) => spec.deg_to_mm((val.0 * 360.0).deg()),
            Distance::Degrees(val) => spec.deg_to_mm(val),
        }
    }
}

impl From<Degrees> for Distance {
    fn from(value: Degrees) -> Self {
        Distance::Degrees(value)
    }
}

impl From<Rotations> for Distance {
    fn from(value: Rotations) -> Self {
        Distance::Rotations(value)
    }
}

impl From<Milimeters> for Distance {
    fn from(value: Milimeters) -> Self {
        Distance::Milimeters(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum Speed {
    DegreesPerSecond(DegreesPerSecond),
    MilimetersPerSecond(MilimetersPerSecond),
}

impl Speed {
    pub fn to_dps(self, spec: &RobotSpec) -> DegreesPerSecond {
        match self {
            Speed::DegreesPerSecond(val) => val,
            Speed::MilimetersPerSecond(val) => DegreesPerSecond(spec.mm_to_deg(val.0.mm()).0),
        }
    }

    pub fn to_mmps(self, spec: &RobotSpec) -> MilimetersPerSecond {
        match self {
            Speed::DegreesPerSecond(val) => MilimetersPerSecond(spec.deg_to_mm(val.0.deg()).0),
            Speed::MilimetersPerSecond(val) => val,
        }
    }
}

impl From<DegreesPerSecond> for Speed {
    fn from(value: DegreesPerSecond) -> Self {
        Speed::DegreesPerSecond(value)
    }
}

impl From<MilimetersPerSecond> for Speed {
    fn from(value: MilimetersPerSecond) -> Self {
        Speed::MilimetersPerSecond(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub enum Acceleration {
    Degrees(DegreesPerSecondPerSecond),
    Milimeters(MilimetersPerSecondPerSecond),
}

impl Acceleration {
    pub fn to_dps2(self, spec: &RobotSpec) -> DegreesPerSecondPerSecond {
        match self {
            Acceleration::Milimeters(val) => {
                DegreesPerSecondPerSecond(spec.mm_to_deg(val.0.mm()).0)
            }
            Acceleration::Degrees(val) => val,
        }
    }

    pub fn to_mmps2(self, spec: &RobotSpec) -> MilimetersPerSecondPerSecond {
        match self {
            Acceleration::Milimeters(val) => val,
            Acceleration::Degrees(val) => {
                MilimetersPerSecondPerSecond(spec.deg_to_mm(val.0.deg()).0)
            }
        }
    }
}

impl From<DegreesPerSecondPerSecond> for Acceleration {
    fn from(value: DegreesPerSecondPerSecond) -> Self {
        Acceleration::Degrees(value)
    }
}

impl From<MilimetersPerSecondPerSecond> for Acceleration {
    fn from(value: MilimetersPerSecondPerSecond) -> Self {
        Acceleration::Milimeters(value)
    }
}

pub trait UnitsExt {
    /// Converts this value to `Milimeters` (istance)
    fn mm(self) -> Milimeters;
    /// Converts this value to `Degrees` (istance)
    fn deg(self) -> Degrees;
    /// Converts this value to `Rotations` (distance)
    fn rot(self) -> Rotations;

    /// Converts this value to `DegreesPerSecond` (speed)
    fn dps(self) -> DegreesPerSecond;
    /// Converts this value to `MilimetersPerSecond` (speed)
    fn mmps(self) -> MilimetersPerSecond;

    /// Converts this value to `DegreesPerSecondPerSecond` (acceleration)
    fn dps2(self) -> DegreesPerSecondPerSecond;
    /// Converts this value to `MilimetersPerSecondPerSecond` (acceleration)
    fn mmps2(self) -> MilimetersPerSecondPerSecond;

    /// Converts this value to `Heading` (heading)
    fn angle(self) -> Heading;
}

impl UnitsExt for f32 {
    fn mm(self) -> Milimeters {
        Milimeters(self)
    }

    fn deg(self) -> Degrees {
        Degrees(self)
    }

    fn rot(self) -> Rotations {
        Rotations(self)
    }

    fn dps(self) -> DegreesPerSecond {
        DegreesPerSecond(self)
    }

    fn mmps(self) -> MilimetersPerSecond {
        MilimetersPerSecond(self)
    }

    fn dps2(self) -> DegreesPerSecondPerSecond {
        DegreesPerSecondPerSecond(self)
    }

    fn mmps2(self) -> MilimetersPerSecondPerSecond {
        MilimetersPerSecondPerSecond(self)
    }

    fn angle(self) -> Heading {
        Heading(self)
    }
}

impl UnitsExt for i32 {
    fn mm(self) -> Milimeters {
        Milimeters(self as f32)
    }

    fn deg(self) -> Degrees {
        Degrees(self as f32)
    }

    fn rot(self) -> Rotations {
        Rotations(self as f32)
    }

    fn dps(self) -> DegreesPerSecond {
        DegreesPerSecond(self as f32)
    }

    fn mmps(self) -> MilimetersPerSecond {
        MilimetersPerSecond(self as f32)
    }

    fn dps2(self) -> DegreesPerSecondPerSecond {
        DegreesPerSecondPerSecond(self as f32)
    }

    fn mmps2(self) -> MilimetersPerSecondPerSecond {
        MilimetersPerSecondPerSecond(self as f32)
    }

    fn angle(self) -> Heading {
        Heading(self as f32)
    }
}
