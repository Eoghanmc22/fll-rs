use std::error::Error;
use std::fmt::{Debug, Display, Formatter};
use ev3dev_lang_rust::Ev3Error;

pub use anyhow::Result;

pub struct Ev3ErrorWrapper(pub Ev3Error);

impl Debug for Ev3ErrorWrapper {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        self.0.fmt(f)
    }
}

impl Display for Ev3ErrorWrapper {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        match &self.0 {
            Ev3Error::InternalError { msg } => { write!(f, "{}", msg) }
            Ev3Error::NotConnected { device, .. } => { write!(f, "{} is not connected", device) }
            Ev3Error::MultipleMatches { device, .. } => { write!(f, "There are multiple {} connected", device) }
        }
    }
}

impl Error for Ev3ErrorWrapper {

}