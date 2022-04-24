use std::error::Error;
use std::fmt::{Debug, Display, Formatter};
use ev3dev_lang_rust::Ev3Error;

pub mod robot;
pub mod math;
pub mod menu;
pub mod graphics;
pub mod profiler;
pub mod movement;
pub mod lego;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}

pub type Result<T> = std::result::Result<T, FLLError>;

#[derive(Debug)]
pub enum FLLError {
    StdError(Box<dyn Error + Send + Sync>),
    Ev3Error {
        device: String,
        info: String
    }
}

impl Display for FLLError {
    fn fmt(&self, f: &mut Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:#?}", self)
    }
}

impl Error for FLLError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            FLLError::StdError(error) => {
                Some(&**error)
            }
            _ => {
                None
            }
        }
    }
}

impl From<Box<dyn Error + Send + Sync>> for FLLError {
    fn from(err: Box<dyn Error + Send + Sync>) -> Self {
        FLLError::StdError(err)
    }
}

impl From<Ev3Error> for FLLError {
    fn from(err: Ev3Error) -> Self {
        match err {
            Ev3Error::InternalError { msg } => {
                FLLError::StdError(msg.into())
            }
            Ev3Error::NotConnected { device, port } => {
                if let Some(port) = port {
                    FLLError::Ev3Error { device, info: format!("Not found on port {}", port) }
                } else {
                    FLLError::Ev3Error { device, info: format!("Not connected on any port") }
                }
            }
            Ev3Error::MultipleMatches { device, .. } => {
                FLLError::Ev3Error { device, info: format!("Found multiple") }
            }
        }
    }
}