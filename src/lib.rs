#![feature(once_cell)]

// TODO enable more linting rules

pub mod error;
pub mod graphics;
pub mod input;
pub mod lego;
pub mod math;
pub mod movement;
pub mod robot;
pub mod types;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
