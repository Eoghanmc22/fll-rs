pub mod robot;
pub mod math;
pub mod menu;
pub mod graphics;
pub mod profiler;
pub mod movement;
#[cfg(target_arch = "arm")]
#[path = "lego.rs"]
pub mod robot_impl;
#[cfg(not(target_arch = "arm"))]
#[path = "mock.rs"]
pub mod robot_impl;
pub mod error;

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}