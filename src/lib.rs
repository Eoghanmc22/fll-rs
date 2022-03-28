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
