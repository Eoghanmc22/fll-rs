use crate::{error::Result, graphics::display::Display};
use ev3dev_lang_rust::Screen;

/// Implemention of `Display` for ev3dev
pub struct LegoDisplay {
    screen: Screen,
}

impl LegoDisplay {
    pub fn new() -> Result<Self> {
        let screen = Screen::new()?;

        Ok(LegoDisplay { screen })
    }
}

impl Display for LegoDisplay {
    fn get_image(&self) -> &image::RgbImage {
        &self.screen.image
    }

    fn get_image_mut(&mut self) -> &mut image::RgbImage {
        &mut self.screen.image
    }

    fn update(&mut self) {
        self.screen.update()
    }
}
