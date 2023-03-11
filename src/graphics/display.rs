use image::{Rgb, RgbImage};
use imageproc::rect::Rect;
use rusttype::{Font, Scale};
use std::rc::Rc;

pub const WHITE: Rgb<u8> = Rgb([255, 255, 255]);
pub const BLACK: Rgb<u8> = Rgb([0, 0, 0]);

/// Repersents a phyical screen that can display static images
pub trait Display {
    fn get_image(&self) -> &RgbImage;
    fn get_image_mut(&mut self) -> &mut RgbImage;

    fn update(&mut self);
}

/// Simple graphics api that allows drawing text and rectangles
pub struct Renderer<D> {
    handler: D,
    font: Rc<Font<'static>>,
}

impl<D: Display> Renderer<D> {
    pub fn new(handler: D) -> Self {
        let font = Rc::new(Font::try_from_bytes(include_bytes!("font.ttf")).unwrap());

        Self { handler, font }
    }

    pub fn draw_rectangle_solid(
        &mut self,
        x: i32,
        y: i32,
        width: u32,
        height: u32,
        color: Rgb<u8>,
    ) {
        imageproc::drawing::draw_filled_rect_mut(
            self.get_image_mut(),
            Rect::at(x, y).of_size(width, height),
            color,
        );
    }

    pub fn draw_rectangle_hollow(
        &mut self,
        x: i32,
        y: i32,
        width: u32,
        height: u32,
        color: Rgb<u8>,
    ) {
        imageproc::drawing::draw_hollow_rect_mut(
            self.get_image_mut(),
            Rect::at(x, y).of_size(width, height),
            color,
        );
    }

    pub fn draw_text(&mut self, string: &str, x: i32, y: i32, size: f32, color: Rgb<u8>) {
        let font = self.font.clone();

        imageproc::drawing::draw_text_mut(
            self.get_image_mut(),
            color,
            x,
            y,
            Scale::uniform(size),
            &font,
            string,
        );
    }

    pub fn dimensions(&self) -> (u32, u32) {
        self.get_image().dimensions()
    }

    pub fn width(&self) -> u32 {
        self.get_image().width()
    }

    pub fn height(&self) -> u32 {
        self.get_image().height()
    }

    pub fn clear(&mut self) {
        self.get_image_mut().fill(255)
    }

    pub fn get_image(&self) -> &RgbImage {
        self.handler.get_image()
    }

    pub fn get_image_mut(&mut self) -> &mut RgbImage {
        self.handler.get_image_mut()
    }

    pub fn update(&mut self) {
        self.handler.update()
    }
}
