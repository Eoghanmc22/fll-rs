use crate::graphics::display;
use crate::graphics::display::{Display, Renderer};
use crate::input::Input;

/// Simple ui to select an item from a list
pub struct Menu {
    top: usize,
    selected: usize,
    rows: usize,

    items: Vec<MenuItem>,
}

/// Repersents an item displayed in a `Menu`
struct MenuItem {
    name: String,
    callback: Box<dyn FnMut()>,
}

impl Menu {
    /// Creats a menu that displays `rows` rows at a time
    pub fn new(rows: usize) -> Self {
        Self {
            top: 0,
            selected: 0,
            rows,

            items: vec![],
        }
    }

    /// Adds a new `MenuItem` to the bottom of the `Menu`
    pub fn push<F: FnMut() + 'static>(&mut self, name: &str, code: F) {
        self.items.push(MenuItem {
            name: name.to_owned(),
            callback: Box::new(code),
        });
    }

    /// Makes sure the cursor is in a valid location
    fn update_bounds(&mut self) {
        // Wraps cursor to be a valid entry in the list
        self.selected %= self.items.len();

        // Updates what the top entry displayed is to follow the cursor
        let offset = self.selected as isize - self.top as isize;
        if offset < 0 {
            self.top = self.selected;
        } else if offset >= self.rows as isize {
            self.top = self.selected - self.rows + 1;
        }
    }
}

impl Menu {
    /// Renders the `Menu` to the provided `renderer`
    pub fn render<D: Display>(&self, renderer: &mut Renderer<D>) {
        let start = self.top;
        let end = usize::max(start + self.rows, self.items.len());

        let row_height = renderer.height() / self.rows as u32;

        for idx in start..end {
            let name = &self.items[idx].name;

            if idx == self.selected {
                renderer.draw_rectangle_solid(
                    0,
                    row_height as i32 * (idx as i32 - start as i32),
                    renderer.width(),
                    row_height,
                    display::BLACK,
                );
                renderer.draw_text(
                    name,
                    10,
                    row_height as i32 * (idx as i32 - start as i32),
                    row_height as f32,
                    display::WHITE,
                );
            } else {
                renderer.draw_text(
                    name,
                    10,
                    row_height as i32 * (idx as i32 - start as i32),
                    row_height as f32,
                    display::BLACK,
                );
            }
        }

        renderer.update();
    }

    /// Handles user input
    ///
    /// Return true if the menu should remain open
    pub fn notify_input(&mut self, input: &Input) -> bool {
        // Update cursor
        if input.pressed_down() {
            self.selected += 1;
        }
        if input.pressed_up() {
            self.selected -= 1;
        }

        self.update_bounds();

        if input.pressed_enter() {
            if let Some(item) = self.items.get_mut(self.selected) {
                // Enter the selected menu
                (item.callback)();
            } else {
                eprintln!("Menu cursor in invalid position");
            }
        }

        // If the user pressed left, we should return to the parent menu
        !input.pressed_left()
    }
}
