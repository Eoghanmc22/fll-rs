use std::mem;

/// Repersents which butons are pressed on the robot
#[derive(Clone, Default)]
pub struct Input {
    // up; down; left; right; enter; backspace
    current_input: [bool; 6],
    last_input: [bool; 6],
}

impl Input {
    pub const UP: usize = 0;
    pub const DOWN: usize = 1;
    pub const LEFT: usize = 2;
    pub const RIGHT: usize = 3;
    pub const ENTER: usize = 4;
    pub const BACKSPACE: usize = 5;

    /// Update with new data
    pub fn update(&mut self, new_state: [bool; 6]) {
        let last_input = mem::replace(&mut self.current_input, new_state);
        self.last_input = last_input;
    }

    pub fn is_up(&self) -> bool {
        self.current_input[Input::UP]
    }

    pub fn pressed_up(&self) -> bool {
        self.current_input[Input::UP] && !self.last_input[Input::UP]
    }

    pub fn released_up(&self) -> bool {
        !self.current_input[Input::UP] && self.last_input[Input::UP]
    }

    pub fn is_down(&self) -> bool {
        self.current_input[Input::DOWN]
    }

    pub fn pressed_down(&self) -> bool {
        self.current_input[Input::DOWN] && !self.last_input[Input::DOWN]
    }

    pub fn released_down(&self) -> bool {
        !self.current_input[Input::DOWN] && self.last_input[Input::DOWN]
    }

    pub fn is_left(&self) -> bool {
        self.current_input[Input::LEFT]
    }

    pub fn pressed_left(&self) -> bool {
        self.current_input[Input::LEFT] && !self.last_input[Input::LEFT]
    }

    pub fn released_left(&self) -> bool {
        !self.current_input[Input::LEFT] && self.last_input[Input::LEFT]
    }

    pub fn is_right(&self) -> bool {
        self.current_input[Input::RIGHT]
    }

    pub fn pressed_right(&self) -> bool {
        self.current_input[Input::RIGHT] && !self.last_input[Input::RIGHT]
    }

    pub fn released_right(&self) -> bool {
        !self.current_input[Input::RIGHT] && self.last_input[Input::RIGHT]
    }

    pub fn is_enter(&self) -> bool {
        self.current_input[Input::ENTER]
    }

    pub fn pressed_enter(&self) -> bool {
        self.current_input[Input::ENTER] && !self.last_input[Input::ENTER]
    }

    pub fn released_enter(&self) -> bool {
        !self.current_input[Input::ENTER] && self.last_input[Input::ENTER]
    }

    pub fn is_backspace(&self) -> bool {
        self.current_input[Input::BACKSPACE]
    }

    pub fn pressed_backspace(&self) -> bool {
        self.current_input[Input::BACKSPACE] && !self.last_input[Input::BACKSPACE]
    }

    pub fn released_backspace(&self) -> bool {
        !self.current_input[Input::BACKSPACE] && self.last_input[Input::BACKSPACE]
    }
}
