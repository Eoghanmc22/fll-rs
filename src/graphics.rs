#[cfg(target_arch = "arm")]
use ev3dev_lang_rust::Screen;
#[cfg(not(target_arch = "arm"))]
use minifb::Window;

pub struct Display {

}

enum Backend {
    #[cfg(target_arch = "arm")]
    Ev3(Screen),
    #[cfg(not(target_arch = "arm"))]
    Mock(Window),
}