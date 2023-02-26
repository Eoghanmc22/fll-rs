pub use fll_rs_core::*;

#[cfg(target_arch = "arm")]
pub use fll_rs_lego::*;

#[cfg(not(target_arch = "arm"))]
pub use fll_rs_mock::*;
