See https://crates.io/crates/ev3dev-lang-rust for information about the underlying api used and cross compiling your code

Including the following in `Cargo.toml` will substantially decrease binary size (and therefore upload times) in release mode

```toml
[profile.release]
strip = true
```
