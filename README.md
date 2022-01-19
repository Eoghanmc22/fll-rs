See https://crates.io/crates/ev3dev-lang-rust for information about the underlying api used and cross compiling your code

you should add options like this to your Cargo.toml

```toml
[profile.release]
lto = true
codegen-units = 1
# this line tells the compilet to try to make the binary smaller
# this might not be necessary as it tends to negativly impacts preformance
opt-level = "s"
```