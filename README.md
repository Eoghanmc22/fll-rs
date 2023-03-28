## Cross Compiling

To cross compile, the rust tool chain for `armv5te-musl` needs to be installed

```bash
rustup target add armv5te-unknown-linux-musleabi
```

To tell rust to compile for `armv5te-musl` by default and to set the correct linker
include the following in `.cargo/config.toml`

```toml
[build]
target = "armv5te-unknown-linux-musleabi"

[target.armv5te-unknown-linux-musleabi]
linker = "rust-lld"
```

See https://crates.io/crates/ev3dev-lang-rust for information about cross compiling your code

## Reducing Binary Size

Enabling global lto causes more aggressive dead code elimination

Setting strip to true removes debugging information from the binary

```toml
[profile.release]
lto = true
strip = true
```
