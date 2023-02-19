# firmware

## Install dependencies

Install
- [rustup][https://www.rust-lang.org/tools/install]
- OpenOCD, see [the embedded Rust book][embedded Rust book]

Add the target:

``` sh
rustup target add thumbv7em-none-eabihf
```

There are two ways to program and debug the device:
1. Using GDB directly. For installation instructions see [the embedded Rust book][embedded Rust book].
2. Visual Studio Code.

## Run using GDB

Uncomment the correct runner in `.cargo/config.toml`.

``` sh
openocd
cargo run --release
tail -f itm.txt
```

[embedded Rust book]: https://rust-embedded.github.io/book
