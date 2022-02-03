# `rp2040-embassy-example`

> Example project for Raspberry Pi PICO that uses [embassy](https://github.com/embassy-rs/embassy/) and [defmt](https://github.com/knurling-rs/defmt)

Based on examples from within the embassy project.

## Hardware Setup
  - Program a Raspberry Pi PICO to be a CMSIS-DAP probe using [DapperMime](https://github.com/majbthrd/DapperMime)
    - Download from [DapperMime Releases](https://github.com/majbthrd/DapperMime/releases)
    - Install onto the PICO you'll use as the probe
  - Connect the probe PICO to the PICO you'll be programming
    - [digikey guide](https://www.digikey.com/en/maker/projects/raspberry-pi-pico-and-rp2040-cc-part-2-debugging-with-vs-code/470abc7efb07432b82c95f6f67f184c0) covers this well, though you don't need to connect the UART between the devices.
    - in short:

| Probe PICO | PICO to be programmed |
| ---------- | --------------------- |
| VSYS       | VSYS                  |
| GND        | GND                   |
| GP2        | SWCLK                 |
| GP3        | SWDIO                 |

## Software Setup

- Rust
  - Install [rustup](https://rustup.rs/)
  - `rustup self update` - update rustup
  - `rustup update nightly` - update rust nightly
  - `rustup target add thumbv6m-none-eabi` - add cortex m0 compilation target for cross-compiling

- Rust Tools
  - `cargo install flip-link` - linker to enable [flip-link](https://github.com/knurling-rs/flip-link)
  - `cargo install probe-run` - run using probe (default, configured runner in .cargo/config.toml)
  - `cargo install elf2uf2-rs --locked` - build uf2 files (configure runner in .cargo/config.toml)
  - probe-rs (not currently working; don't use)
    - `cargo install --force --git https://github.com/probe-rs/probe-rs probe-rs-debugger`


- VS Code
  - Install [VS Code](https://code.visualstudio.com/)
  - Extensions:
    - [rust-analyzer](https://marketplace.visualstudio.com/items?itemName=matklad.rust-analyzer)
    - [PIO ASM Syntax Highlighting](https://marketplace.visualstudio.com/items?itemName=chris-hock.pioasm)

- Rust helper commands (only used to follow cortex_m_quickstart, used to build this example; not needed to use it)
  - `cargo install cargo-edit`
  - `cargo install cargo-generate`

## Running

`cargo run`

That's about it!

## Helpful Documentation

High-level documentation:
  - [rust book](https://doc.rust-lang.org/book/)
  - [rust embedded book](https://rust-embedded.github.io/book)

Setup Guide for rp-rs (covers getting rust running on the pi pico or similar boards, including probe-run, elf2uf2-rs, etc)
  - [rp-rs README.md](https://github.com/rp-rs/rp-hal)

Other useful information:
  - [Writing embedded drivers in rust](https://hboeving.dev/blog/rust-2c-driver-p1/)
  - [Embedonomicon](https://docs.rust-embedded.org/embedonomicon/preface.html)
  - [Embedonomicon - Concurrency](https://japaric.github.io/embedonomicon/concurrency.html)
  - [embedded-hal Documentation](https://docs.rs/embedded-hal/latest/embedded_hal/)
  - [nb (non-blocking abstraction used by embedded-hal)](https://docs.rs/nb/latest/nb/)

# TODO

- get breakpoints/debugging working

# License

This project is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)

- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.
