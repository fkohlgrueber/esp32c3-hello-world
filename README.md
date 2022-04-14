# esp32-c3 hello world

Minimal hello world example project for bare-metal no_std esp32-c3. Heavily copied from [esp-hal](https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal).

## Preparation

- Install Rust
- Add riscv target (`rustup target add riscv32imc-unknown-none-elf`)
- Install espflash (`cargo install espflash`)

## Build & Run

```
cargo build
cargo run
```