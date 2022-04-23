# esp32-c3: RGB LED example using the rmt peripheral

Prototypical implementation of using the rmt peripheral of the esp32-c3 to drive an WS2812 rgb led. Works directly on top of [esp-hal](https://github.com/esp-rs/esp-hal/tree/main/esp32c3-hal) and doesn't require esp-idf.

## Preparation

- Install Rust
- Add riscv target (`rustup target add riscv32imc-unknown-none-elf`)
- Install espflash (`cargo install espflash`)

## Build & Run

```
cargo build
cargo run
```