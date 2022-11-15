# rust-esp32-xtensa-blinky

This is still largely a work in progress.

If you have questions, [find me on Discord](https://discord.gg/rust-lang-community) in the `#embedded` channel.

## Quick setup

- Install rustup, preferrably in linux.  Using an x86 CPU will help a great deal. Attemps on an [Apple M1 Max failed terribly](https://desilva.io/posts/rust-for-embedded-is-better-on-x86) for me.
- Build your ESP toolchain by following: https://github.com/esp-rs/rust-build
- Run `cargo +esp build --release`
- Use `espflash` to flash your ESP32, make sure you enable the `--monitor` option to watch the output.

Conecting to Wifi can be flakey at times, I usually have to reboot it once after each flashing attempt to connect.