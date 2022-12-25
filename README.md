# rued
[Ru]st [E]lectronic [D]atalogger - my explorations into an embedded, async, real-world project

This is heavily inspired by [ruwm](https://github.com/ivmarkov/ruwm) by [ivmarkov](https://github.com/ivmarkov).

If you have questions, [find me on Discord](https://discord.gg/rust-lang-community) in the `#embedded` channel or in `#esp-rs` on [Matrix](https://matrix.to/#/#esp-rs:matrix.org)

## Yet another IoT stack?

With any IoT 'stack', there are some rudimentary basics; this repo is an attempt at building these out using the [embedded_svc](https://github.com/esp-rs/embedded-svc) library and its family of crates from Espressif, such as [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc), [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys), and [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal).

### Feature list

| Done? | Feature | Notes |
|-----|---|---|
| [X] | RTOS: High priority executor |  |
| [X] | Wifi |  |
| [X] | Httpd server with TLS |  |
| [X] | Httpd server Middleware | Added to provide JSON responses for matching `Content-Type` headers|
| [X] | SNTP | *Fallback to RTC is pending |
| [X] | Ext. RTC | [RV8803 module used.](https://www.sparkfun.com/products/16281) |
| [Testing] | Display: I2C | **Requires further testing. |
| [X] | Display: SPI | ili9342 fully tested. |
| [Testing] | SD/MMC Card: SPI | **Requires further testing. |
| [ ] | MQTT |  |


## Quick setup

- Install rustup, preferrably in linux.  Using an x86 CPU will help a great deal. Attempts on an [Apple M1 Max failed terribly](https://desilva.io/posts/rust-for-embedded-is-better-on-x86) for me.
- Build your ESP toolchain by following: https://github.com/esp-rs/rust-build or _even better_ use `espup`.
- Run `cargo +esp build --release`
- Use the following:

```
// flash
scp 10.0.3.70:~/esp/rued/target/xtensa-esp32-espidf/release/rued-esp32 . && espflash flash --baud 920000 --monitor rued-esp32

// monitor for stack-traces
scp 10.0.3.70:~/esp/rued/target/xtensa-esp32-espidf/release/rued-esp32 . && espflash monitor --baud 115200 --elf rued-esp32
// You will need to restart the chip
```

### VSCode

Add the following to settings for `rust-analyzer`
```javascript
{
    "rust-analyzer.server.extraEnv": {"LIBCLANG_PATH": "/home/mdesilva/.espressif/tools/xtensa-esp32-elf-clang/esp-15.0.0-20221014-x86_64-unknown-linux-gnu/esp-clang/lib/"},
}
```

### Upgrading to ESP-IDF 5 (and beyond)
```
espup install --esp-idf-version 5.0
export LIBCLANG_PATH=$HOME/.espressif/tools/xtensa-esp32-elf-clang/esp-15.0.0-20221014-x86_64-unknown-linux-gnu/esp-clang/lib/

# cd into your app root
cargo clean
rm -rf .embuild
cargo build --release

# check for `~/.espressif/esp-idf/release-v5.0`
```

## Semantic Versioning (SemVer)

This project [implements SemVer](https://semver.org/). Please pay attension to _Major_ version changes as these largely include backwards incompatible changes introduced to the public API.
