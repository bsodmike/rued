# rust-esp32-xtensa-blinky

This is still largely a work in progress.

If you have questions, [find me on Discord](https://discord.gg/rust-lang-community) in the `#embedded` channel.

## Yet another IoT stack?

With any IoT 'stack', there are some rudimentary basics; this repo is an attempt at building these out using the [embedded_svc](https://github.com/esp-rs/embedded-svc) library and its family of crates from Espressif, such as [esp-idf-svc](https://github.com/esp-rs/esp-idf-svc), [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys), and [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal).

### Feature list

| Done? | Feature | Notes |
|-----|---|---|
| [ ] | Wifi | *Currently unstable; needs a reboot on first flash. |
| [X] | SNTP |  |
| [X] | Httpd server | Handlers can be customised |
| [X] | Httpd server: JSON responses | Also implemented for default ESP error, if the request `Content-Type` is also JSON. |
| [X] | SNTP fallback to RTC |  |
| [X] | I2C | Added on a per-sensor basis |
| [ ] | Display | Needs testing |
| [ ] | RTOS | Working across multiple threads. |

## Quick setup

- Install rustup, preferrably in linux.  Using an x86 CPU will help a great deal. Attempts on an [Apple M1 Max failed terribly](https://desilva.io/posts/rust-for-embedded-is-better-on-x86) for me.
- Build your ESP toolchain by following: https://github.com/esp-rs/rust-build
- Run `cargo +esp build --release`
- Use `espflash` to flash your ESP32, make sure you enable the `--monitor` option to watch the output.

Conecting to Wifi can be flakey at times, I usually have to reboot it once after each flashing attempt to connect.

## Features

To enable the Httpd server, use the features provided such as `cargo build --features httpd_server_enabled --release`.

## Httpd Server

### GET /test

In this example, it is a quick demo on how to send query strings, custom headers and the request handler makes another HTTP request to `http://info.cern.ch` and returns the response as JSON.

```
curl --location --request GET 'http://<WIFI_IP>/test?bob=foo&baz=bar' \
--header 'Content-type: application/json'

// Success:
{
    "body": "<html><head></head><body><header>\n<title>http://info.cern.ch</title>\n</header>\n\n<h1>http://info.cern.ch - home of the first website</h1>\n<p>From here you can:</p>\n<ul>\n<li><a href=\"http://info.cern.ch/hypertext/WWW/TheProject.html\">Browse the first website",
    "code": 200,
    "success": true
}

// Failure response
{
    "code": 500,
    "error": "ESP_ERR_HTTP_CONNECT",
    "success": false
}
```