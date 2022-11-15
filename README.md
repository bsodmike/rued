# rust-esp32-xtensa-blinky

This is still largely a work in progress.

If you have questions, [find me on Discord](https://discord.gg/rust-lang-community) in the `#embedded` channel.

## Quick setup

- Install rustup, preferrably in linux.  Using an x86 CPU will help a great deal. Attemps on an [Apple M1 Max failed terribly](https://desilva.io/posts/rust-for-embedded-is-better-on-x86) for me.
- Build your ESP toolchain by following: https://github.com/esp-rs/rust-build
- Run `cargo +esp build --release`
- Use `espflash` to flash your ESP32, make sure you enable the `--monitor` option to watch the output.

Conecting to Wifi can be flakey at times, I usually have to reboot it once after each flashing attempt to connect.

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