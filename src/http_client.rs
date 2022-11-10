use anyhow::Result;

use embedded_svc::{
    http::client::{Client, Request, RequestWrite, Response, Status},
    io::Read,
};
use esp_idf_svc::http::client::EspHttpClient;

pub fn get(url: impl AsRef<str>) -> Result<Option<String>> {
    // 1. Create a new EspHttpClient. (Check documentation)
    let mut client = EspHttpClient::new_default()?;

    // 2. Open a GET request to `url`
    let request = client.get(url.as_ref())?;

    // 3. Requests *may* send data to the server. Turn the request into a writer, specifying 0 bytes as write length
    // (since we don't send anything - but have to do the writer step anyway)
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/protocols/esp_http_client.html
    // If this were a POST request, you'd set a write length > 0 and then writer.do_write(&some_buf);

    let writer = request.into_writer(0)?;

    // 4. Submit our write request and check the status code of the response.
    // Successful http status codes are in the 200..=299 range.

    let mut response = writer.submit()?;
    let status = response.status();
    let mut _total_size = 0;

    println!("response code: {}\n", status);

    match status {
        200..=299 => {
            // 5. if the status is OK, read response data chunk by chunk into a buffer and print it until done
            let mut buf = [0_u8; 256];
            let mut reader = response.reader();
            loop {
                if let Ok(size) = Read::read(&mut reader, &mut buf) {
                    if size == 0 {
                        break Ok(None);
                    }
                    _total_size += size;

                    // Read raw data from buffer
                    // let reader = BufReader::new(&buf[..size]);
                    // for line in reader.lines() {
                    //     println!("{:?}", line?);
                    // }

                    // 6. try converting the bytes into a Rust (UTF-8) string and print it
                    let response_text = std::str::from_utf8(&buf[..size])?;
                    // println!("{}", response_text);

                    return Ok(Some(String::from(response_text)));
                }
            }
        }
        _ => anyhow::bail!("unexpected response code: {}", status),
    }
}
