use anyhow::Result;

use embedded_svc::http::Method;
use esp_idf_svc::http::client::{Configuration, EspHttpConnection};

const BUFFER_SIZE: usize = 1024;

pub fn get(url: impl AsRef<str>) -> Result<Option<String>> {
    let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
    let mut client = EspHttpConnection::new(&Configuration::default())?;
    #[allow(unused_mut)]
    let mut total_size = 0;

    client.initiate_request(Method::Get, url.as_ref(), &[("Content-Type", "text/html")])?;
    let status_code = client.status();

    match status_code {
        200..=299 => {
            let (conn, _) = client.split();

            loop {
                if let Ok(size) = client.read(&mut buffer) {
                    if size == 0 {
                        break Ok(None);
                    }
                    // total_size += size;

                    let response_text = std::str::from_utf8(&buffer[..size])?;

                    return Ok(Some(String::from(response_text)));
                }
            }
        }
        _ => anyhow::bail!("Unexpected Response code: {}", status_code),
    }
}
