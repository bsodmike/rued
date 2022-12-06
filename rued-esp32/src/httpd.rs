use crate::core::internal::{self, pwm::DEFAULT_DUTY_CYCLE};
use anyhow::Result;
use embedded_svc::http::Method;
use http::status::StatusCode;
use log::info;
use serde_json::json;

use esp_idf_svc::http::{server::EspHttpConnection, server::EspHttpServer};

pub fn configure_handlers(httpd: &mut EspHttpServer) -> Result<()> {
    httpd.fn_handler("/health", Method::Get, move |request| {
        request
            .into_response(StatusCode::OK.as_u16(), Some(StatusCode::OK.as_str()), &[])
            .expect("Response for /health");

        Ok(())
    })?;

    httpd.fn_handler("/pwm", Method::Post, move |mut request| {
        let (conn, conn_mut) = request.split();

        const BUFFER_SIZE: usize = 1024;
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

        let uri = conn.uri();
        log::info! {"Uri: {}", uri};

        if let Some(ct) = conn.header("Content-Type") {
            log::info! {"Content-Type: {}", ct};
        } else {
            respond_err(
                conn_mut,
                StatusCode::BAD_REQUEST,
                "Content-Type header is required",
            )?;

            return Ok(());
        }

        // FIXME
        // And one more: even if you passed a big-enough buffer to read, you have no warranty that all of the input will be read in a single pass. Mentioning this as you might stumble on that next. :) Basically you have to read in a loop, until read returns you 0 bytes read (with a non-empty buffer, that is). STD had something like read_fully or whatever and a bunch of utilities for working with Vec. You can transform the native embedded-io Read into STD Read to use those. But read_fully is also dangerous, as then malicious folks can crash your firmware with out of mem
        let body_size = conn_mut.read(&mut buffer)?;
        if body_size > 0 {
            let mut body = String::from_utf8(buffer.to_vec())?;
            body = body.replace("\0", "");

            let json: serde_json::Value;
            match serde_json::from_str(&body) {
                Ok(value) => json = value,
                Err(error) => {
                    respond_err(conn_mut, StatusCode::BAD_REQUEST, "Body must be valid JSON")?;

                    return Ok(());
                }
            };
            log::info!("json: {:?}", json);

            let mut duty_cycle = DEFAULT_DUTY_CYCLE;
            if let Some(value) = json["pwm"]["duty_cycle"].as_u64() {
                duty_cycle = value as u32;
            }

            // NOTE: Signal change of PWM duty-cycle
            internal::pwm::COMMAND.signal(internal::pwm::PwmCommand::SetDutyCycle(duty_cycle));

            // Response
            let json = json!({
                "code": 200,
                "success": true,
                "processed_command": json
            });
            let text = json.to_string();

            let status = Some(StatusCode::OK.as_str());
            conn_mut.initiate_response(200, status, &[("Content-Type", "application/json")])?;

            conn_mut.write(text.as_bytes())?;
        } else {
            respond_err(conn_mut, StatusCode::BAD_REQUEST, "Body is missing")?;
        }

        Ok(())
    })?;

    // httpd.fn_handler("/dummy", Method::Post, move |mut request| {
    //     const BUFFER_SIZE: usize = 1024;
    //     let (conn, conn_mut) = request.split();
    //     let uri = conn.uri();
    //     // let _conn_raw = conn_mut.raw_connection()?;

    //     let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

    //     if let Some(ct) = conn.header("Content-Type") {
    //         log::info! {"Content-Type: {}", ct};
    //     }
    //     log::info! {"Uri: {}", uri};

    //     // FIXME
    //     // And one more: even if you passed a big-enough buffer to read, you have no warranty that all of the input will be read in a single pass. Mentioning this as you might stumble on that next. :) Basically you have to read in a loop, until read returns you 0 bytes read (with a non-empty buffer, that is). STD had something like read_fully or whatever and a bunch of utilities for working with Vec. You can transform the native embedded-io Read into STD Read to use those. But read_fully is also dangerous, as then malicious folks can crash your firmware with out of mem
    //     let body_size = conn_mut.read(&mut buffer)?;
    //     // log::info!("Body buffer: {:?}", &buffer);
    //     if body_size > 0 {
    //         let body = String::from_utf8(buffer.to_vec())?;
    //         log::info!("Body: {}", body);
    //     }

    //     // Response
    //     let json = json!({
    //         "code": 200,
    //         "success": true,
    //         "body": "test"
    //     });
    //     let text = json.to_string();

    //     conn_mut.initiate_response(200, Some("OK"), &[("Content-Type", "application/json")])?;

    //     conn_mut.write(text.as_bytes())?;

    //     Ok(())
    // })?;

    Ok(())
}

fn respond_err(conn: &mut EspHttpConnection, status: StatusCode, error: &str) -> Result<()> {
    conn.initiate_response(
        status.as_u16(),
        Some(status.as_str()),
        &[("Content-Type", "application/json")],
    )?;

    conn.write(
        json!({
            "code": status.as_u16(),
            "success": false,
            "error": error
        })
        .to_string()
        .as_bytes(),
    )?;

    Ok(())
}
