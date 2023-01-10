use std::cell::{RefCell, RefMut};

use crate::{
    core::internal::{self, pwm::DEFAULT_DUTY_CYCLE, ws},
    services::httpd::LazyInitHttpServer,
};
use anyhow::Result;
use http::status::StatusCode;
use serde_json::json;

use embassy_sync::blocking_mutex::Mutex;
use embedded_svc::{
    http::{server::Middleware, Method},
    utils::{asyncify::ws::server::Processor, mutex::RawCondvar},
    ws::asynch::server::Acceptor,
};
use esp_idf_hal::task::embassy_sync::EspRawMutex;
use esp_idf_svc::http::server::{
    fn_handler,
    ws::{EspHttpWsConnection, EspHttpWsProcessor},
    EspHttpConnection, EspHttpServer,
};
use middleware::DefaultMiddleware;

pub mod middleware;

pub fn configure_handlers<'a>(httpd: &mut RefMut<'_, EspHttpServer>) -> Result<()> {
    // HTTPd
    httpd.handler(
        "/health",
        Method::Get,
        DefaultMiddleware {}.compose(
            //
            fn_handler(|request| {
                request
                    .into_response(StatusCode::OK.as_u16(), Some(StatusCode::OK.as_str()), &[])
                    .expect("Response for /health");

                Ok(())
            }),
        ),
    )?;

    #[cfg(feature = "pwm")]
    httpd.handler(
        "/pwm",
        Method::Post,
        DefaultMiddleware {}.compose(
            //
            fn_handler(|mut request| {
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
                            respond_err(
                                conn_mut,
                                StatusCode::BAD_REQUEST,
                                "Body must be valid JSON",
                            )?;

                            return Ok(());
                        }
                    };
                    log::info!("json: {:?}", json);

                    let mut duty_cycle = DEFAULT_DUTY_CYCLE;
                    if let Some(value) = json["pwm"]["duty_cycle"].as_u64() {
                        duty_cycle = value as u32;

                        if duty_cycle > 100 {
                            respond_err(
                                conn_mut,
                                StatusCode::BAD_REQUEST,
                                "Duty-cycle percentage must be less than or equal to 100%",
                            )?;

                            return Ok(());
                        }
                    }

                    // NOTE: Signal change of PWM duty-cycle
                    internal::pwm::COMMAND
                        .signal(internal::pwm::PwmCommand::SetDutyCycle(duty_cycle));

                    // Response
                    let json = json!({
                        "code": 200,
                        "success": true,
                        "processed_command": json
                    });
                    let text = json.to_string();

                    let status = Some(StatusCode::OK.as_str());
                    conn_mut.initiate_response(
                        200,
                        status,
                        &[("Content-Type", "application/json")],
                    )?;

                    conn_mut.write(text.as_bytes())?;
                } else {
                    respond_err(conn_mut, StatusCode::BAD_REQUEST, "Body is missing")?;
                }

                Ok(())
            }),
        ),
    )?;

    Ok(())
}

pub fn configure_websockets<'a>(server: &mut LazyInitHttpServer) -> Result<impl Acceptor> {
    let mut httpd = server.create();

    // Websockets
    let (ws_processor, ws_acceptor) =
        EspHttpWsProcessor::<{ ws::WS_MAX_CONNECTIONS }, { ws::WS_MAX_FRAME_LEN }>::new(());

    let ws_processor = Mutex::<EspRawMutex, _>::new(RefCell::new(ws_processor));

    httpd.ws_handler("/ws", move |connection| {
        ws_processor.lock(|ws_processor| ws_processor.borrow_mut().process(connection))
    })?;

    Ok(ws_acceptor)
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
