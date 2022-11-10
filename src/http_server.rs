use anyhow::Result;
use serde_json::json;

use embedded_svc::{
    http::{
        server::{
            registry::Registry, Headers, Request as ServerRequest, Response as ServerResponse,
        },
        SendHeaders,
    },
    io::Write,
};
use esp_idf_svc::http::server::EspHttpServer;

pub fn configure_handlers(mut server: EspHttpServer) -> Result<()> {
    server.handle_get("/test", move |request, mut response| {
        response.set_header("Content-type", "application/json");

        // let req_id = request.get_request_id();
        println!("Request Details:");

        let header_type = request.header("Content-type");
        if let Some(value) = header_type {
            println!("{}", format!("Header \"Content-type\": {:?}", value));
        }

        let query_string = request.query_string();
        if !query_string.is_empty() {
            println!("{}", format!("Query: {:?}", query_string));
        }

        // fetch url
        let resp = crate::http_client::get("http://info.cern.ch/")?;
        let body = if let Some(body) = resp {
            println!("Response body: {}", body);
            body
        } else {
            String::default()
        };

        let mut writer = response.into_writer()?;

        let json = json!({
            "code": 200,
            "success": true,
            "body": body
        });
        writer.write_all(json.to_string().as_bytes())?;

        Ok(())
    })?;

    Ok(())
}
