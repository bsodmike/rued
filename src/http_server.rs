// use anyhow::Result;
// use log::info;
// use serde_json::json;

// use crate::http::registry::Registry;
// use crate::http::server::EspHttpServer;
// use embedded_svc::{
//     http::{
//         server::{Headers, Request as ServerRequest, Response as ServerResponse},
//         SendHeaders, SendStatus,
//     },
//     io::Write,
// };

// pub fn configure_handlers(server: &mut EspHttpServer) -> Result<()> {
//     server.handle_get("/health", move |_request, mut response| {
//         response.set_ok();

//         let mut writer = response.into_writer()?;
//         let buf: [u8; 0] = [];
//         writer.write(&buf)?;

//         Ok(())
//     })?;

//     server.handle_get("/test", move |request, mut response| {
//         response.set_content_type("application/json");

//         // let req_id = request.get_request_id();
//         info!("Request Details:");

//         let header_type = request.header("Content-type");
//         if let Some(value) = header_type {
//             info!("{}", format!("Header \"Content-type\": {:?}", value));
//         }

//         let query_string = request.query_string();
//         if !query_string.is_empty() {
//             info!("{}", format!("Query: {:?}", query_string));
//         }

//         // fetch url
//         let resp = crate::http_client::get("http://info.cern.ch/")?;
//         let body = if let Some(body) = resp {
//             println!("Response body: {}", body);
//             body
//         } else {
//             String::default()
//         };

//         let mut writer = response.into_writer()?;

//         let json = json!({
//             "code": 200,
//             "success": true,
//             "body": body
//         });
//         writer.write_all(json.to_string().as_bytes())?;

//         Ok(())
//     })?;

//     Ok(())
// }
