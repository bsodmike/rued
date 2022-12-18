use log::info;
use serde_json::json;

use embedded_svc::{
    http::{
        headers::content_type,
        server::{CompositeHandler, Connection, Handler, HandlerResult, Middleware, Request},
        Query,
    },
    io::Write,
};

#[derive(Copy, Clone)]
pub struct DefaultMiddleware {}

impl<C> Middleware<C> for DefaultMiddleware
where
    C: Connection,
{
    fn handle<'a, H>(&'a self, connection: &'a mut C, handler: &'a H) -> HandlerResult
    where
        H: Handler<C>,
    {
        let req = Request::wrap(connection);

        info!("DefaultMiddleware called with uri: {}", req.uri());

        let connection = req.release();

        if let Err(error) = handler.handle(connection) {
            if !connection.is_response_initiated() {
                let mut resp = Request::wrap(connection).into_response(
                    500,
                    Some("Internal Error"),
                    &[content_type("application/json")],
                )?;

                let json = json!({
                    "code": 500,
                    "success": false,
                    "error": &error.to_string()
                });
                write!(resp, "{}", json.to_string())?;
            } else {
                // Nothing can be done as the error happened after the response was initiated, propagate further
                return Err(error);
            }
        }

        Ok(())
    }

    fn compose<H>(self, handler: H) -> CompositeHandler<Self, H>
    where
        H: Handler<C>,
        Self: Sized,
    {
        CompositeHandler::new(self, handler)
    }
}

#[derive(Copy, Clone)]
pub struct SimpleMiddleware {}

impl<C> Middleware<C> for SimpleMiddleware
where
    C: Connection,
{
    fn handle<'a, H>(&'a self, connection: &'a mut C, handler: &'a H) -> HandlerResult
    where
        H: Handler<C>,
    {
        let req = Request::wrap(connection);

        info!("SimpleMiddleware called with uri: {}", req.uri());
        let connection = req.release();

        handler.handle(connection)
    }

    fn compose<H>(self, handler: H) -> CompositeHandler<Self, H>
    where
        H: Handler<C>,
        Self: Sized,
    {
        CompositeHandler::new(self, handler)
    }
}
