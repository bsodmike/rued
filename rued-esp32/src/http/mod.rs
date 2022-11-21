// https://github.com/esp-rs/embedded-svc/blob/v0.22.3/src/http/server/registry.rs
pub mod registry;

// https://github.com/esp-rs/esp-idf-svc/blob/v0.42.5/src/http/server.rs
pub mod server;

mod private {
    pub mod cstr {
        pub use cstr_core::CString;
    }
    pub mod common {
        use core::cell::UnsafeCell;

        pub struct Newtype<T>(pub T);

        pub struct UnsafeCellSendSync<T>(pub UnsafeCell<T>);

        unsafe impl<T> Send for UnsafeCellSendSync<T> {}
        unsafe impl<T> Sync for UnsafeCellSendSync<T> {}
    }
}
