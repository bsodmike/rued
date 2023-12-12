use std::cell::{RefCell, RefMut};
use std::rc::Rc;

use esp_idf_svc::http::server::{Configuration, EspHttpServer};

pub struct LazyInitHttpServer<'a> {
    data: Rc<RefCell<Option<EspHttpServer<'a>>>>,
    config: Configuration,
}

impl LazyInitHttpServer<'_> {
    pub fn new(config: Configuration) -> Self {
        Self {
            data: Rc::new(RefCell::new(None)),
            config,
        }
    }
    pub fn create(&self) -> RefMut<'_, EspHttpServer> {
        if self.data.borrow().is_none() {
            *self.data.borrow_mut() = Some(EspHttpServer::new(&self.config).unwrap());
        }
        let m = self.data.borrow_mut();
        RefMut::map(m, |m| m.as_mut().unwrap())
    }

    #[allow(dead_code)]
    pub fn get(&self) -> Option<RefMut<'_, EspHttpServer>> {
        let m = self.data.borrow_mut();
        if m.is_some() {
            Some(RefMut::map(m, |m| m.as_mut().unwrap()))
        } else {
            None
        }
    }

    pub fn clear(&self) {
        *self.data.borrow_mut() = None;
    }

    #[allow(dead_code)]
    fn ref_count(&self) -> usize {
        Rc::strong_count(&self.data)
    }
}
