use crate::{DISABLE_HTTPD, DISABLE_SNTP, FALLBACK_TO_RTC, UPDATE_RTC};
use log::{info, warn};

pub mod internal;

pub fn update_rtc_enable() {
    let lock = UPDATE_RTC.try_lock();
    if let Ok(mut mutex) = lock {
        *mutex = true;

        info!("UPDATE_RTC now set to {}", *mutex);
    } else {
        warn!("try_lock failed: UPDATE_RTC mutex, when attempting to enable flag value");
    }
}

pub fn update_rtc_disable() {
    let mut flag = UPDATE_RTC
        .lock()
        .expect("Could not lock UPDATE_RTC mutex, when attempting to disable flag");
    *flag = false;
}

pub fn get_update_rtc_flag() -> bool {
    let mut resp = false;

    let lock = UPDATE_RTC.try_lock();
    if let Ok(mut mutex) = lock {
        let value = *mutex;
        resp = value.clone();

        std::mem::drop(mutex);
    } else {
        println!("try_lock failed: UPDATE_RTC mutex, when attempting to read flag value");
    }

    info!("get_update_rtc_flag: {}", resp);
    resp
}

pub fn fallback_to_rtc_enable() {
    let mut flag = FALLBACK_TO_RTC
        .lock()
        .expect("Could not lock FALLBACK_TO_RTC mutex, when attempting to enable flag");
    *flag = true;
}

pub fn fallback_to_rtc_disable() {
    let mut flag = FALLBACK_TO_RTC
        .lock()
        .expect("Could not lock FALLBACK_TO_RTC mutex, when attempting to disable flag");
    *flag = false;
}

pub fn get_fallback_to_rtc_flag() -> bool {
    let flag = FALLBACK_TO_RTC
        .lock()
        .expect("Could not lock FALLBACK_TO_RTC mutex, when attempting to read flag value");

    let value = *flag;
    std::mem::drop(flag);

    value
}

// DISABLE_SNTP

pub fn disable_sntp() {
    let mut flag = DISABLE_SNTP
        .lock()
        .expect("Could not lock DISABLE_SNTP mutex, when attempting to enable flag");
    *flag = true;
}

pub fn enable_sntp() {
    let mut flag = DISABLE_SNTP
        .lock()
        .expect("Could not lock DISABLE_SNTP mutex, when attempting to disable flag");
    *flag = false;
}

/// Returns the value of the `DISABLE_SNTP` flag.
///
/// - false: SNTP is enabled.
/// - true: SNTP is disabled.
pub fn get_disable_sntp_flag() -> bool {
    let flag = DISABLE_SNTP
        .lock()
        .expect("Could not lock DISABLE_SNTP mutex, when attempting to read flag value");

    let value = *flag;
    std::mem::drop(flag);

    value
}

// DISABLE_HTTPD

pub fn disable_httpd() {
    let mut flag = DISABLE_HTTPD
        .lock()
        .expect("Could not lock DISABLE_HTTPD mutex, when attempting to enable flag");
    *flag = true;
}

pub fn enable_httpd() {
    let mut flag = DISABLE_HTTPD
        .lock()
        .expect("Could not lock DISABLE_HTTPD mutex, when attempting to disable flag");
    *flag = false;
}

/// Returns the value of the `DISABLE_HTTPD` flag.
///
/// - false: HTTPD is enabled.
/// - true: HTTPD is disabled.
pub fn get_disable_httpd_flag() -> bool {
    let flag = DISABLE_HTTPD
        .lock()
        .expect("Could not lock DISABLE_HTTPD mutex, when attempting to read flag value");

    let value = *flag;
    std::mem::drop(flag);

    value
}
