use crate::{DISABLE_HTTPD, DISABLE_SNTP, FALLBACK_TO_RTC, UPDATE_RTC};

pub fn update_rtc_enable() {
    let mut flag = UPDATE_RTC
        .lock()
        .expect("Could not lock UPDATE_RTC mutex, when attempting to enable flag");
    *flag = true;
}

pub fn update_rtc_disable() {
    let mut flag = UPDATE_RTC
        .lock()
        .expect("Could not lock UPDATE_RTC mutex, when attempting to disable flag");
    *flag = false;
}

pub fn get_update_rtc_flag() -> bool {
    let flag = UPDATE_RTC
        .lock()
        .expect("Could not lock UPDATE_RTC mutex, when attempting to read flag value");

    let value = *flag;
    std::mem::drop(flag);

    value
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
