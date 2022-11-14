use crate::{FALLBACK_TO_RTC, UPDATE_RTC};

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
