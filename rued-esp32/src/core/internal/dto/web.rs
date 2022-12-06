use core::fmt::Debug;

use serde::{Deserialize, Serialize};

use heapless::String;

use edge_frame::dto::Role;

use super::battery::BatteryState;

pub const USERNAME_MAX_LEN: usize = 32;
pub const PASSWORD_MAX_LEN: usize = 32;

#[derive(Clone, PartialEq, Debug, Serialize, Deserialize)]
pub enum WebRequest {
    Authenticate(String<USERNAME_MAX_LEN>, String<PASSWORD_MAX_LEN>),
    Logout,

    BatteryStateRequest,

    WifiStatusRequest,
}

impl WebRequest {
    pub fn role(&self) -> Role {
        match self {
            Self::Authenticate(_, _) => Role::None,
            Self::Logout => Role::None,
            Self::BatteryStateRequest => Role::User,
            Self::WifiStatusRequest => Role::Admin,
        }
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug, Serialize, Deserialize)]
pub enum WebEvent {
    NoPermissions,

    AuthenticationFailed,

    RoleState(Role),

    BatteryState(BatteryState),
    //WifiState(Status),

    // MqttPublishNotification(MessageId),
    // MqttClientNotification(MqttClientNotification),
}

impl WebEvent {
    pub fn role(&self) -> Role {
        match self {
            Self::NoPermissions => Role::None,
            Self::AuthenticationFailed => Role::None,
            Self::RoleState(_) => Role::None,
            Self::BatteryState(_) => Role::User,
            //Self::WifiState(_) => Role::User,
        }
    }
}
