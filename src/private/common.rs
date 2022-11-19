// impl From<ipv4::IpInfo> for Newtype<esp_netif_ip_info_t> {
//     fn from(ip_info: ipv4::IpInfo) -> Self {
//         Newtype(esp_netif_ip_info_t {
//             ip: Newtype::<esp_ip4_addr_t>::from(ip_info.ip).0,
//             netmask: Newtype::<esp_ip4_addr_t>::from(ip_info.subnet.mask).0,
//             gw: Newtype::<esp_ip4_addr_t>::from(ip_info.subnet.gateway).0,
//         })
//     }
// }
pub struct Newtype<T>(pub T);
