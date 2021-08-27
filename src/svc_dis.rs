
/// Device Information Service...
///
pub mod dis {
    // Should these just be an enum?
    pub mod uuid {
        const DEVICE_INFORMATION_SERVICE: u16 = 0x180a;
        const SYSTEM_ID: u16 = 0x2a23;
        const MODEL_NUMBER: u16 = 0x2a24;
        const SERIAL_NUMBER: u16 = 0x2a25;
        const FW_REVISION: u16 = 0x2a26;
        const HW_REVISION: u16 = 0x2a27;
        const SW_REVISION: u16 = 0x2a28;
        const MANUFACTURER_NAME: u16 = 0x2a29;
        const IEEE_CERT: u16 = 0x2a2a;
        const PNP_ID: u16 = 0x2a50;
    }

    pub struct DeviceInformation<'a> {
        pub manufacturer_name: Option<&'a str>,
        pub model_number: Option<&'a str>,
        pub serial_number: Option<&'a str>,
        pub hw_revision: Option<&'a str>,
        pub fw_revision: Option<&'a str>,
        pub sw_revision: Option<&'a str>,
        pub system_id: Option<&'a str>,
        pub ieee_cert: Option<&'a str>,
        pub pnp_id: Option<&'a str>,
    }

    pub fn init(di: &DeviceInformation) {

    }

}