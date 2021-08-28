use crate::ble::{Characteristic, Service};

use stm32wb55::{
    event::AttributeHandle,
    gatt::{
        CharacteristicEvent, CharacteristicProperty, Commands as GattCommands, DescriptorHandle,
        DescriptorValueParameters, ServiceType, Uuid,
    },
};


/// Device Information Service...
///
// Should these just be an enum?
// Should these be directly Uuid16 types frrom upstream
#[allow(dead_code)]
pub mod uuid {
    pub const DEVICE_INFORMATION_SERVICE: u16 = 0x180a;
    pub const SYSTEM_ID: u16 = 0x2a23;
    pub const MODEL_NUMBER: u16 = 0x2a24;
    pub const SERIAL_NUMBER: u16 = 0x2a25;
    pub const FW_REVISION: u16 = 0x2a26;
    pub const HW_REVISION: u16 = 0x2a27;
    pub const SW_REVISION: u16 = 0x2a28;
    pub const MANUFACTURER_NAME: u16 = 0x2a29;
    pub const IEEE_CERT: u16 = 0x2a2a;
    pub const PNP_ID: u16 = 0x2a50;
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

pub fn init(di: &DeviceInformation) {}

pub struct DisService {
    /// Bluetooth handle of the service
    service: Service,
    pub uuid: u16,
    // pub instance_id: u16,
    // instance_id_characteristic: Characteristic,
}

impl DisService {
    pub fn new(
        uuid: u16,
        max_attribute_records: u8,
        //instance_id: u16,
    ) -> Result<DisService, ()> {
        let service = Service::new(
            ServiceType::Primary,
            Uuid::Uuid16(uuid),
            max_attribute_records,
        )?;

        // I don't _think_ we need this...
        // let instance_id_characteristic = service.add_characteristic(
        //     &Uuid::Uuid128(UUID_SERVICE_INSTANCE),
        //     CharacteristicProperty::READ,
        //     CharacteristicEvent::CONFIRM_READ | CharacteristicEvent::ATTRIBUTE_WRITE,
        //     2,
        //     false,
        // )?;
        //
        //instance_id_characteristic.set_value(&instance_id.to_le_bytes())?;

        Ok(DisService {
            service,
            uuid,
            //instance_id,
            //instance_id_characteristic,
        })
    }

    pub(crate) fn contains_handle(&self, handle: AttributeHandle) -> bool {
        self.service.contains_handle(handle)
    }
}

// type CharWriteHandler = fn(&mut DisCharacteristic, tid: u8, data: Option<&[u8]>) -> Result<(), ()>;


pub struct DisCharacteristic {
    characteristic: Characteristic,
    pub uuid: u16,

    // read_handler: fn(&mut Self, request: &HapRequest) -> Result<(), ()>,
    //
    // write_handler: CharWriteHandler,
}

impl DisCharacteristic {
    pub fn build(
        service: &DisService,
        uuid: u16,
        ble_properties: CharacteristicProperty,
        characteristic_len: usize,
        //read_handler: fn(&mut Self, request: &HapRequest) -> Result<(), ()>,
        //write_handler: CharWriteHandler,
    ) -> Result<Self, ()> {
        let characteristic = service.service.add_characteristic(
            &Uuid::Uuid16(uuid),
            ble_properties,
            CharacteristicEvent::empty(),
            characteristic_len,
            true,
        )?;

        Ok(DisCharacteristic {
            characteristic,
            uuid,
            // read_handler,
            // write_handler,
        })
    }

    pub fn set_value(&self, value: &[u8]) -> Result<(), ()> {
        /*
        rprintln!(
            "{:?}: value={:x?}",
            self.characteristic.characteristic,
            value
        );
        */

        self.characteristic.set_value(value)
    }
}
