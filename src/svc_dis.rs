use crate::ble::{Characteristic, Service};

use stm32wb55::{
    event::AttributeHandle,
    gatt::{
        CharacteristicEvent, CharacteristicProperty, Commands as GattCommands, ServiceType, Uuid,
    },
};

/// Device Information Service...
///
// Should these just be an enum?
// Should these be directly Uuid16 types frrom upstream
// Karl - I hate how verbose this is, there's gotta be a better way...
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

pub struct UuidStr<'a> {
    pub uuid: u16,
    pub s: Option<&'a str>,
}

impl<'a> UuidStr<'a> {
    pub fn register(&self, dis: &DisService) -> Result<(), ()> {
        if let Some(x) = self.s {
            let this_char =
                DisCharacteristic::build(dis, self.uuid, CharacteristicProperty::READ, x.len())?;
            this_char.set_value(x.as_bytes())?;
        }
        // No real return, unless there was a string and we failed to add it
        Ok(())
    }
}

pub struct DeviceInformation<'a> {
    manufacturer_name: UuidStr<'a>,
    model_number: UuidStr<'a>,
    serial_number: UuidStr<'a>,
    hw_revision: UuidStr<'a>,
    sw_revision: UuidStr<'a>,
    fw_revision: UuidStr<'a>,
    system_id: UuidStr<'a>,
    ieee_cert: UuidStr<'a>,
    pnp_id: UuidStr<'a>,
}

impl<'a> DeviceInformation<'a> {
    pub fn new(
        manufacturer_name: Option<&'a str>,
        model_number: Option<&'a str>,
        serial_number: Option<&'a str>,
        hw_revision: Option<&'a str>,
        sw_revision: Option<&'a str>,
        fw_revision: Option<&'a str>,
        system_id: Option<&'a str>,
        ieee_cert: Option<&'a str>,
        pnp_id: Option<&'a str>,
    ) -> Self {
        return DeviceInformation {
            manufacturer_name: UuidStr {
                uuid: uuid::MANUFACTURER_NAME,
                s: manufacturer_name,
            },
            model_number: UuidStr {
                uuid: uuid::MODEL_NUMBER,
                s: model_number,
            },
            serial_number: UuidStr {
                uuid: uuid::SERIAL_NUMBER,
                s: serial_number,
            },
            hw_revision: UuidStr {
                uuid: uuid::HW_REVISION,
                s: hw_revision,
            },
            sw_revision: UuidStr {
                uuid: uuid::SW_REVISION,
                s: sw_revision,
            },
            fw_revision: UuidStr {
                uuid: uuid::FW_REVISION,
                s: fw_revision,
            },
            system_id: UuidStr {
                uuid: uuid::SYSTEM_ID,
                s: system_id,
            },
            ieee_cert: UuidStr {
                uuid: uuid::IEEE_CERT,
                s: ieee_cert,
            },
            pnp_id: UuidStr {
                uuid: uuid::PNP_ID,
                s: pnp_id,
            },
        };
    }

    pub fn register(&self, dis: &DisService) {
        self.manufacturer_name.register(dis);
        self.model_number.register(dis);
        self.hw_revision.register(dis);
        self.sw_revision.register(dis);
        self.fw_revision.register(dis);
        self.pnp_id.register(dis);
        self.ieee_cert.register(dis);
        self.system_id.register(dis);
        self.serial_number.register(dis);
    }
}

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
            CharacteristicEvent::CONFIRM_READ,
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
