use crate::ble::{Characteristic, Service};

use bitflags::bitflags;
use stm32wb55::{
    event::AttributeHandle,
    gatt::{
        CharacteristicEvent, CharacteristicProperty, Commands as GattCommands, ServiceType, Uuid,
    },
};

const UUID_HEART_RATE_SERVICE: u16 = 0x180d;
const UUID_CLIENT_CHAR_CONFIG_DESCRIPTOR: u16 = 0x2902;
const UUID_HEART_RATE_MEASUREMENT: u16 = 0x2a37;
const UUID_SENSOR_LOCATION: u16 = 0x2a38;
const UUID_CONTROL_POINT: u16 = 0x2a39;
const UUID_BM_REQ_CHAR: [u8; 16] = [0x19, 0xed, 0x82, 0xae, 0xed, 0x21, 0x4c, 0x9d, 0x41, 0x45, 0x22, 0x8e, 0x11, 0xFE, 0x00, 0x00];

#[allow(dead_code)]
pub enum HrsBodySensorLocation {
    Other = 0,
    Chest = 1,
    Wrist = 2,
    Finger = 3,
    EarLobe = 5,
    Foot = 6,
}

bitflags! {
    pub struct HrsHrmFlags: u8 {
        const VALUE_FORMAT_UINT16 = 1;
        const SENSOR_CONTACTS_PRESENT = 2;
        const SENSOR_CONTACTS_SUPPORTED = 4;
        const ENERGY_EXPENDED_PRESENT = 8;
        const RR_INTERVAL_PRESENT = 0x10;
    }
}

pub struct HrsService {
    /// Bluetooth handle of the service
    pub service: Service,
    pub heart_rate_measurement: Characteristic,
    pub body_sensor_location: Option<Characteristic>,
    pub control_point: Option<Characteristic>,
    pub with_location: bool,
    pub with_energy: bool,
    pub with_ota: bool,
}

pub struct HrsMeasure {
    pub value: u16,
    pub energy_expended: u16,
    pub aRR_interval_values: [u16; 1], // FIXME - this isn't real, we'r ejust making this up
    pub valid_intervals: u8,
    pub flags: HrsHrmFlags,
}



impl HrsMeasure {
    pub fn to_u8_hack(self) {
        // um... how to we do this?!
    }
}

impl HrsService {
    pub fn new(with_location: bool, with_energy: bool, with_ota: bool) -> Result<HrsService, ()> {
        let mut attrs = 4;
        if with_location { attrs = attrs + 2;}
        if with_energy { attrs = attrs + 2;}
        if with_ota { attrs = attrs + 2;}
        let service = Service::new(
            ServiceType::Primary,
            Uuid::Uuid16(UUID_HEART_RATE_SERVICE),
            attrs,
        )?;

        let mut hrm_len = 3;
        // Only allow a singel RR interval at the moment...
        if with_energy { hrm_len = hrm_len + 4;}
        let hrm = service.add_characteristic(
            &Uuid::Uuid16(UUID_HEART_RATE_MEASUREMENT),
            CharacteristicProperty::NOTIFY,
            CharacteristicEvent::empty(),
            hrm_len,
            true,
        )?;

        let mut sensor_location = None;
        if with_location {
            sensor_location = Some(service.add_characteristic(
                &Uuid::Uuid16(UUID_SENSOR_LOCATION),
                CharacteristicProperty::READ,
                CharacteristicEvent::empty(),
                1,
                false,
            )?);
        }

        let mut control_point = None;
        if with_energy {
            control_point = Some(service.add_characteristic(
                &Uuid::Uuid16(UUID_CONTROL_POINT),
                CharacteristicProperty::WRITE,
                CharacteristicEvent::CONFIRM_WRITE,
                1,
                false,
            )?);
        }

        if with_ota {
            service.add_characteristic(
                &Uuid::Uuid128(UUID_BM_REQ_CHAR),
                CharacteristicProperty::WRITE_WITHOUT_RESPONSE,
                CharacteristicEvent::ATTRIBUTE_WRITE,
                3,
                false
            )?;
        }

        Ok(HrsService {
            service,
            heart_rate_measurement: hrm,
            body_sensor_location: sensor_location,
            control_point,
            with_location,
            with_energy,
            with_ota
        })

    }
    fn handle_events() {

    }
}
