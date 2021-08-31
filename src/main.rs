// ~Similar to the ST Heart Rate Sensor example
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_rtt_target as _;
// use panic_halt as _;
use rtt_target::{rprintln, rtt_init_print};

use stm32wb_hal as hal;

use core::time::Duration;

use cortex_m_rt::{entry, exception};
use nb::block;
use byteorder::{ByteOrder, LittleEndian};

use hal::{
    flash::FlashExt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    tl_mbox::{lhci::LhciC1DeviceInformationCcrp, shci::ShciBleInitCmdParam, TlMbox},
};

use bluetooth_hci::{
    event::{command::ReturnParameters, Event},
    host::{uart::Packet, AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType},
    BdAddr,
};

use ble::{perform_command, receive_event, setup_coprocessor, Characteristic, RadioCopro};
use stm32wb55::{
    event::{AttReadPermitRequest, AttributeHandle, GattAttributeModified, Stm32Wb5xEvent},
    gap::{
        AdvertisingDataType, AdvertisingType, AuthenticationRequirements, Commands as GapCommands,
        DiscoverableParameters, LocalName, OutOfBandAuthentication, Pin, Role,
    },
    gatt::{CharacteristicProperty, Commands as GattCommads, UpdateCharacteristicValueParameters},
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
};

mod ble;
mod svc_dis;
mod svc_hrs;
mod bt_appearances;

use svc_dis::{uuid, DeviceInformation, DisCharacteristic, DisService};
use svc_hrs::{HrsService, HrsBodySensorLocation, HrsHrmFlags};
use crate::ble::Service;
use crate::svc_hrs::HrsMeasure;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BT_NAME: &[u8] = b"KToy";
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = BT_NAME.len() as u8;

// const MY_DEVICE_INFO: DeviceInformation = DeviceInformation {
//     fw_revision: Some("fw1.23"),
//     manufacturer_name: Some("demo Company"),
//     model_number: None,
//     serial_number: None,
//     system_id: Some("sysid69"),
//     ieee_cert: None,
//     hw_revision: None,
//     sw_revision: None,
//     pnp_id: None
// };
//

#[entry]
fn entry() -> ! {
    //rtt_init_print!(BlockIfFull, 4096);
    rtt_init_print!(NoBlockSkip, 4096);
    run();

    loop {
        continue;
    }
}

fn run() {
    let dp = hal::device::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.constrain();
    rcc.set_stop_wakeup_clock(StopWakeupClock::HSI16);

    // Fastest clock configuration.
    // * External low-speed crystal is used (LSE)
    // * 32 MHz HSE with PLL
    // * 64 MHz CPU1, 32 MHz CPU2
    // * 64 MHz for APB1, APB2
    // * HSI as a clock source after wake-up from low-power mode
    let clock_config = Config::new(SysClkSrc::Pll(PllSrc::Hse(HseDivider::NotDivided)))
        .with_lse()
        .cpu1_hdiv(HDivider::NotDivided)
        .cpu2_hdiv(HDivider::Div2)
        .apb1_div(ApbDivider::NotDivided)
        .apb2_div(ApbDivider::NotDivided)
        .pll_cfg(PllConfig {
            m: 2,
            n: 12,
            r: 3,
            q: Some(4),
            p: Some(3),
        })
        .rtc_src(RtcClkSrc::Lse)
        .rf_wkp_sel(RfWakeupClock::Lse);

    let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);

    rprintln!("Boot");

    // RTC is required for proper operation of BLE stack
    let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

    let mut ipcc = dp.IPCC.constrain();
    let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

    let config = ShciBleInitCmdParam {
        p_ble_buffer_address: 0,
        ble_buffer_size: 0,
        num_attr_record: 100,
        num_attr_serv: 10,
        attr_value_arr_size: 3500, //2788,
        num_of_links: 8,
        extended_packet_length_enable: 1,
        pr_write_list_size: 0x3A,
        mb_lock_count: 0x79,
        att_mtu: 312,
        slave_sca: 500,
        master_sca: 0,
        ls_source: 1,
        max_conn_event_length: 0xFFFFFFFF,
        hs_startup_time: 0x148,
        viterbi_enable: 1,
        ll_only: 0,
        hw_version: 0,
    };

    setup_coprocessor(config, ipcc, mbox);

    // enable interrupts -> interrupts are enabled in Ipcc::init(), which is called TlMbox::tl_init

    // Boot CPU2
    hal::pwr::set_cpu2(true);

    let ready_event = block!(receive_event());

    rprintln!("Received packet: {:?}", ready_event);

    rprintln!("Resetting processor...");

    let reset_response = perform_command(|rc| rc.reset()).expect("Failed to reset processor");

    rprintln!("Received packet: {:?}", reset_response);

    init_gap_and_gatt().expect("Failed to initialize GAP and GATT");

    rprintln!("Succesfully initialized GAP and GATT");

    let di = DeviceInformation::new(
        Some("klabs"),
        Some("9871234"),
        Some("my-serial"),
        None,
        None,
        Some("fw1.234"),
        Some("my-system-id"),
        None,
        None,
    );

    let dis_service = init_dis(&di).expect("failed to activate DIS");
    let hrs_service = init_hrs().expect("failed to activate heart rate service");

    // Set our discovery parameters, this is "application specific" regardless of what services
    // we've turned on
    perform_command(|rc| {
        rc.set_discoverable(&DISCOVERY_PARAMS)
            .map_err(|_| nb::Error::Other(()))
    });

    loop {
        let response = block!(receive_event());

        rprintln!("Received event: {:x?}", response);

        if let Ok(Packet::Event(event)) = response {
            match event {
                // karl - this isn't quite working...
                Event::DisconnectionComplete(_state) => {
                    // Enter advertising mode again
                    // Put the device in a connectable mode
                    perform_command(|rc| {
                        rc.set_discoverable(&DISCOVERY_PARAMS)
                            .map_err(|_| nb::Error::Other(()))
                    })
                    .expect("Failed to enable discoverable mode again");

                    // perform_command(|rc| {
                    //     rc.update_advertising_data(&ADVERTISING_DATA[..])
                    //         .map_err(|_| nb::Error::Other(()))
                    // })
                    // .expect("Failed to update advertising data");
                }
                // FIXME - I want some sort of "list of event handlers" that can be plugged in here?
                // ST style has a list of handlers, and stops at the first one to say "handled"
                _ => handle_event(&event, &dis_service, &hrs_service),
            }
        }
    }
}

fn handle_event(event: &Event<Stm32Wb5xEvent>, dis: &DisService, hrs: &HrsService) {

    if let Event::Vendor(stm_event) = event {
        match stm_event {
            Stm32Wb5xEvent::AttReadPermitRequest(AttReadPermitRequest {
                                                     conn_handle,
                                                     attribute_handle,
                                                     offset,
                                                 }) => {
                rprintln!("Allowing read on ch: {:?} ah: {:?}, offset: {}", conn_handle, attribute_handle, offset);
                perform_command(|rc| rc.allow_read(*conn_handle))
                    .expect("Failed to allow read");
            }

            other => rprintln!("ignoring event {:?}", other),
        }
    }
}


#[exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
    panic!("Unhandled IRQ: {}", irqn);
}

fn init_gap_and_gatt() -> Result<(), ()> {
    let response = perform_command(|rc: &mut RadioCopro| {
        rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
    })?;

    rprintln!("Response to write_config_data: {:?}", response);

    perform_command(|rc| {
        rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
    })?;

    perform_command(|rc| rc.write_config_data(&ConfigData::identity_root(&get_irk()).build()))?;

    perform_command(|rc| rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build()))?;

    perform_command(|rc| rc.set_tx_power_level(PowerLevel::ZerodBm))?;

    perform_command(|rc| rc.init_gatt())?;

    let return_params =
        perform_command(|rc| rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH))?;

    // let sh, dh, ah == return parameters... if it was of the type of GapInit ReturnParameters....?
    let (service_handle, dev_name_handle, appearence_handle) = if let ReturnParameters::Vendor(
        stm32wb55::event::command::ReturnParameters::GapInit(stm32wb55::event::command::GapInit {
            service_handle,
            dev_name_handle,
            appearance_handle,
            ..
        }),
    ) = return_params
    {
        (service_handle, dev_name_handle, appearance_handle)
    } else {
        rprintln!("Unexpected response to init_gap command");
        return Err(());
    };

    perform_command(|rc| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle,
            characteristic_handle: dev_name_handle,
            offset: 0,
            value: BT_NAME,
        })
        .map_err(|_| nb::Error::Other(()))
    })?;

    let appearance_characteristic = Characteristic {
        service: service_handle,
        characteristic: appearence_handle,
        max_len: 4,
    };

//    appearance_characteristic.set_value(&[0x80, 0x00])?;
    //appearance_characteristic.set_value(bt_appearances::Appearance
    let mut bytes:[u8;2] = [0;2];
    //LittleEndian::write_u16(&mut bytes[0..2], bt_appearances::Sensor::ENERGYMETER.0);
    LittleEndian::write_u16(&mut bytes[0..2], bt_appearances::HeartRateSensor::GENERIC.0);
    appearance_characteristic.set_value(&bytes);
    return Ok(());
}

fn init_dis(di: &DeviceInformation) -> Result<DisService, ()> {
    // homekit demo uses 24, st uses max 19, 2 per char, plus 1.
    // using less than required here saves memory in the shared space I believe, so, ideally, this would
    // check how many "real" values are configured in the "DisServiceInfo" blob....
    let dis_service = DisService::new(svc_dis::uuid::DEVICE_INFORMATION_SERVICE, 19)?;
    di.register(&dis_service);

    // FIXME - neither of these should be in this function, it makes it hard to compose services...
    // Disable scan response.  not even sure we need this at all.
    // perform_command(|rc: &mut RadioCopro| {
    //     rc.le_set_scan_response_data(&[])
    //         .map_err(|_| nb::Error::Other(()))
    // })?;

    return Ok(dis_service);
}

// https://stackoverflow.com/questions/28127165/how-to-convert-struct-to-u8
// except we have no std....
// unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
//     ::std::slice::from_raw_parts(
//         (p as *const T) as *const u8,
//         ::std::mem::size_of::<T>(),
//     )
// }

fn init_hrs() -> Result<HrsService, ()> {
    // analog to hrs_init
    let hrs_service = HrsService::new(true, true, true)?;

    // analog to hrsapp_init...
    if hrs_service.with_location {
        let loc = HrsBodySensorLocation::Finger as u8;
        hrs_service.body_sensor_location.as_ref().unwrap().set_value(&loc.to_le_bytes());
    }

    let mut hrs_measure = HrsMeasure {
        value: 1,
        energy_expended: 100,
        aRR_interval_values: [200],
        valid_intervals: 1,
        flags: HrsHrmFlags::VALUE_FORMAT_UINT16 | HrsHrmFlags::SENSOR_CONTACTS_PRESENT | HrsHrmFlags::SENSOR_CONTACTS_SUPPORTED | HrsHrmFlags::ENERGY_EXPENDED_PRESENT | HrsHrmFlags::RR_INTERVAL_PRESENT,
    };
    // TODO We need to keep that hrs_measure around somewhere, and get our task to start processing periodic events for it....
    let mut bytes:[u8;8] = [0; 8];
    LittleEndian::write_u16(&mut bytes[0..2], hrs_measure.value);
    //bytes[0..2] = *hrs_measure.value.to_le_bytes();
    LittleEndian::write_u16(&mut bytes[2..4], hrs_measure.energy_expended);
    LittleEndian::write_u16(&mut bytes[4..6], hrs_measure.aRR_interval_values[0]);
    bytes[6] = hrs_measure.valid_intervals;
    bytes[7] = hrs_measure.flags.bits();

    hrs_service.heart_rate_measurement.set_value(&bytes);

    return Ok(hrs_service);

}

fn get_bd_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = lhci_info.device_type_id;
    bytes[4] = (lhci_info.st_company_id & 0xff) as u8;
    bytes[5] = (lhci_info.st_company_id >> 8 & 0xff) as u8;

    BdAddr(bytes)
}

fn get_random_addr() -> BdAddr {
    let mut bytes = [0u8; 6];

    let lhci_info = LhciC1DeviceInformationCcrp::new();
    bytes[0] = (lhci_info.uid64 & 0xff) as u8;
    bytes[1] = ((lhci_info.uid64 >> 8) & 0xff) as u8;
    bytes[2] = ((lhci_info.uid64 >> 16) & 0xff) as u8;
    bytes[3] = 0;
    bytes[4] = 0x6E;
    bytes[5] = 0xED;

    BdAddr(bytes)
}

const BLE_CFG_IRK: [u8; 16] = [
    0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0xde, 0xf0,
];
const BLE_CFG_ERK: [u8; 16] = [
    0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21, 0xfe, 0xdc, 0xba, 0x09, 0x87, 0x65, 0x43, 0x21,
];

fn get_irk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_IRK)
}

fn get_erk() -> EncryptionKey {
    EncryptionKey(BLE_CFG_ERK)
}

const DISCOVERY_PARAMS: DiscoverableParameters = DiscoverableParameters {
    advertising_type: AdvertisingType::ConnectableUndirected,
    advertising_interval: Some((
        Duration::from_millis(ADV_INTERVAL_MS),
        Duration::from_millis(ADV_INTERVAL_MS),
    )),
    address_type: OwnAddressType::Public,
    filter_policy: AdvertisingFilterPolicy::AllowConnectionAndScan,
    // Local name should be empty for the device to be recognized as an Eddystone beacon
    local_name: Some(LocalName::Complete(BT_NAME)),
    advertising_data: &[],
    conn_interval: (None, None),
};
