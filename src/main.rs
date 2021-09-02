// ~Similar to the ST Heart Rate Sensor example
#![no_main]
#![no_std]
#![allow(non_snake_case)]

use panic_itm as _;
// use panic_halt as _;
//use rtt_target::{rprintln, rtt_init_print};

use stm32wb_hal as hal;

use core::time::Duration;

use cortex_m::peripheral::{ITM, itm};
use cortex_m::iprintln;
use cortex_m_rt::{entry, exception};
use heapless::spsc::Queue;
use nb::block;
use bbqueue::{consts::U514, BBBuffer, ConstBBBuffer};
use byteorder::{ByteOrder, LittleEndian};

use hal::{
    flash::FlashExt,
    prelude::*,
    rcc::{
        ApbDivider, Config, HDivider, HseDivider, PllConfig, PllSrc, RfWakeupClock, RtcClkSrc,
        StopWakeupClock, SysClkSrc,
    },
    pwr::Cpu2LowPowerMode,
    tl_mbox::{lhci::LhciC1DeviceInformationCcrp, shci::ShciBleInitCmdParam, TlMbox},
};

use bluetooth_hci::{
    event::{
        command::{CommandComplete, ReturnParameters},
        Event},
    host::{
        uart::{Hci as UartHci, Packet},
        AdvertisingFilterPolicy, EncryptionKey, Hci, OwnAddressType
    },
    BdAddr,
    ConnectionHandle
};

use stm32wb55::{
    event::{AttReadPermitRequest, AttributeHandle, GattAttributeModified, Stm32Wb5xEvent},
    gap::{
        AdvertisingDataType, AdvertisingType, AuthenticationRequirements, Commands as GapCommands,
        DiscoverableParameters, LocalName, OutOfBandAuthentication, Pin, Role,
    },
    gatt::{AddServiceParameters, CharacteristicProperty, Commands as GattCommads, UpdateCharacteristicValueParameters, Uuid, ServiceHandle, ServiceType, CharacteristicHandle},
    hal::{Commands as HalCommands, ConfigData, PowerLevel},
    RadioCoprocessor,
};

//mod svc_dis;
//mod svc_hrs;
mod bt_appearances;

//use svc_dis::{uuid, DeviceInformation, DisCharacteristic, DisService};
//use svc_hrs::{HrsService, HrsBodySensorLocation, HrsHrmFlags, HrsMeasure};

use rtic::{
    app,
    cyccnt,
    cyccnt::U32Ext,
};

// Needs to hold two packets, at least 257 bytes for biggest possible HCI BLE event + header
pub type HciCommandsQueue =
    Queue<fn(&mut RadioCoprocessor<'static, U514>, &MyAppContext), 32>;

/// Advertisement interval in milliseconds.
const ADV_INTERVAL_MS: u64 = 250;

const BT_NAME: &[u8] = b"KToy2";
const BLE_GAP_DEVICE_NAME_LENGTH: u8 = BT_NAME.len() as u8;

const PERIOD: u32 = 30_000_000;

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



// We'll put stuff we want here right...?
#[derive(Debug, Default)]
pub struct MyAppContext {
    gap_service_handle: Option<ServiceHandle>,
    dis_service_handle: Option<ServiceHandle>,
    hrs_service_handle: Option<ServiceHandle>,
    dev_name_handle: Option<CharacteristicHandle>,
    appearance_handle: Option<CharacteristicHandle>,
    ad_cnt: u8,
}

#[app(device = stm32wb_hal::pac, peripherals = true, monotonic = rtic::cyccnt::CYCCNT)]
const APP: () = {
    struct Resources {
        rc: RadioCoprocessor<'static, U514>,
        hci_commands_queue: HciCommandsQueue,
        app_context: MyAppContext,
        stim0: &'static mut itm::Stim,
    }

    #[init]
    fn init(cx: init::Context) -> init::LateResources {
        //rtt_init_print!(NoBlockSkip, 4096);
        let dp = cx.device;

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
        // let clock_config = Config::new(SysClkSrc::HseSys(HseDivider::NotDivided))
        //     .with_lse()
        //     .cpu1_hdiv(HDivider::NotDivided)
        //     .cpu2_hdiv(HDivider::NotDivided)
        //     .apb1_div(ApbDivider::NotDivided)
        //     .apb2_div(ApbDivider::NotDivided)
            .rtc_src(RtcClkSrc::Lse)
            .rf_wkp_sel(RfWakeupClock::Lse);

        let mut rcc = rcc.apply_clock_config(clock_config, &mut dp.FLASH.constrain().acr);
        let itm = unsafe { &mut *ITM::ptr() };
        let stim0 = &mut itm.stim[0];

        iprintln!(stim0, "Boot!");

        // RTC is required for proper operation of BLE stack
        let _rtc = hal::rtc::Rtc::rtc(dp.RTC, &mut rcc);

        let mut ipcc = dp.IPCC.constrain();
        let mbox = TlMbox::tl_init(&mut rcc, &mut ipcc);

        // Boot CPU2
        hal::pwr::set_cpu2_lpmode(Cpu2LowPowerMode::Shutdown);
        hal::pwr::set_cpu2(true);

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

        static BB: BBBuffer<U514> = BBBuffer(ConstBBBuffer::new());
        let (producer, consumer) = BB.try_split().unwrap();
        // TODO - can't get the mbox out of this again as it's now owned? so can't call wireless_fw_info?
        let rc = RadioCoprocessor::new(producer, consumer, mbox, ipcc, config);

        let mut cp = cx.core;
        cp.SCB.set_sleepdeep();

        init::LateResources {
            rc,
            hci_commands_queue: HciCommandsQueue::new(),
            app_context: MyAppContext::default(),
            stim0,
        }
    }

    #[idle(resources = [rc, app_context], spawn = [setup, exec_hci, event])]
    fn idle(mut cx: idle::Context) -> ! {
        // Idle doesn't have access to resources the same way? hate this though.
        let itm = unsafe { &mut *ITM::ptr() };
        let stim0 = &mut itm.stim[0];
        loop {
            cortex_m::asm::wfi();
            iprintln!(stim0, "woke!");

            // At this point, an interrupt was received.
            // Radio co-processor talks to the app via IPCC interrupts, so this interrupt
            // may be one of the IPCC interrupts and the app can start processing events from
            // radio co-processor here.
            let evt = cx.resources.rc.lock(|rc| {
                if rc.process_events() {
                    // iprintln!(stim0, "processed");
                    Some(block!(rc.read()))
                } else {
                    None
                }
            });

            if let Some(Ok(Packet::Event(evt))) = evt {
                if let Event::Vendor(stm32wb55::event::Stm32Wb5xEvent::CoprocessorReady(_)) = evt {
                    // Setup BLE service when BLE co-processor is ready
                    cx.spawn.setup().unwrap();
                } else {
                    cx.spawn.event(evt).unwrap();
                    cx.spawn.exec_hci().unwrap();
                }
            }
        }
    }

    #[task(resources = [rc, hci_commands_queue, stim0], spawn = [exec_hci, setup_dis], schedule=[update_advertising_hack])]
    fn setup(mut cx: setup::Context) {

        // let rc: RadioCoprocessor<'static, U514> = cx.resources.rc;
        // iprintln!(stim0, "wireless fw info: {:?}", &mbox.wireless_fw_info());


        cx.resources
            .hci_commands_queue
            .enqueue(|rc, _| rc.reset().unwrap())
            .ok();

        init_gap_and_gatt(&mut cx.resources.hci_commands_queue);

        iprintln!(cx.resources.stim0, "gap/gatt init complete");

        // let di = DeviceInformation::new(
        //     Some("klabs"),
        //     Some("9871234"),
        //     Some("my-serial"),
        //     None,
        //     None,
        //     Some("fw1.234"),
        //     Some("my-system-id"),
        //     None,
        //     None,
        // );

        //let dis_service = init_dis(cx.resources.hci_command_queue, &di).expect("failed to activate DIS");
        cx.spawn.setup_dis().unwrap();
        // let hrs_service = init_hrs().expect("failed to activate heart rate service");
        //
        // // Set our discovery parameters, this is "application specific" regardless of what services
        // // we've turned on
        // perform_command(|rc| {
        //     rc.set_discoverable(&DISCOVERY_PARAMS)
        //         .map_err(|_| nb::Error::Other(()))
        // });

        // Execute first HCI command from the queue
        cx.spawn.exec_hci().unwrap();

        cx.schedule.update_advertising_hack(cx.scheduled + PERIOD.cycles()).unwrap();

    }

    /// Executes HCI command from the queue.
    #[task(resources = [rc, hci_commands_queue, app_context, stim0])]
    fn exec_hci(mut cx: exec_hci::Context) {
        // iprintln!(cx.resources.stim0, "exec_hci");
        if let Some(cmd) = cx.resources.hci_commands_queue.dequeue() {
            cmd(&mut cx.resources.rc, &cx.resources.app_context);
        }
    }

    /// Processes BLE events.
    #[task(resources = [app_context, stim0])]
    fn event(mut cx: event::Context, event: Event<stm32wb55::event::Stm32Wb5xEvent>) {
        iprintln!(cx.resources.stim0, "Got event: {:?}", &event);
        if let Event::CommandComplete(CommandComplete { return_params, .. }) = event {
            match return_params {
                ReturnParameters::Vendor(stm32wb55::event::command::ReturnParameters::GapInit(
                                             stm32wb55::event::command::GapInit {
                                                 service_handle,
                                                 dev_name_handle,
                                                 appearance_handle,
                                                 ..
                                             },
                                         )) => {
                    cx.resources.app_context.gap_service_handle = Some(service_handle);
                    cx.resources.app_context.dev_name_handle = Some(dev_name_handle);
                    cx.resources.app_context.appearance_handle = Some(appearance_handle);
                }

                other => {
                    iprintln!(cx.resources.stim0, "unhandled event: {:?}", other);
                },
            }
        }
    }

    #[task(resources = [rc, hci_commands_queue, app_context, stim0])]
    fn setup_dis(mut cx: setup_dis::Context) {
        iprintln!(cx.resources.stim0, "Pretending to setup DIS");
        let hci = cx.resources.hci_commands_queue;
        // ST uses max 19 attrs, 2 per char, + 1
        hci.enqueue(|rc: &mut RadioCoprocessor<'static, U514>, cx| {
                rc.add_service(&AddServiceParameters {
                    service_type: ServiceType::Primary,
                    uuid: Uuid::Uuid16(uuid::DEVICE_INFORMATION_SERVICE),
                    max_attribute_records: 19,
                })
                    .unwrap()
            });

        init_advertising(hci);
    }

    #[task(resources = [rc, hci_commands_queue, app_context, stim0], schedule=[update_advertising_hack])]
    fn update_advertising_hack(mut cx: update_advertising_hack::Context) {
        let ac: &mut MyAppContext = cx.resources.app_context;
        iprintln!(cx.resources.stim0, "updating advertising: {}", ac.ad_cnt);
        let hci = cx.resources.hci_commands_queue;
        // ST uses max 19 attrs, 2 per char, + 1
        let cnt = 42;

        hci.enqueue(|rc: &mut RadioCoprocessor<'static, U514>, cx| {
            let cnt = cx.ad_cnt;
            let mut service_data = [0u8; 10];
            service_data[0] = 9;
            service_data[1] = AdvertisingDataType::ManufacturerSpecificData as u8;
            service_data[2] = 0x99;
            service_data[3] = 0x09; // eTactica ehf company code
            service_data[4] = cnt;
            service_data[5] = 255 - cnt;
            service_data[6] = cnt;
            service_data[7] = 255 - cnt;
            service_data[8] = 0xca;
            service_data[9] = 0xfe;

            rc.update_advertising_data(&service_data)
                    .expect("set scan response data")
            })
            .ok();

        ac.ad_cnt += 1;

            cx.schedule.update_advertising_hack(cx.scheduled + PERIOD.cycles()).unwrap();
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_RX_IT, resources = [rc])]
    fn mbox_rx(cx: mbox_rx::Context) {
        cx.resources.rc.handle_ipcc_rx();
    }

    /// Handles IPCC interrupt and notifies `RadioCoprocessor` code about it.
    #[task(binds = IPCC_C1_TX_IT, resources = [rc])]
    fn mbox_tx(cx: mbox_tx::Context) {
        cx.resources.rc.handle_ipcc_tx();
    }

    // Interrupt handlers used to dispatch software tasks.
    // One per priority.
    extern "C" {
        fn USART1();
    }


};


#[exception]
unsafe fn DefaultHandler(irqn: i16) -> ! {
    panic!("Unhandled IRQ: {}", irqn);
}

fn init_gap_and_gatt(hci_commands_queue: &mut HciCommandsQueue) {

    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::public_address(get_bd_addr()).build())
                .expect("set public address");
        });

    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::random_address(get_random_addr()).build())
                .expect("set random address");
        });

    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::identity_root(&get_irk()).build())
                .expect("set IRK address");
        });
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.write_config_data(&ConfigData::encryption_root(&get_erk()).build())
                .expect("set ERK address");
        });
    hci_commands_queue
        .enqueue(|rc, _| {
            rc.set_tx_power_level(PowerLevel::ZerodBm)
                .expect("set TX power level")
        });
    hci_commands_queue
        .enqueue(|rc, _| rc.init_gatt().expect("GATT init"));

    hci_commands_queue
        .enqueue(|rc, _| {
            rc.init_gap(Role::PERIPHERAL, false, BLE_GAP_DEVICE_NAME_LENGTH)
                .expect("GAP init")
        });
    hci_commands_queue
        .enqueue(|rc, cx| {
            rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
                service_handle: cx.gap_service_handle.expect("service handle to be set"),
                characteristic_handle: cx.dev_name_handle.expect("dev name handle to be set"),
                offset: 0,
                value: BT_NAME,
            })
                .unwrap()
        });

    hci_commands_queue.enqueue(|rc, cx| {
        rc.update_characteristic_value(&UpdateCharacteristicValueParameters {
            service_handle: cx.gap_service_handle.expect("gap sh should be set?"),
            characteristic_handle: cx.appearance_handle.expect("appearance ch should be set?"),
            value: &bt_appearances::HeartRateSensor::GENERIC.0.to_le_bytes(),
            offset: 0,
        }).unwrap()
    });

}

// https://stackoverflow.com/questions/28127165/how-to-convert-struct-to-u8
// except we have no std....
// unsafe fn any_as_u8_slice<T: Sized>(p: &T) -> &[u8] {
//     ::std::slice::from_raw_parts(
//         (p as *const T) as *const u8,
//         ::std::mem::size_of::<T>(),
//     )
// }

// fn init_hrs() -> Result<HrsService, ()> {
//     // analog to hrs_init
//     let hrs_service = HrsService::new(true, true, true)?;
//
//     // analog to hrsapp_init...
//     if hrs_service.with_location {
//         let loc = HrsBodySensorLocation::Finger as u8;
//         hrs_service.body_sensor_location.as_ref().unwrap().set_value(&loc.to_le_bytes());
//     }
//
//     let mut hrs_measure = HrsMeasure {
//         value: 1,
//         energy_expended: 100,
//         aRR_interval_values: [200],
//         valid_intervals: 1,
//         flags: HrsHrmFlags::VALUE_FORMAT_UINT16 | HrsHrmFlags::SENSOR_CONTACTS_PRESENT | HrsHrmFlags::SENSOR_CONTACTS_SUPPORTED | HrsHrmFlags::ENERGY_EXPENDED_PRESENT | HrsHrmFlags::RR_INTERVAL_PRESENT,
//     };
//     // TODO We need to keep that hrs_measure around somewhere, and get our task to start processing periodic events for it....
//     let mut bytes:[u8;8] = [0; 8];
//     LittleEndian::write_u16(&mut bytes[0..2], hrs_measure.value);
//     //bytes[0..2] = *hrs_measure.value.to_le_bytes();
//     LittleEndian::write_u16(&mut bytes[2..4], hrs_measure.energy_expended);
//     LittleEndian::write_u16(&mut bytes[4..6], hrs_measure.aRR_interval_values[0]);
//     bytes[6] = hrs_measure.valid_intervals;
//     bytes[7] = hrs_measure.flags.bits();
//
//     hrs_service.heart_rate_measurement.set_value(&bytes);
//
//     return Ok(hrs_service);
//
// }

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

    fn init_advertising(hci_commands_queue: &mut HciCommandsQueue) {
        hci_commands_queue
            .enqueue(|rc, _| {
                rc.le_set_scan_response_data(&[])
                    .expect("set scan response data")
            })
            .ok();

        // Set discovery, but no advertising data yet.
        hci_commands_queue
            .enqueue(|rc, _| {
                rc.set_discoverable(&DISCOVERY_PARAMS)
                    .expect("set discoverable params")
            })
            .ok();

        // Remove some advertisements (this is done to decrease the packet size)
        hci_commands_queue
            .enqueue(|rc, _| {
                rc.delete_ad_type(AdvertisingDataType::TxPowerLevel)
                    .expect("delete tx power ad type")
            })
            .ok();
        hci_commands_queue
            .enqueue(|rc, _| {
                rc.delete_ad_type(AdvertisingDataType::PeripheralConnectionInterval)
                    .expect("delete conn interval ad type")
            })
            .ok();

    }