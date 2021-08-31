// Implementation based on "opcodes.rs" from the bluetooth-hci project.
// Values from BT sig rev 2021-04-20

pub struct Appearance(pub u16);

impl Appearance {
    /// Create an appearance from the category and subcategory.
    pub const fn new(cat: u16, sub: u16) -> Appearance {
        Appearance((cat << 6) | (sub & 0x3f))
    }

    /// Return the Category of the appearance
    pub fn category(&self) -> u16 {
        self.0 >> 6
    }

    /// Return the subcategory of the appearance
    pub fn subcategory(&self) -> u16 {
        self.0 & 0x3f
    }
}

macro_rules! appearances {
    (
        $(
            $_cat_name:ident = $cat:expr;
            {
                $(pub const $var:ident = $sub:expr;)+
            }
        )+
    ) => {
        $(
        pub mod $_cat_name {
            use crate::bt_appearances::Appearance;
            $(
                #[allow(dead_code)]
                pub const $var: Appearance = Appearance::new($cat, $sub);
            )+
        }
        )+
    }
}

// TODO - many of these haven't had the full tables dumped in yet!
appearances! {
    Unknown = 0x000;
    {
        pub const GENERIC = 0x00;
    }

    Phone = 0x001;
    {
        pub const GENERIC = 0x00;
    }

    Computer = 0x002;
    {
        pub const GENERIC = 0x00;
        pub const DESKTOP_WORKSTATION = 0x01;
        pub const SERVER_CLASS_COMPUTER = 0x02;
        pub const LAPTOP = 0x03;
        pub const HANDHELD = 0x04;
        pub const PALMSIZE = 0x05;
        pub const WEARABLE = 0x06;
        pub const TABLET = 0x07;
        pub const DOCKING_STATION = 0x08;
        pub const ALL_IN_ONE = 0x09;
        pub const BLADE_SERVER = 0x0a;
        pub const CONVERTIBLE = 0x0b;
        pub const DETACHABLE = 0x0c;
        pub const IOT_GATEWAY = 0x0d;
        pub const MINI_PC = 0x0e;
        pub const STICK_PC = 0x0f;
    }

    Watch = 0x003;
    {
        pub const GENERIC = 0x00;
        pub const SPORTS = 0x01;
        pub const SMART = 0x02;
    }

    Clock = 0x004;
    {
        pub const GENERIC = 0x00;
    }

    Display = 0x005;
    {
        pub const GENERIC = 0x00;
    }

    RemoteControl = 0x006;
    {
        pub const GENERIC = 0x00;
    }

    EyeGlasses = 0x007;
    {
        pub const GENERIC = 0x00;
    }

    Tag = 0x008;
    {
        pub const GENERIC = 0x00;
    }

    Keyring = 0x009;
    {
        pub const GENERIC = 0x00;
    }
    MediaPlayer = 0x00a;
    {
        pub const GENERIC = 0x00;
    }
    BarcodeScanner = 0x00b;
    {
        pub const GENERIC = 0x00;
    }
    Thermometer = 0x00c;
    {
        pub const GENERIC = 0x00;
        pub const EAR = 0x01;
    }

    HeartRateSensor = 0x00d;
    {
        pub const GENERIC = 0x00;
        pub const BELT = 0x01;
    }

    BloodPressure = 0x00e;
    {
        pub const GENERIC = 0x00;
        pub const ARM = 0x01;
        pub const WRIST = 0x02;
    }
    HumanInterfaceDevice = 0x00f;
    {
        pub const GENERIC = 0x00;
        pub const KEYBOARD = 0x01;
        pub const MOUSE = 0x02;
        pub const JOYSTICK = 0x03;
        pub const GAMEPAD = 0x04;
        pub const DIGITIZER_TABLE = 0x05;
        pub const CARD_READER = 0x06;
        pub const DIGITAL_PEN = 0x07;
        pub const BARCODE_SCANNER = 0x08;
    }
    GlucoseMeter = 0x010;
    {
        pub const GENERIC = 0x00;
    }
    RunningWalkingSensor = 0x011;
    {
        pub const GENERIC = 0x00;
        pub const IN_SHOE = 0x01;
        pub const ON_SHOE = 0x02;
        pub const ON_HIP = 0x03;
    }
    Cycling = 0x012;
    {
        pub const GENERIC = 0x00;
        pub const COMPUTER = 0x01;
        pub const SPEED_SENSOR = 0x02;
        pub const CADENCE_SENSOR = 0x03;
        pub const POWER_SENSOR = 0x04;
        pub const SPEED_CADENCE_SENSOR = 0x05;
    }
    ControlDevice = 0x013;
    {
        pub const GENERIC = 0x00;
    }
    NetworkDevice = 0x014;
    {
        pub const GENERIC = 0x00;
    }

    Sensor = 0x015;
    {
        pub const GENERIC = 0x00;
        pub const MOTION = 0x01;
        pub const AIR_QUALITY = 0x02;
        pub const TEMPERATURE = 0x03;
        pub const HUMIDITY = 0x04;
        pub const LEAK = 0x05;
        pub const SMOKE = 0x06;
        pub const OCCUPANCY = 0x07;
        pub const CONTACT = 0x08;
        pub const CARBON_MONOXIDE = 0x09;
        pub const CARBON_DIOXIDE = 0x0A;
        pub const AMBIENT_LIGHT = 0x0B;
        pub const ENERGY = 0x0C;
        pub const COLOR_LIGHT = 0x0D;
        pub const RAIN = 0x0E;
        pub const FIRE = 0x0F;
        pub const WIND = 0x10;
        pub const PROXIMITY = 0x11;
        pub const MULTI_SENSOR = 0x12;
        pub const FLUSH_MOUNTED = 0x13;
        pub const CEILINGMOUNTED = 0x14;
        pub const WALL_MOUNTED = 0x15;
        pub const MULTISENSOR = 0x16;
        pub const ENERGY_METER = 0x17;
        pub const FLAME_DETECTOR = 0x18;
        pub const VEHICLE_TIRE_PRESSURE = 0x19;

    }
    LightFixture = 0x016;
    {
        pub const GENERIC = 0x00;
    }
    Fan = 0x017;
    {
        pub const GENERIC = 0x00;
    }
    HVAC = 0x018;
    {
        pub const GENERIC = 0x00;
    }
    AirConditioning = 0x019;
    {
        pub const GENERIC = 0x00;
    }
    Humidifier = 0x01a;
    {
        pub const GENERIC = 0x00;
    }
    Heating = 0x01b;
    {
        pub const GENERIC = 0x00;
    }
    AccessControl = 0x1c;
    {
        pub const GENERIC = 0x00;
    }

    MotorizedDevice = 0x01D;
    {
        pub const GENERIC = 0x00;
    }
    PowerDevice = 0x01E;
    {
        pub const GENERIC = 0x00;
    }
    LightSource = 0x01F;
    {
        pub const GENERIC = 0x00;
    }
    WindowCovering = 0x020;
    {
        pub const GENERIC = 0x00;
    }
    AudioSink = 0x021;
    {
        pub const GENERIC = 0x00;
    }
    AudioSource = 0x022;
    {
        pub const GENERIC = 0x00;
    }
    MotorizedVehicle = 0x023;
    {
        pub const GENERIC = 0x00;
    }
    DomesticAppliance = 0x024;
    {
        pub const GENERIC = 0x00;
    }
    WearableAudioDevice = 0x025;
    {
        pub const GENERIC = 0x00;
    }
    Aircraft = 0x026;
    {
        pub const GENERIC = 0x00;
    }
    AVEquipment = 0x027;
    {
        pub const GENERIC = 0x00;
    }
    DisplayEquipment = 0x028;
    {
        pub const GENERIC = 0x00;
    }
    HearingAid = 0x029;
    {
        pub const GENERIC = 0x00;
    }
    Gaming =  0x02A;
    {
        pub const GENERIC = 0x00;
    }
    Signage =  0x02B;
    {
        pub const GENERIC = 0x00;
    }
    PulseOximeter = 0x031;
    {
        pub const GENERIC = 0x00;
    }
    WeightScale = 0x032;
    {
        pub const GENERIC = 0x00;
    }
    PersonalMobilityDevice = 0x033;
    {
        pub const GENERIC = 0x00;
    }
    ContinuousGlucoseMonitor = 0x034;
    {
        pub const GENERIC = 0x00;
    }
    InsulinPump = 0x035;
    {
        pub const GENERIC = 0x00;
    }
    MedicationDelivery = 0x036;
    {
        pub const GENERIC = 0x00;
    }
    OutdoorSportsActivity = 0x051;
    {
        pub const GENERIC = 0x00;
    }
}
