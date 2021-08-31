// Implementation based on "opcodes.rs" from the bluetooth-hci project.

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

// TODO - put each chunk into their own namespace? learn how to macro program more?
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
        //pub mod $_cat_name {
            $(
                pub const $var: Appearance = Appearance::new($cat, $sub);
            )+
        //}
        )+
    }
}

appearances! {
    Unknown = 0x000;
    {
        pub const GENERIC_UNKNOWN = 0x0000;
    }

    Phone = 0x001;
    {
        pub const GENERIC_PHONE = 0x0000;
    }

    Computer = 0x002;
    {
        pub const GENERIC_COMPUTER = 0x00;
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
        pub const GENERIC_WATCH = 0x00;
        pub const SPORTS_WATCH = 0x01;
        pub const SMART_WATCH = 0x02;
    }

    HeartRateSensor = 0x00d;
    {
        pub const GENERIC_HRS = 0x00;
        pub const BELT = 0x01;
    }

    Sensor = 0x015;
    {
        pub const GENERIC_SENSOR = 0x00;
        pub const MOTION = 0x01;
        pub const AIRQUALITY = 0x02;
        pub const TEMPERATURE = 0x03;
        pub const HUMIDITY = 0x04;
        pub const LEAK = 0x05;
        pub const SMOKE = 0x06;
        pub const OCCUPANCY = 0x07;
        pub const CONTACT = 0x08;
        pub const CARBONMONOXIDE = 0x09;
        pub const CARBONDIOXIDE = 0x0A;
        pub const AMBIENTLIGHT = 0x0B;
        pub const ENERGY = 0x0C;
        pub const COLORLIGHT = 0x0D;
        pub const RAIN = 0x0E;
        pub const FIRE = 0x0F;
        pub const WIND = 0x10;
        pub const PROXIMITY = 0x11;
        pub const MULTI_DASH_SENSOR = 0x12;
        pub const FLUSHMOUNTED = 0x13;
        pub const CEILINGMOUNTED = 0x14;
        pub const WALLMOUNTED = 0x15;
        pub const MULTISENSOR = 0x16;
        pub const ENERGYMETER = 0x17;
        pub const FLAMEDETECTOR = 0x18;
        pub const VEHICLETIREPRESSURE = 0x19;

    }
}
