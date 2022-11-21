use esp_idf_hal::adc::*;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::*;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::*;

pub struct SystemPeripherals<P, ADC, V, B1, B2, B3, SPI, I2C> {
    pub pulse_counter: PulseCounterPeripherals<P>,
    pub valve: ValvePeripherals,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplaySpiPeripherals<SPI>,
    pub rtc_module: ExternalRtcPeripherals<I2C>,
    pub modem: Modem,
}

#[cfg(esp32)]
impl SystemPeripherals<Gpio33, ADC1, Gpio36, Gpio2, Gpio4, Gpio32, SPI2, I2C0> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio33,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio25.into(),
                open: peripherals.pins.gpio26.into(),
                close: peripherals.pins.gpio27.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio35.into(),
                voltage: peripherals.pins.gpio36,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio2,
                button2: peripherals.pins.gpio4,
                button3: peripherals.pins.gpio32,
            },
            display: DisplaySpiPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio15.into()),
                    dc: peripherals.pins.gpio18.into(),
                    rst: peripherals.pins.gpio19.into(),
                },
                spi: peripherals.spi2,
                sclk: peripherals.pins.gpio14.into(),
                sdo: peripherals.pins.gpio13.into(),
                cs: Some(peripherals.pins.gpio5.into()),
            },
            modem: peripherals.modem,
            rtc_module: ExternalRtcPeripherals {
                i2c: peripherals.i2c0,
                sda: peripherals.pins.gpio21.into(),
                scl: peripherals.pins.gpio22.into(),
            },
        }
    }
}

#[cfg(any(esp32s2, esp32s3))]
impl SystemPeripherals<Gpio1, ADC1, Gpio9, Gpio2, Gpio4, Gpio12, SPI2> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio1,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio3.into(),
                open: peripherals.pins.gpio6.into(),
                close: peripherals.pins.gpio7.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio8.into(),
                voltage: peripherals.pins.gpio9,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio2,
                button2: peripherals.pins.gpio4,
                button3: peripherals.pins.gpio12,
            },
            display: DisplaySpiPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio15.into()),
                    dc: peripherals.pins.gpio18.into(),
                    rst: peripherals.pins.gpio19.into(),
                },
                spi: peripherals.spi2,
                sclk: peripherals.pins.gpio14.into(),
                sdo: peripherals.pins.gpio13.into(),
                cs: Some(peripherals.pins.gpio5.into()),
            },
            modem: peripherals.modem,
        }
    }
}

#[cfg(not(any(esp32, esp32s2, esp32s3)))]
impl SystemPeripherals<Gpio1, ADC1, Gpio0, Gpio2, Gpio3, Gpio4, SPI2> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio1,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio6.into(),
                open: peripherals.pins.gpio7.into(),
                close: peripherals.pins.gpio8.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio5.into(),
                voltage: peripherals.pins.gpio0,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio2,
                button2: peripherals.pins.gpio3,
                button3: peripherals.pins.gpio4,
            },
            display: DisplaySpiPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio9.into()),
                    dc: peripherals.pins.gpio10.into(),
                    rst: peripherals.pins.gpio18.into(),
                },
                spi: peripherals.spi2,
                sclk: peripherals.pins.gpio15.into(),
                sdo: peripherals.pins.gpio16.into(),
                cs: Some(peripherals.pins.gpio14.into()),
            },
            modem: peripherals.modem,
        }
    }
}

pub struct PulseCounterPeripherals<P> {
    pub pulse: P,
    #[cfg(feature = "ulp")]
    pub ulp: esp_idf_hal::ulp::ULP,
}

pub struct ValvePeripherals {
    pub power: AnyIOPin,
    pub open: AnyIOPin,
    pub close: AnyIOPin,
}

pub struct BatteryPeripherals<ADC, V> {
    pub power: AnyInputPin,
    pub voltage: V,
    pub adc: ADC,
}

pub struct ButtonsPeripherals<B1, B2, B3> {
    pub button1: B1,
    pub button2: B2,
    pub button3: B3,
}

pub struct DisplayControlPeripherals {
    pub backlight: Option<AnyOutputPin>,
    pub dc: AnyOutputPin,
    pub rst: AnyOutputPin,
}

pub struct DisplaySpiPeripherals<SPI> {
    pub control: DisplayControlPeripherals,
    pub spi: SPI,
    pub sclk: AnyOutputPin,
    pub sdo: AnyOutputPin,
    pub cs: Option<AnyOutputPin>,
}

pub struct ExternalRtcPeripherals<I2C> {
    pub i2c: I2C,
    pub scl: AnyOutputPin,
    pub sda: AnyOutputPin,
}
