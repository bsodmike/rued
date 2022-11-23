use std::marker::PhantomData;

use esp_idf_hal::adc::*;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::*;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::Hertz;
use shared_bus::{BusManager, NullMutex};

pub struct SystemPeripherals<P, ADC, V, B1, B2, B3, SPI, I2C> {
    pub pulse_counter: PulseCounterPeripherals<P>,
    pub valve: ValvePeripherals,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplaySpiPeripherals<SPI>,
    pub display_i2c: DisplayI2cPeripherals<I2C>,
    // pub rtc_module: ExternalRtcPeripherals<I2C>,
    pub modem: Modem,
}

#[cfg(esp32)]
impl<'a> SystemPeripherals<Gpio33, ADC1, Gpio36, Gpio14, Gpio27, Gpio15, SPI2, I2C0> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        // let (i2c_driver, i2c_bus_manager) = {
        //     let mut config = I2cConfig::new();
        //     config.baudrate(Hertz::from(400 as u32));

        //     let i2c_driver = I2cDriver::new::<_>(
        //         peripherals.i2c0,
        //         peripherals.pins.gpio21,
        //         peripherals.pins.gpio22,
        //         &config,
        //     )
        //     .expect("Expected to initialise I2C");

        //     // Create a shared-bus for the I2C devices that supports threads
        //     let i2c_bus_manager = shared_bus::BusManagerSimple::new(i2c_driver);

        //     (i2c_driver, i2c_bus_manager)
        // };

        // let i2c_bus0 = I2cBus {
        //     i2c: peripherals.i2c0,
        //     sda: peripherals.pins.gpio21.into(),
        //     scl: peripherals.pins.gpio22.into(),
        //     driver: i2c_driver,
        //     bus: i2c_bus_manager,
        // };

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio33,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio25.into(),
                open: peripherals.pins.gpio26.into(),
                close: peripherals.pins.gpio4.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio35.into(),
                voltage: peripherals.pins.gpio36,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio14, // D0
                button2: peripherals.pins.gpio27, // D1
                button3: peripherals.pins.gpio15, // G0
            },
            display: DisplaySpiPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio32.into()),
                    dc: peripherals.pins.gpio18.into(),
                    rst: peripherals.pins.gpio19.into(),
                },
                spi: peripherals.spi2,
                sclk: peripherals.pins.gpio2.into(),
                sdo: peripherals.pins.gpio13.into(),
                cs: Some(peripherals.pins.gpio5.into()),
            },
            modem: peripherals.modem,
            display_i2c: DisplayI2cPeripherals {
                i2c: peripherals.i2c0,
                sda: peripherals.pins.gpio21.into(),
                scl: peripherals.pins.gpio22.into(),
            },
            // rtc_module: ExternalRtcPeripherals {
            //     i2c: peripherals.i2c0,
            //     sda: peripherals.pins.gpio21.into(),
            //     scl: peripherals.pins.gpio22.into(),
            // },
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

// pub struct I2cBus<'a, I2C> {
//     pub i2c: I2C,
//     pub scl: AnyOutputPin,
//     pub sda: AnyOutputPin,
//     pub driver: I2cDriver<'a>,
//     pub bus: BusManager<NullMutex<I2cDriver<'a>>>,
// }
pub struct DisplayI2cPeripherals<I2C> {
    pub i2c: I2C,
    pub scl: AnyIOPin,
    pub sda: AnyIOPin,
}

pub struct ExternalRtcPeripherals<I2C> {
    pub i2c: I2C,
    pub scl: AnyOutputPin,
    pub sda: AnyOutputPin,
}
