use std::marker::PhantomData;

use esp_idf_hal::adc::*;
use esp_idf_hal::gpio::*;
use esp_idf_hal::i2c::*;
use esp_idf_hal::ledc::CHANNEL0;
use esp_idf_hal::ledc::CHANNEL1;
use esp_idf_hal::ledc::CHANNEL2;
use esp_idf_hal::ledc::TIMER0;
use esp_idf_hal::modem::Modem;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::Hertz;
use shared_bus::{BusManager, NullMutex};

pub const SPI_BUS_FREQ: u32 = 26;

#[cfg(esp32)]
#[cfg(any(
    feature = "micromod-qwiic-carrier-single",
    feature = "micromod-main-board-single"
))]
pub struct SystemPeripherals<
    P,
    ADC,
    V,
    B1,
    B2,
    B3,
    I2C,
    TIMER,
    CHAN0,
    CHAN1,
    CHAN2,
    CHAN0PIN,
    CHAN1PIN,
    CHAN2PIN,
> {
    pub pulse_counter: PulseCounterPeripherals<P>,
    pub valve: ValvePeripherals,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplayPeripherals,
    pub spi1: SpiBusPeripherals,
    pub sd_card: SdCardPeripherals,
    pub i2c0: I2c0Peripherals<I2C>,
    pub timer0: LedcTimer<TIMER>,
    pub ledc0: LedcPwmDriver<CHAN0, CHAN0PIN>,
    pub ledc1: LedcPwmDriver<CHAN1, CHAN1PIN>,
    pub ledc2: LedcPwmDriver<CHAN2, CHAN2PIN>,
    pub modem: Modem,
}

#[cfg(esp32)]
#[cfg(any(
    feature = "micromod-qwiic-carrier-single",
    feature = "micromod-main-board-single"
))]
impl
    SystemPeripherals<
        Gpio33,
        ADC1,
        Gpio2,
        Gpio32,
        Gpio0,
        Gpio36,
        I2C0,
        TIMER0,
        CHANNEL0,
        CHANNEL1,
        CHANNEL2,
        Gpio13,
        Gpio15,
        Gpio25,
    >
{
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        let spi1 = SpiPeripherals {
            spi: peripherals.spi2,
            sclk: peripherals.pins.gpio18.into(),    // SCK
            sdo: peripherals.pins.gpio23.into(),     // COPI
            sdi: peripherals.pins.gpio19.into(),     // CIPO
            cs: Some(peripherals.pins.gpio5.into()), // CS
        };

        let driver = std::sync::Arc::new(
            SpiDriver::new(
                spi1.spi,
                spi1.sclk,      // SCK
                spi1.sdo,       // MOSI
                Some(spi1.sdi), // MISO / NOTE: Default value
                Dma::Disabled,
            )
            .unwrap(),
        );

        let spi1 = SpiBusPeripherals {
            driver,
            cs: spi1.cs,
        };

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio33,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio17.into(),
                open: peripherals.pins.gpio26.into(),
                close: peripherals.pins.gpio12.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio35.into(), // A1
                voltage: peripherals.pins.gpio2,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio32, // G5
                button2: peripherals.pins.gpio0,  //
                button3: peripherals.pins.gpio36, //
            },
            display: DisplayPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio16.into()), // G4
                    dc: peripherals.pins.gpio14.into(),              // D0
                    rst: peripherals.pins.gpio4.into(),              // I2C_INT
                },
            },
            spi1, // SPI2
            sd_card: SdCardPeripherals {
                cs: peripherals.pins.gpio27.into(), // D1
            },
            modem: peripherals.modem,
            i2c0: I2c0Peripherals {
                i2c: peripherals.i2c0,
                sda: peripherals.pins.gpio21.into(),
                scl: peripherals.pins.gpio22.into(),
            },
            timer0: LedcTimer(peripherals.ledc.timer0),
            ledc0: LedcPwmDriver {
                chan: peripherals.ledc.channel0,
                pin: peripherals.pins.gpio13, // PWM0-Processor
            },
            ledc1: LedcPwmDriver {
                chan: peripherals.ledc.channel1,
                pin: peripherals.pins.gpio15, // G0-Processor
            },
            ledc2: LedcPwmDriver {
                chan: peripherals.ledc.channel2,
                pin: peripherals.pins.gpio25, // G1-Processor
            },
        }
    }
}

#[cfg(esp32)]
#[cfg(feature = "micromod-data-logging-carrier")]
pub struct SystemPeripherals<P, ADC, V, B1, B2, B3, I2C> {
    pub pulse_counter: PulseCounterPeripherals<P>,
    pub valve: ValvePeripherals,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplayPeripherals,
    pub spi1: SpiBusPeripherals,
    pub sd_card: SdCardPeripherals,
    pub i2c0: I2c0Peripherals<I2C>,
    pub modem: Modem,
}

#[cfg(esp32)]
#[cfg(feature = "micromod-data-logging-carrier")]
impl SystemPeripherals<Gpio33, ADC1, Gpio35, Gpio27, Gpio13, Gpio12, I2C0> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        let spi1 = SpiPeripherals {
            spi: peripherals.spi2,
            sclk: peripherals.pins.gpio18.into(),     // SCK
            sdo: peripherals.pins.gpio23.into(),      // COPI
            sdi: peripherals.pins.gpio19.into(),      // CIPO
            cs: Some(peripherals.pins.gpio15.into()), // HEADER_CS / G0-Processor
        };

        let driver = std::sync::Arc::new(
            SpiDriver::new(
                spi1.spi,
                spi1.sclk,      // SCK
                spi1.sdo,       // MOSI
                Some(spi1.sdi), // MISO / NOTE: Default value
                Dma::Disabled,
            )
            .unwrap(),
        );

        let spi1 = SpiBusPeripherals {
            driver,
            cs: spi1.cs,
        };

        // gpio25, // G1-Processor

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio33,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            valve: ValvePeripherals {
                power: peripherals.pins.gpio17.into(),
                open: peripherals.pins.gpio26.into(),
                close: peripherals.pins.gpio10.into(),
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio34.into(), // A0
                voltage: peripherals.pins.gpio35,      // A1
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio27, // D1
                button2: peripherals.pins.gpio13, // PWM0-Processor
                button3: peripherals.pins.gpio12, // PWM1-Processor
            },
            display: DisplayPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio16.into()), // G4
                    dc: peripherals.pins.gpio14.into(),              // D0
                    rst: peripherals.pins.gpio4.into(),              // I2C_INT
                },
            },
            spi1, // SPI2
            sd_card: SdCardPeripherals {
                cs: peripherals.pins.gpio5.into(), // SPI_CS
            },
            modem: peripherals.modem,
            i2c0: I2c0Peripherals {
                i2c: peripherals.i2c0,
                sda: peripherals.pins.gpio21.into(),
                scl: peripherals.pins.gpio22.into(),
            },
        }
    }
}

#[cfg(esp32c3)]
pub struct SystemPeripherals<P, ADC, V, B1, B2, B3, I2C> {
    pub pulse_counter: PulseCounterPeripherals<P>,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplayPeripherals,
    pub spi1: SpiBusPeripherals,
    pub sd_card: SdCardPeripherals,
    pub i2c0: I2c0Peripherals<I2C>,
    pub modem: Modem,
}

#[cfg(esp32c3)]
impl SystemPeripherals<Gpio0, ADC1, Gpio4, Gpio1, Gpio2, Gpio3, I2C0> {
    pub fn take() -> Self {
        let peripherals = Peripherals::take().unwrap();

        let spi1 = SpiPeripherals {
            spi: peripherals.spi2,
            sclk: peripherals.pins.gpio15.into(),     // SCK
            sdo: peripherals.pins.gpio16.into(),      // COPI
            sdi: peripherals.pins.gpio17.into(),      // CIPO
            cs: Some(peripherals.pins.gpio18.into()), // HEADER_CS / G0-Processor
        };

        let driver = std::sync::Arc::new(
            SpiDriver::new(
                spi1.spi,
                spi1.sclk,      // SCK
                spi1.sdo,       // MOSI
                Some(spi1.sdi), // MISO / NOTE: Default value
                Dma::Disabled,
            )
            .unwrap(),
        );

        let spi1 = SpiBusPeripherals {
            driver,
            cs: spi1.cs,
        };

        // gpio25, // G1-Processor

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio0,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio10.into(), // A1
                voltage: peripherals.pins.gpio4,       // A0
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio1, // D1
                button2: peripherals.pins.gpio2, // PWM0-Processor
                button3: peripherals.pins.gpio3, // PWM1-Processor
            },
            display: DisplayPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio19.into()), // G4
                    dc: peripherals.pins.gpio20.into(),              // D0
                    rst: peripherals.pins.gpio21.into(),             // I2C_INT
                },
            },
            spi1, // SPI2
            sd_card: SdCardPeripherals {
                cs: peripherals.pins.gpio7.into(), // SPI_CS
            },
            modem: peripherals.modem,
            i2c0: I2c0Peripherals {
                i2c: peripherals.i2c0,
                sda: peripherals.pins.gpio8.into(),
                scl: peripherals.pins.gpio9.into(),
            },
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

pub struct DisplayPeripherals {
    pub control: DisplayControlPeripherals,
}

pub struct SpiPeripherals<SPI> {
    pub spi: SPI,
    pub sclk: AnyOutputPin,
    pub sdo: AnyOutputPin,
    pub sdi: AnyIOPin,
    pub cs: Option<AnyOutputPin>,
}

pub struct SpiBusPeripherals {
    pub driver: std::sync::Arc<SpiDriver<'static>>,
    pub cs: Option<AnyOutputPin>,
}

pub struct SdCardPeripherals {
    pub cs: AnyOutputPin,
}

pub struct I2c0Peripherals<I2C> {
    pub i2c: I2C,
    pub scl: AnyIOPin,
    pub sda: AnyIOPin,
}

pub struct LedcTimer<TIMER>(pub TIMER);
pub struct LedcPwmDriver<CHAN, PIN> {
    pub chan: CHAN,
    pub pin: PIN,
}
