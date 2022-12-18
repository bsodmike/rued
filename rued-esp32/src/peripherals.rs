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

pub struct SystemPeripherals<
    P,
    ADC,
    V,
    B1,
    B2,
    B3,
    SPI_CHAN1,
    SPI_CHAN2,
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
    // pub valve: ValvePeripherals,
    pub battery: BatteryPeripherals<ADC, V>,
    pub buttons: ButtonsPeripherals<B1, B2, B3>,
    pub display: DisplayPeripherals,
    pub spi1: SpiPeripherals<SPI_CHAN1>,
    pub spi2: SpiPeripherals<SPI_CHAN2>,
    pub i2c0: I2c0Peripherals<I2C>,
    pub timer0: LedcTimer<TIMER>,
    pub ledc0: LedcPwmDriver<CHAN0, CHAN0PIN>,
    pub ledc1: LedcPwmDriver<CHAN1, CHAN1PIN>,
    pub ledc2: LedcPwmDriver<CHAN2, CHAN2PIN>,
    pub modem: Modem,
}

#[cfg(esp32)]
impl
    SystemPeripherals<
        Gpio33,
        ADC1,
        Gpio2,
        Gpio14,
        Gpio32,
        Gpio36,
        SPI2,
        SPI3,
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
            sclk: peripherals.pins.gpio18.into(), // SCK
            sdo: peripherals.pins.gpio23.into(),  // COPI
            // NOTE: Display does not need a valid SDI pin; therefore
            // settings this to an unused GPIO.
            sdi: peripherals.pins.gpio10.into(), // dummy (SDI is not needed)
            cs: Some(peripherals.pins.gpio5.into()), // CS
        };

        let spi2 = SpiPeripherals {
            spi: peripherals.spi3,
            sclk: peripherals.pins.gpio17.into(),     // -
            sdo: peripherals.pins.gpio26.into(),      // -
            sdi: peripherals.pins.gpio19.into(),      // CIPO
            cs: Some(peripherals.pins.gpio12.into()), // -
        };

        SystemPeripherals {
            pulse_counter: PulseCounterPeripherals {
                pulse: peripherals.pins.gpio33,
                #[cfg(feature = "ulp")]
                ulp: peripherals.ulp,
            },
            // valve: ValvePeripherals {
            //     power: peripherals.pins.gpio17.into(),
            //     open: peripherals.pins.gpio26.into(),
            //     close: peripherals.pins.gpio12.into(),
            // },
            battery: BatteryPeripherals {
                power: peripherals.pins.gpio35.into(), // A1
                voltage: peripherals.pins.gpio2,
                adc: peripherals.adc1,
            },
            buttons: ButtonsPeripherals {
                button1: peripherals.pins.gpio14, // D0
                button2: peripherals.pins.gpio32, //
                button3: peripherals.pins.gpio36, // G5
            },
            display: DisplayPeripherals {
                control: DisplayControlPeripherals {
                    backlight: Some(peripherals.pins.gpio16.into()), // G4
                    dc: peripherals.pins.gpio27.into(),              // D1
                    rst: peripherals.pins.gpio4.into(),              // I2C_INT
                },
            },
            spi1, // SPI2
            spi2, // SPI3
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
            display: Spi1Peripherals {
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
            display: Spi1Peripherals {
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
