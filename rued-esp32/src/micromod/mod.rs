use anyhow::Result;
use esp_idf_hal::gpio::{
    AnyIOPin, Input, InputOutput, InputPin, Output, OutputPin, Pin, PinDriver,
};
use esp_idf_hal::i2c::{self as HalI2c, I2cConfig, I2cDriver, I2C0};
use esp_idf_hal::units::{FromValueType, Hertz};
use esp_idf_hal::{peripheral::PeripheralRef, peripherals::Peripherals};
use std::ops::DerefMut;
use std::sync::{Arc, RwLock};

use self::chip::{GpioD1, GpioScl, GpioSda, I2cBus, Modem, OnboardLed};

#[cfg(esp32)]
pub mod chip {
    use std::sync::{Arc, Mutex, RwLock};

    use super::*;
    use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output, PinDriver};
    use esp_idf_hal::{
        modem::Modem as HalModem, peripheral::PeripheralRef, peripherals::Peripherals,
    };
    use shared_bus::{BusManager, NullMutex};

    pub type GpioSda = Gpio21;
    pub type GpioScl = Gpio22;
    pub type OnboardLed = Gpio14;
    pub type GpioD1 = Gpio27;
    pub type Modem = HalModem;
    pub type I2cBus = BusManager<NullMutex<I2cDriver<'static>>>;

    pub fn setup_peripherals() -> Result<AllGpio> {
        let peripherals = Peripherals::take().unwrap();

        let i2c0 = peripherals.i2c0;
        let sda = peripherals.pins.gpio21;
        let scl = peripherals.pins.gpio22;

        // D0 - GPIO pin 14, pad 10 on the MicroMod
        let gpio_d0 = peripherals.pins.gpio14;
        let led_onboard = gpio_d0;
        // D1 - GPIO pin 27, pad 18 on the MicroMod
        let gpio_d1 = peripherals.pins.gpio27;

        let modem = peripherals.modem;
        let active = AllGpio {
            sda,
            scl,
            i2c0,
            led_onboard,
            gpio_d1,
            modem,
        };

        Ok(active)
    }

    pub fn configure() -> Result<(ActiveGpio, I2cBus)> {
        let chip = Chip::ESP32;
        let sku = BoardSKU::WRL16781;

        let board: MicroModBoard<Chip, BoardSKU> = MicroModBoard::new(chip, sku).unwrap();
        let active = board.gpio()?;

        let i2c_bus =
            MicroModBoard::<Chip, BoardSKU>::configure(active.i2c0, active.scl, active.sda)?;

        Ok((
            ActiveGpio {
                led_onboard: active.led_onboard,
                gpio_d1: active.gpio_d1,
                modem: active.modem,
            },
            i2c_bus,
        ))
    }
}

pub enum BoardSKU {
    WRL16781,
}

#[derive(PartialEq)]
pub enum Chip {
    ESP32,
}

pub trait Board<CHIP, SKU> {
    type Processor;
    type BoardSKU;

    fn configure(
        i2c0: I2C0,
        scl: GpioScl,
        sda: GpioSda,
    ) -> Result<I2cBus, crate::errors::BlanketError>;

    fn led_onboard(&self) -> Result<&OnboardLed>;

    fn gpio_sda(&self) -> Result<&GpioSda>;

    fn gpio_scl(&self) -> Result<&GpioScl>;

    fn gpio(self) -> Result<AllGpio>;
}

pub struct ActiveGpio {
    pub led_onboard: OnboardLed,
    pub gpio_d1: GpioD1,
    pub modem: Modem,
}

pub struct AllGpio {
    pub led_onboard: OnboardLed,
    pub gpio_d1: GpioD1,
    pub modem: Modem,
    pub sda: GpioSda,
    pub scl: GpioScl,
    pub i2c0: I2C0,
}

#[allow(dead_code)]
pub struct MicroModBoard<CHIP, SKU> {
    chip: CHIP,
    sku: SKU,
    sda: GpioSda,
    scl: GpioScl,
    i2c0: I2C0,
    led_onboard: OnboardLed,
    gpio_d1: GpioD1,
    modem: Modem,
}

impl<CHIP, SKU> Board<CHIP, SKU> for MicroModBoard<CHIP, SKU>
where
    CHIP: 'static + std::cmp::PartialEq<super::micromod::Chip>,
{
    type BoardSKU = BoardSKU;
    type Processor = Chip;

    fn configure(
        i2c0: I2C0,
        scl: GpioScl,
        sda: GpioSda,
    ) -> Result<I2cBus, crate::errors::BlanketError> {
        let mut config = I2cConfig::new();
        config.baudrate(Hertz::from(400 as u32));

        let i2c_driver = I2cDriver::new(i2c0, sda, scl, &config)?;

        // Create a shared-bus for the I2C devices that supports threads
        let i2c_bus = shared_bus::BusManagerSimple::new(i2c_driver);

        Ok(i2c_bus)
    }

    fn gpio_sda(&self) -> Result<&GpioSda> {
        Ok(&self.sda)
    }

    fn gpio_scl(&self) -> Result<&GpioScl> {
        Ok(&self.scl)
    }

    fn led_onboard(&self) -> Result<&OnboardLed> {
        Ok(&self.led_onboard)
    }

    fn gpio(self) -> Result<AllGpio> {
        Ok((AllGpio {
            led_onboard: self.led_onboard,
            gpio_d1: self.gpio_d1,
            sda: self.sda,
            scl: self.scl,
            i2c0: self.i2c0,
            modem: self.modem,
        }))
    }
}

impl<CHIP, SKU> MicroModBoard<CHIP, SKU> {
    pub fn new(chip: CHIP, sku: SKU) -> Result<Self> {
        let active = super::micromod::chip::setup_peripherals()?;

        Ok(Self {
            chip,
            sku,
            sda: active.sda,
            scl: active.scl,
            i2c0: active.i2c0,
            led_onboard: active.led_onboard,
            gpio_d1: active.gpio_d1,
            modem: active.modem,
        })
    }
}

#[cfg(test)]
mod test {
    #![allow(dead_code)]

    use super::*;

    fn instantiate_chip() {
        let chip = Chip::ESP32;
        let sku = BoardSKU::WRL16781;

        let board: MicroModBoard<Chip, BoardSKU> = MicroModBoard::new(chip, sku).unwrap();

        let chip = board.chip;
        let value: String = match chip {
            Chip::ESP32 => String::from("ESP32"),
        };
        assert_eq!(value, "ESP32".to_string());

        let _i2c_master =
            MicroModBoard::<Chip, BoardSKU>::configure(board.i2c0, board.scl, board.sda).unwrap();
    }
}
