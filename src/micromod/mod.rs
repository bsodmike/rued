use anyhow::Result;
use esp_idf_hal::gpio::{
    AnyIOPin, Input, InputOutput, InputPin, Output, OutputPin, Pin, PinDriver,
};
use esp_idf_hal::i2c::{self as HalI2c, I2cConfig, I2cDriver, I2C0};
use esp_idf_hal::units::{FromValueType, Hertz};
use esp_idf_hal::{peripheral::PeripheralRef, peripherals::Peripherals};
use std::ops::DerefMut;
use std::sync::{Arc, RwLock};

use self::chip::{GpioD1, GpioScl, GpioSda, Modem, OnboardLed};

#[cfg(esp32)]
pub mod chip {
    use std::sync::{Arc, Mutex, RwLock};

    use super::*;
    use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output, PinDriver};
    use esp_idf_hal::{
        modem::Modem as HalModem, peripheral::PeripheralRef, peripherals::Peripherals,
    };

    pub type GpioSda = PeripheralRef<'static, Gpio21>;
    pub type GpioScl = PeripheralRef<'static, Gpio22>;
    pub type OnboardLed = PeripheralRef<'static, Gpio14>;
    pub type GpioD1 = PeripheralRef<'static, Gpio27>;
    pub type Modem = PeripheralRef<'static, HalModem>;

    pub fn setup_peripherals() -> Result<(GpioSda, GpioScl, I2C0, OnboardLed, GpioD1, Modem)> {
        let peripherals = Peripherals::take().unwrap();

        let mut i2c0 = peripherals.i2c0;
        let sda = PeripheralRef::new(peripherals.pins.gpio21);
        let scl = PeripheralRef::new(peripherals.pins.gpio22);

        // D0 - GPIO pin 14, pad 10 on the MicroMod
        let gpio_d0 = PeripheralRef::new(peripherals.pins.gpio14);
        let mut led_onboard = gpio_d0;
        // D1 - GPIO pin 27, pad 18 on the MicroMod
        let gpio_d1 = PeripheralRef::new(peripherals.pins.gpio27);

        let modem: PeripheralRef<HalModem> = PeripheralRef::new(peripherals.modem);

        Ok((sda, scl, i2c0, led_onboard, gpio_d1, modem))
    }

    pub fn configure() -> Result<(ActiveGpio, I2cDriver<'static>)> {
        let chip = Chip::ESP32;
        let sku = BoardSKU::WRL16781;

        let board: MicroModBoard<Chip, BoardSKU> = MicroModBoard::new(chip, sku).unwrap();

        let (led_onboard, gpio_d1, sda, scl, i2c0, modem) = board.fetch_peripherals()?;

        let i2c_driver = MicroModBoard::<Chip, BoardSKU>::configure(i2c0, scl, sda).unwrap();

        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        Ok((
            ActiveGpio {
                led_onboard,
                gpio_d1,
                modem,
            },
            i2c_driver,
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
    ) -> Result<I2cDriver<'static>, crate::error::BlanketError>;

    fn led_onboard(&self) -> Result<&OnboardLed>;

    fn gpio_sda(&self) -> Result<&GpioSda>;

    fn gpio_scl(&self) -> Result<&GpioScl>;

    fn fetch_peripherals(self) -> Result<(OnboardLed, GpioD1, GpioSda, GpioScl, I2C0, Modem)>;
}

pub struct ActiveGpio {
    led_onboard: OnboardLed,
    gpio_d1: GpioD1,
    pub modem: Modem,
}

impl ActiveGpio {
    pub fn led_onboard(&mut self) -> &mut OnboardLed {
        &mut self.led_onboard
    }
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
    ) -> Result<I2cDriver<'static>, crate::error::BlanketError> {
        let mut config = I2cConfig::new();
        config.baudrate(Hertz::from(400 as u32));

        let mut i2c_driver = I2cDriver::new(i2c0, sda, scl, &config)?;

        Ok(i2c_driver)
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

    fn fetch_peripherals(self) -> Result<(OnboardLed, GpioD1, GpioSda, GpioScl, I2C0, Modem)> {
        Ok((
            self.led_onboard,
            self.gpio_d1,
            self.sda,
            self.scl,
            self.i2c0,
            self.modem,
        ))
    }
}

impl<CHIP, SKU> MicroModBoard<CHIP, SKU> {
    pub fn new(chip: CHIP, sku: SKU) -> Result<Self> {
        let (sda, scl, i2c0, led_onboard, gpio_d1, modem) =
            super::micromod::chip::setup_peripherals()?;

        Ok(Self {
            chip,
            sku,
            sda,
            scl,
            i2c0,
            led_onboard,
            gpio_d1,
            modem,
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
