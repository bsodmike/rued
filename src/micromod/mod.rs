use anyhow::Result;
use esp_idf_hal::i2c::{self as HalI2c, Master, I2C0};
use esp_idf_hal::ledc::Peripheral;
use esp_idf_hal::prelude::Peripherals;
use esp_idf_hal::units::FromValueType;

use self::chip::{GpioD1, GpioScl, GpioSda, OnboardLed};

#[cfg(esp32)]
pub mod chip {
    use super::*;
    use esp_idf_hal::gpio::{Gpio14, Gpio21, Gpio22, Gpio27, InputOutput, Output};
    use esp_idf_hal::prelude::Peripherals;
    use shared_bus::{BusManager, NullMutex};

    pub type GpioSda = Gpio21<InputOutput>;
    pub type GpioScl = Gpio22<Output>;
    pub type OnboardLed = Gpio14<Output>;
    pub type GpioD1 = Gpio27<Output>;

    pub fn setup_peripherals(
        peripherals: Peripherals,
    ) -> Result<(GpioSda, GpioScl, I2C0, OnboardLed, GpioD1)> {
        // let peripherals = Peripherals::take().unwrap();
        let i2c0 = peripherals.i2c0;
        let sda = peripherals.pins.gpio21.into_input_output().unwrap();
        let scl = peripherals.pins.gpio22.into_output().unwrap();

        // D0 - GPIO pin 14, pad 10 on the MicroMod
        let gpio_d0: Gpio14<Output> = peripherals.pins.gpio14.into_output().unwrap();
        let mut led_onboard = gpio_d0;
        // D1 - GPIO pin 27, pad 18 on the MicroMod
        let gpio_d1: Gpio27<Output> = peripherals.pins.gpio27.into_output().unwrap();

        Ok((sda, scl, i2c0, led_onboard, gpio_d1))
    }

    pub fn configure(
        peripherals: Peripherals,
    ) -> Result<(
        ActiveGpio,
        BusManager<NullMutex<Master<I2C0, GpioSda, GpioScl>>>,
    )> {
        let chip = Chip::ESP32;
        let sku = BoardSKU::WRL16781;

        let board: MicroModBoard<Chip, BoardSKU> =
            MicroModBoard::new(chip, sku, peripherals).unwrap();

        let (led_onboard, gpio_d1, sda, scl, i2c0) = board.fetch_peripherals()?;

        let i2c_master = MicroModBoard::<Chip, BoardSKU>::configure(i2c0, scl, sda).unwrap();

        unsafe {
            esp_idf_sys::esp_task_wdt_reset();
        } // Reset WDT

        // Instantiate the bus manager, pass the i2c bus.
        let bus = shared_bus::BusManagerSimple::new(i2c_master);

        Ok((
            ActiveGpio {
                led_onboard,
                gpio_d1,
            },
            bus,
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

    fn configure<T, U>(
        i2c0: HalI2c::I2C0,
        scl: T,
        sda: U,
    ) -> Result<Master<I2C0, U, T>, crate::error::BlanketError>
    where
        T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
        U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin;

    fn led_onboard(&self) -> Result<&OnboardLed>;

    fn gpio_sda(&self) -> Result<&GpioSda>;

    fn gpio_scl(&self) -> Result<&GpioScl>;

    fn fetch_peripherals(self) -> Result<(OnboardLed, GpioD1, GpioSda, GpioScl, I2C0)>;
}

pub struct ActiveGpio {
    led_onboard: OnboardLed,
    gpio_d1: GpioD1,
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
}

impl<CHIP, SKU> Board<CHIP, SKU> for MicroModBoard<CHIP, SKU>
where
    CHIP: 'static + std::cmp::PartialEq<super::micromod::Chip>,
{
    type BoardSKU = BoardSKU;
    type Processor = Chip;

    fn configure<T, U>(
        i2c0: HalI2c::I2C0,
        scl: T,
        sda: U,
    ) -> Result<Master<I2C0, U, T>, crate::error::BlanketError>
    where
        T: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::OutputPin + esp_idf_hal::gpio::InputPin,
        U: esp_idf_hal::gpio::Pin + esp_idf_hal::gpio::InputPin + esp_idf_hal::gpio::OutputPin,
    {
        let i2c = Master::new(
            i2c0,
            HalI2c::MasterPins {
                scl, // O
                sda, // I+O
            },
            HalI2c::config::MasterConfig::new().baudrate(400.kHz().into()),
        )?;

        Ok(i2c)
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

    fn fetch_peripherals(self) -> Result<(OnboardLed, GpioD1, GpioSda, GpioScl, I2C0)> {
        Ok((
            self.led_onboard,
            self.gpio_d1,
            self.sda,
            self.scl,
            self.i2c0,
        ))
    }
}

impl<CHIP, SKU> MicroModBoard<CHIP, SKU> {
    pub fn new(chip: CHIP, sku: SKU, peripherals: Peripherals) -> Result<Self> {
        let (sda, scl, i2c0, led_onboard, gpio_d1) =
            super::micromod::chip::setup_peripherals(peripherals)?;

        Ok(Self {
            chip,
            sku,
            sda,
            scl,
            i2c0,
            led_onboard,
            gpio_d1,
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

        //  configure peripherals
        let peripherals = Peripherals::take().unwrap();

        let board: MicroModBoard<Chip, BoardSKU> =
            MicroModBoard::new(chip, sku, peripherals).unwrap();

        let chip = board.chip;
        let value: String = match chip {
            Chip::ESP32 => String::from("ESP32"),
        };
        assert_eq!(value, "ESP32".to_string());

        let _i2c_master =
            MicroModBoard::<Chip, BoardSKU>::configure(board.i2c0, board.scl, board.sda).unwrap();
    }
}
