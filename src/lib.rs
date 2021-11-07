#![no_std]
#![feature(unchecked_math)]

use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

/// Celcius scale factor for pixel temperature
pub const CELCIUS_PIXEL_SCALE: f32 = 0.25;
/// Celcius scale factor for device temperature
pub const CELCIUS_THERMISTOR_SCALE: f32 = 0.0625;

#[non_exhaustive]
pub struct Amg88<I2C> {
    twim: I2C,
    address: Address,
}

impl<I2C, E> Amg88<I2C>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
{
    /// Creates a new Amg88 driver in Normal mode 10 fps
    pub fn new(twim: I2C, address: Address) -> Self {
        Amg88 { twim, address }
    }

    /// Reset
    pub fn reset(&mut self) -> Result<(), Amg88Error> {
        self.set_register(Register::Reset, 0x3F)
            .map_err(|_| Amg88Error::ConnectionError)
    }

    /// Get raw pixel value for all 64 pixels in one transaction which saves 63 i2c round trips
    pub fn get_pixels_raw(&mut self, buffer: &mut [u8; 128]) -> Result<(), Amg88Error> {
        let cmd = [Register::TemperatureStart as u8];
        self.twim
            .write_read(self.address as u8, &cmd, buffer)
            .map_err(|_| Amg88Error::ConnectionError)?;

        Ok(())
    }

    /// Get raw pixel value for a pixel 0-63 left to right. This instead of
    /// `get_pixel_celcius` can allow you to skip the cost of the f32
    /// multiplication from the scale factor
    pub fn get_pixel_raw(&mut self, pixel: u8) -> Result<[u8; 2], Amg88Error> {
        let mut buffer = [0u8, 0];
        let cmd = [Register::TemperatureStart as u8 + (2 * pixel)];
        self.twim
            .write_read(self.address as u8, &cmd, &mut buffer)
            .map_err(|_| Amg88Error::ConnectionError)?;
        Ok(buffer)
    }

    /// Get pixel value in celcius for pixel 0-63 as raw value
    pub fn get_pixel_celcius(&mut self, pixel: u8) -> Result<f32, Amg88Error> {
        let raw = self.get_pixel_raw(pixel)?;
        let c = raw_to_f32(&raw) * CELCIUS_THERMISTOR_SCALE;
        Ok(c)
    }

    /// Get raw device temperature. This instead of `get_temperature_celcius`
    /// can allow you to skip the cost of the f32 multiplication from the scale
    /// factor
    pub fn get_temperature_raw(&mut self) -> Result<[u8; 2], Amg88Error> {
        let mut buffer = [0u8, 0];
        let cmd = [Register::ThermistorLsb as u8];
        self.twim
            .write_read(self.address as u8, &cmd, &mut buffer)
            .map_err(|_| Amg88Error::ConnectionError)?;
        Ok(buffer)
    }

    /// Get device temperature in celcius
    pub fn get_temperature_celcius(&mut self) -> Result<f32, Amg88Error> {
        let raw = self.get_temperature_raw()?;
        let c = raw_to_f32(&raw) * CELCIUS_THERMISTOR_SCALE;
        Ok(c)
    }

    /// Set the device framerate update
    pub fn set_framerate(&mut self, framerate: Framerate) -> Result<(), Amg88Error> {
        self.set_register(Register::Framerate, framerate as u8)
            .map_err(|_| Amg88Error::ConnectionError)
    }

    /// Set the power mode of the device
    pub fn power(&mut self, power: Power) -> Result<(), Amg88Error> {
        self.set_register(Register::PowerControl, power as u8)
            .map_err(|_| Amg88Error::ConnectionError)
    }

    /// Set the device framerate update
    fn set_register(&mut self, register: Register, value: u8) -> Result<(), Amg88Error> {
        let cmd_bytes = [register as u8, value];
        self.twim
            .write(self.address as u8, &cmd_bytes)
            .map_err(|_| Amg88Error::ConnectionError)
    }
}

/// Convert u8 pairs from raw getters into an f32. f32 is the only safe
/// saturating downcast so its the right choice to return, though you may want
/// to immediately downcast to your preferred primitive
pub fn raw_to_f32(chunk: &[u8; 2]) -> f32 {
    unsafe {
        i16::from_le_bytes(*chunk)
            // sign extend, trust us the top 4 are 0s so unchecked
            .unchecked_shl(4)
            .unchecked_shr(4)
            // use saturating add because 12bit could be i16MIN which would panic
            .saturating_add(1) as f32
    }
}

/// Possible runtime errors
#[derive(Debug)]
pub enum Amg88Error {
    ConnectionError,
}

/// I2C address
#[derive(Copy, Clone)]
pub enum Address {
    Standard = 0x69,
    Alternate = 0x68,
}

#[derive(Debug, Copy, Clone)]
enum Register {
    PowerControl = 0x00,
    Reset = 0x01,
    Framerate = 0x02,
    ThermistorLsb = 0x0E,
    TemperatureStart = 0x80,
}

/// Choice of power modes for the device
#[derive(Debug, Copy, Clone)]
pub enum Power {
    Normal = 0x00,
    Sleep = 0x10,
    Standby60Seconds = 0x20,
    Standby10Seconds = 0x21,
}

/// Choice of refresh rates of the sensor
#[derive(Debug, Copy, Clone)]
pub enum Framerate {
    Fps10 = 0x00,
    Fps1 = 0x01,
}
