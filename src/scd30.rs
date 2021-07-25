use arrayvec::ArrayVec;
use crc_any::CRCu8;
use embassy_stm32::i2c::{Error as I2cError, I2c, Instance};
use embedded_hal::blocking::i2c::{Read, Write};

const ADDRESS: u8 = 0x61;

#[derive(defmt::Format)]
pub enum Error {
    I2c(I2cError),
    Crc8,
}

impl From<I2cError> for Error {
    fn from(err: I2cError) -> Self {
        Self::I2c(err)
    }
}

/// SCD30 CO2 sensor
pub struct Scd30 {
    crc8: CRCu8,
}

impl Scd30 {
    pub fn new() -> Self {
        Self {
            crc8: CRCu8::create_crc(0x31, 8, 0xFF, 0x00, false),
        }
    }

    fn crc8(&mut self, data: &[u8]) -> u8 {
        self.crc8.digest(data);
        let crc = self.crc8.get_crc();
        self.crc8.reset();
        crc
    }

    pub fn measure_continuously<I2C: Instance>(
        &mut self,
        i2c: &mut I2c<I2C>,
        pressure: u16,
    ) -> Result<(), Error> {
        let mut command = ArrayVec::<u8, 5>::new();
        command.extend(0x0010u16.to_be_bytes());
        command.extend(pressure.to_be_bytes());
        command.push(self.crc8(&pressure.to_be_bytes()));
        i2c.write(ADDRESS, &command)?;
        Ok(())
    }

    pub fn is_data_ready<I2C: Instance>(&mut self, i2c: &mut I2c<I2C>) -> Result<bool, Error> {
        let mut buffer = [0; 3];
        i2c.write(ADDRESS, &0x0202u16.to_be_bytes())?;
        i2c.read(ADDRESS, &mut buffer)?;

        if self.crc8(&buffer[..2]) != buffer[2] {
            return Err(Error::Crc8);
        }

        Ok(u16::from_be_bytes([buffer[0], buffer[1]]) == 1)
    }

    pub fn read_measurement<I2C: Instance>(
        &mut self,
        i2c: &mut I2c<I2C>,
    ) -> Result<(f32, f32, f32), Error> {
        let mut buffer = [0; 18];
        i2c.write(ADDRESS, &0x0300u16.to_be_bytes())?;
        i2c.read(ADDRESS, &mut buffer)?;

        for chunk in buffer.chunks(3) {
            if self.crc8(&chunk[0..2]) != chunk[2] {
                return Err(Error::Crc8);
            }
        }

        let co2 = f32::from_be_bytes([buffer[0], buffer[1], buffer[3], buffer[4]]);
        let temperature = f32::from_be_bytes([buffer[6], buffer[7], buffer[9], buffer[10]]);
        let humidity = f32::from_be_bytes([buffer[12], buffer[13], buffer[15], buffer[16]]);

        Ok((co2, temperature, humidity))
    }
}
