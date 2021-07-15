use arrayvec::ArrayVec;
use crc_any::CRCu8;
use stm32f4xx_hal::hal::blocking::i2c::{Read, Write};

const ADDRESS: u8 = 0x61;

#[derive(Debug)]
pub enum Error<E> {
    I2c(E),
    Crc8,
}

impl<E> From<E> for Error<E> {
    fn from(err: E) -> Self {
        Self::I2c(err)
    }
}

pub struct Scd30<I2C> {
    i2c: I2C,
    crc8: CRCu8,
}

impl<E, I2C: Read<Error = E> + Write<Error = E>> Scd30<I2C> {
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            crc8: CRCu8::create_crc(0x31, 8, 0xFF, 0x00, false),
        }
    }

    fn crc8(&mut self, data: &[u8]) -> u8 {
        self.crc8.digest(data);
        let crc = self.crc8.get_crc();
        self.crc8.reset();
        crc
    }
    pub fn measure_continuously(&mut self, pressure: u16) -> Result<(), Error<E>> {
        let mut command = ArrayVec::<u8, 5>::new();
        command.extend(0x0010u16.to_be_bytes());
        command.extend(pressure.to_be_bytes());
        command.push(self.crc8(&pressure.to_be_bytes()));
        self.i2c.write(ADDRESS, &command)?;
        Ok(())
    }

    pub fn is_data_ready(&mut self) -> Result<bool, Error<E>> {
        let mut buffer = [0; 3];
        self.i2c.write(ADDRESS, &0x0202u16.to_be_bytes())?;
        self.i2c.read(ADDRESS, &mut buffer)?;

        if self.crc8(&buffer[..2]) != buffer[2] {
            return Err(Error::Crc8);
        }

        Ok(u16::from_be_bytes([buffer[0], buffer[1]]) == 1)
    }

    pub fn read_measurement(&mut self) -> Result<(f32, f32, f32), Error<E>> {
        let mut buffer = [0; 18];
        self.i2c.write(ADDRESS, &0x0300u16.to_be_bytes())?;
        self.i2c.read(ADDRESS, &mut buffer)?;

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
