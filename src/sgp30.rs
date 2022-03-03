use crc_any::CRCu8;
use embassy::time::{Duration, Timer};
use embassy_stm32::i2c::{Error as I2cError, I2c, Instance};
use embedded_hal::blocking::i2c::{Read, Write};
use micromath::F32Ext;

const ADDRESS: u8 = 0x58;

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

/// SGP30 CO2 & TVOC sensor
pub struct Sgp30 {
    crc8: CRCu8,
}

impl Sgp30 {
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

    pub fn init_air_quality<I2C: Instance>(
        &mut self,
        i2c: &mut I2c<I2C>,
    ) -> Result<(), Error> {
        defmt::debug!("SGP30: write(init_air_quality)");
        i2c.write(ADDRESS, &0x2003u16.to_be_bytes())?;
        Ok(())
    }

    pub async fn measure_air_quality<I2C: Instance>(
        &mut self,
        i2c: &mut I2c<'_, I2C>,
    ) -> Result<(u16, u16), Error> {
        let mut buffer = [0; 6];
        i2c.write(ADDRESS, &0x2008u16.to_be_bytes())?;
        Timer::after(Duration::from_millis(12)).await;
        i2c.read(ADDRESS, &mut buffer)?;

        for chunk in buffer.chunks(3) {
            if self.crc8(&chunk[0..2]) != chunk[2] {
                return Err(Error::Crc8);
            }
        }

        let co2eq = u16::from_be_bytes([buffer[0], buffer[1]]);
        let tvoc = u16::from_be_bytes([buffer[3], buffer[4]]);

        Ok((co2eq, tvoc))
    }

    pub fn set_humidity<I2C: Instance>(&mut self, i2c: &mut I2c<I2C>, humidity: u16) -> Result<(), Error> {
        let mut cmd = [0; 5];
        cmd[0..2].copy_from_slice(&0x2061u16.to_be_bytes());
        cmd[2..4].copy_from_slice(&humidity.to_be_bytes());
        cmd[4] = self.crc8(&cmd[2..4]);
        i2c.write(ADDRESS, &cmd)?;

        Ok(())
    }

    pub fn relative_humidity_to_absolute(rh: f32, temp: f32) -> u16 {
        // Arden Buck equations
        let saturated = if temp > 0.0 {
            611.21 * ((18.678 - temp / 234.5) * (temp / (257.14 + temp))).exp()
        } else {
            611.15 * ((23.036 - temp / 333.7) * (temp / (279.82 + temp))).exp()
        };
        let partial_pressure = rh * saturated;
        
        // Ideal gas law
        // pressure = density * universal gas constant * temperature / molar mass
        // density = pressure * molar mass / (universal gas constant * temperature)
        const UNIVERSAL_GAS_CONSTANT: f32 = 8.31446261815324; // [m^3 * Pa / (K * mol)]
        const MOLAR_MASS_CONSTANT: f32 = 0.99999999965e-3; // [kg / mol]
        /// A_r(H) * 2 + A_r(O)
        const ATOMIC_WEIGHT: f32 = 1.008 * 2.0 + 15.9999;
        const MOLAR_MASS: f32 = ATOMIC_WEIGHT * MOLAR_MASS_CONSTANT; // [kg / mol]

        let density = partial_pressure * MOLAR_MASS / (UNIVERSAL_GAS_CONSTANT * (temp + 273.15)); // [kg / m^3]

        (density * 1000.0 * 256.0) as u16
    }
}
