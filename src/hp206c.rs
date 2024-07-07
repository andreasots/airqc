use embassy_stm32::i2c::{Error, I2c, Instance, RxDma, TxDma};
use embassy_time::{Duration, Timer};

const ADDRESS: u8 = 0x76;

// Yes, I know most of the variants are unused...
#[allow(dead_code)]
#[derive(Clone, Copy)]
pub enum Osr {
    Osr128,
    Osr256,
    Osr512,
    Osr1024,
    Osr2048,
    Osr4096,
}

impl Osr {
    const fn command_bits(self) -> u8 {
        match self {
            Self::Osr128 => 0b101,
            Self::Osr256 => 0b100,
            Self::Osr512 => 0b011,
            Self::Osr1024 => 0b010,
            Self::Osr2048 => 0b001,
            Self::Osr4096 => 0b000,
        }
    }

    const fn measurement_delay_for_temperature_and_pressure(self) -> Duration {
        match self {
            Self::Osr128 => Duration::from_micros(4_100),
            Self::Osr256 => Duration::from_micros(8_200),
            Self::Osr512 => Duration::from_micros(16_400),
            Self::Osr1024 => Duration::from_micros(32_800),
            Self::Osr2048 => Duration::from_micros(65_600),
            Self::Osr4096 => Duration::from_micros(131_100),
        }
    }
}

/// HP206C barometer
pub struct Hp206c;

impl Hp206c {
    pub async fn read_pressure_and_temperature<
        I2C: Instance,
        TXDMA: TxDma<I2C>,
        RXDMA: RxDma<I2C>,
    >(
        &self,
        i2c: &mut I2c<'_, I2C, TXDMA, RXDMA>,
    ) -> Result<(u32, i32), Error> {
        let mut buffer = [0; 6];
        i2c.write_read(ADDRESS, &[0x10], &mut buffer).await?;

        let temperature = u32::from_be_bytes([0, buffer[0], buffer[1], buffer[2]]) & 0xfffff;
        // Sign extend the 20 bit temperature
        let temperature = ((temperature << (32 - 20)) as i32) >> (32 - 20);
        let pressure = u32::from_be_bytes([0, buffer[3], buffer[4], buffer[5]]) & 0xfffff;

        Ok((pressure, temperature))
    }

    pub async fn measure_pressure_and_temperature<
        I2C: Instance,
        TXDMA: TxDma<I2C>,
        RXDMA: RxDma<I2C>,
    >(
        &self,
        i2c: &mut I2c<'_, I2C, TXDMA, RXDMA>,
        osr: Osr,
    ) -> Result<(u32, i32), Error> {
        // Measure on pressure and temperature channels (0b00).
        i2c.write(ADDRESS, &[0b010_000_00 | (osr.command_bits() << 2)])
            .await?;

        // Wait for the measurement to complete.
        Timer::after(osr.measurement_delay_for_temperature_and_pressure()).await;

        self.read_pressure_and_temperature(i2c).await
    }
}
