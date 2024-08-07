use core::fmt::Write;

use arrayvec::{ArrayString, ArrayVec, CapacityError};
use bstr::ByteSlice;
use defmt::Debug2Format;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::peripherals::{DMA1_CH0, DMA1_CH5};
use embassy_stm32::spi::{Error as SpiError, Instance, Spi};
use embassy_stm32::Peripheral;
use embassy_time::{Duration, Timer};
use itertools::Itertools;

#[derive(defmt::Format)]
pub enum Error {
    Spi(SpiError),
    Buffer(#[defmt(Debug2Format)] CapacityError),
}

impl From<SpiError> for Error {
    fn from(err: SpiError) -> Self {
        Self::Spi(err)
    }
}

impl From<CapacityError> for Error {
    fn from(err: CapacityError) -> Self {
        Self::Buffer(err)
    }
}

pub struct Ism43362<'d, SpiInstance, Reset, DataReady, Ssn>
where
    SpiInstance: Instance,
    Reset: Pin,
    DataReady: Pin,
    Ssn: Pin,
{
    spi: Spi<'d, SpiInstance, DMA1_CH5, DMA1_CH0>,
    reset: Output<'d, Reset>,
    data_ready: ExtiInput<'d, DataReady>,
    ssn: Output<'d, Ssn>,
}

impl<'d, SpiInstance, Reset, DataReady, Ssn> Ism43362<'d, SpiInstance, Reset, DataReady, Ssn>
where
    SpiInstance: Instance,
    Reset: Pin,
    DataReady: Pin,
    Ssn: Pin,
{
    pub fn new(
        spi: Spi<'d, SpiInstance, DMA1_CH5, DMA1_CH0>,
        reset: impl Peripheral<P = Reset> + 'd,
        data_ready: impl Peripheral<P = DataReady> + 'd,
        data_ready_exti_channel: impl Peripheral<P = DataReady::ExtiChannel> + 'd,
        ssn: impl Peripheral<P = Ssn> + 'd,
    ) -> Self {
        Self {
            spi,
            reset: Output::new(reset, Level::High, Speed::Low),
            data_ready: ExtiInput::new(Input::new(data_ready, Pull::Down), data_ready_exti_channel),
            ssn: Output::new(ssn, Level::High, Speed::Low),
        }
    }

    pub async fn reset(&mut self) {
        self.reset.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.reset.set_high();
        Timer::after(Duration::from_millis(500)).await;
    }

    pub async fn send_command(&mut self, cmd: &[u8]) -> Result<ArrayVec<u8, 2048>, Error> {
        self.send_command_with_data(cmd, b"").await
    }

    pub async fn send_s3_command(&mut self, data: &[u8]) -> Result<ArrayVec<u8, 2048>, Error> {
        // `usize::BITS / 3` is an approximation of `(usize::MAX as f32).log10().ceil()`, that is
        // the maximum number of decimal digits `data.len()` could be.
        let mut command = ArrayString::<{ 3 + usize::BITS as usize / 3 }>::new();
        write!(command, "S3={}", data.len()).unwrap();

        self.send_command_with_data(command.as_bytes(), data).await
    }

    pub async fn send_command_with_data(
        &mut self,
        cmd: &[u8],
        data: &[u8],
    ) -> Result<ArrayVec<u8, 2048>, Error> {
        defmt::debug!(
            "sending: {} {}",
            Debug2Format(cmd.as_bstr()),
            Debug2Format(data.as_bstr())
        );

        {
            let _guard = ChipSelectGuard::new(&mut self.ssn);

            let iter = cmd
                .iter()
                .copied()
                .chain(core::iter::once(b'\r'))
                .chain(data.iter().copied())
                .chain(if (cmd.len() + 1 + data.len()) % 2 == 1 {
                    Some(b'\n')
                } else {
                    None
                })
                .tuples();

            for (b0, b1) in iter {
                self.spi.blocking_write(&[u16::from_le_bytes([b0, b1])])?;
            }
        }

        loop {
            let res = self.read().await?;
            if !res.is_empty() {
                return Ok(res);
            }
        }
    }

    pub async fn read(&mut self) -> Result<ArrayVec<u8, 2048>, Error> {
        let mut res = ArrayVec::new();

        defmt::debug!("waiting for data to be ready");
        while self.data_ready.is_low() {
            // The data ready pin can go high between checking that it's low and enabling the
            // interrupt and then all networking hangs. Can't enable the interrupt before checking
            // because that takes a `&mut`.
            let rising = self.data_ready.wait_for_rising_edge();
            pin_utils::pin_mut!(rising);
            futures::future::select(rising, Timer::after(Duration::from_millis(500))).await;
        }
        defmt::debug!("data is ready");

        let _guard = ChipSelectGuard::new(&mut self.ssn);

        while self.data_ready.is_high() {
            let mut data = [u16::from_le_bytes(*b"\n\n")];
            self.spi.blocking_transfer_in_place(&mut data)?;
            defmt::trace!("read {:04x}", data[0]);
            for word in data {
                res.try_extend_from_slice(&word.to_ne_bytes())?;
            }
        }

        // The 0x1515 prepadding is filtered out here so that we don't loop infinitely in the read loop.
        let start = match res.iter().enumerate().find(|(_, b)| **b != 0x15) {
            Some((start, _)) => start,
            None => return Ok(ArrayVec::new()),
        };
        res.drain(..start);

        while let Some(0x15) = res.last() {
            res.pop();
        }

        Ok(res)
    }
}

struct ChipSelectGuard<'a, 'd, P: Pin> {
    pin: &'a mut Output<'d, P>,
}

impl<'a, 'd, P: Pin> ChipSelectGuard<'a, 'd, P> {
    fn new(pin: &'a mut Output<'d, P>) -> Self {
        assert!(pin.is_set_high());
        pin.set_low();

        Self { pin }
    }
}

impl<P: Pin> Drop for ChipSelectGuard<'_, '_, P> {
    fn drop(&mut self) {
        self.pin.set_high();
    }
}
