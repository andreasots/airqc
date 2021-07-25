use core::convert::Infallible;
use core::fmt::Write;

use arrayvec::{ArrayString, ArrayVec};
use bstr::ByteSlice;
use defmt::Debug2Format;
use embassy::time::{Duration, Timer};
use embassy::traits::gpio::WaitForRisingEdge;
use embassy::util::Unborrow;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pin, Pull, Speed};
use embassy_stm32::spi::{Error, Instance, Spi};
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::{InputPin, OutputPin, StatefulOutputPin};
use itertools::Itertools;

trait UnwrapInfallible<T> {
    fn unwrap_infallible(self) -> T;
}

impl<T> UnwrapInfallible<T> for Result<T, Infallible> {
    fn unwrap_infallible(self) -> T {
        match self {
            Ok(val) => val,
            Err(infallible) => match infallible {},
        }
    }
}

pub struct Ism43362<'d, SpiInstance, Reset, DataReady, Ssn>
where
    SpiInstance: Instance,
    Reset: Pin,
    DataReady: Pin,
    Ssn: Pin,
{
    spi: Spi<'d, SpiInstance, NoDma, NoDma>,
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
        spi: Spi<'d, SpiInstance, NoDma, NoDma>,
        reset: impl Unborrow<Target = Reset> + 'd,
        data_ready: impl Unborrow<Target = DataReady> + 'd,
        data_ready_exti_channel: impl Unborrow<Target = DataReady::ExtiChannel> + 'd,
        ssn: impl Unborrow<Target = Ssn> + 'd,
    ) -> Self {
        Self {
            spi,
            reset: Output::new(reset, Level::High, Speed::Low),
            data_ready: ExtiInput::new(Input::new(data_ready, Pull::Down), data_ready_exti_channel),
            ssn: Output::new(ssn, Level::High, Speed::Low),
        }
    }

    pub async fn reset(&mut self) {
        self.reset.set_low().unwrap_infallible();
        Timer::after(Duration::from_millis(10)).await;
        self.reset.set_high().unwrap_infallible();
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
                let word = u16::from_le_bytes([b0, b1]);
                // NOTE: using `transfer` instead of `write` because `write` hangs.
                //  I assume it's because whatever the device sends back isn't read from the
                //  data register and so the transfer doesn't finish.
                self.spi.transfer(&mut [word])?;
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

        while self.data_ready.is_low().unwrap_infallible() {
            // The data ready pin can go high between checking that it's low and enabling the
            // interrupt and then all networking hangs. Can't enable the interrupt before checking
            // because that takes a `&mut`.
            futures::future::select(
                self.data_ready.wait_for_rising_edge(),
                Timer::after(Duration::from_millis(500)),
            )
            .await;
        }

        let _guard = ChipSelectGuard::new(&mut self.ssn);

        while self.data_ready.is_high().unwrap_infallible() {
            let mut data = [u16::from_le_bytes(*b"\n\n")];
            for word in self.spi.transfer(&mut data)? {
                for byte in word.to_ne_bytes() {
                    if !res.is_empty() || byte != 0x15 {
                        res.push(byte);
                    }
                }
            }
        }

        while let Some(0x15) = res.last() {
            res.pop();
        }

        Ok(res)
    }
}

struct ChipSelectGuard<'a, P: OutputPin<Error = Infallible>> {
    pin: &'a mut P,
}

impl<'a, P: OutputPin<Error = Infallible> + StatefulOutputPin<Error = Infallible>>
    ChipSelectGuard<'a, P>
{
    fn new(pin: &'a mut P) -> Self {
        assert!(pin.is_set_high().unwrap_infallible());
        pin.set_low().unwrap_infallible();

        Self { pin }
    }
}

impl<P: OutputPin<Error = Infallible>> Drop for ChipSelectGuard<'_, P> {
    fn drop(&mut self) {
        self.pin.set_high().unwrap_infallible();
    }
}
