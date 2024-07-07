use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
use embassy_stm32::gpio::low_level::{AFType, Pin};
use embassy_stm32::pac::fsmc::vals::{Accmod, Cpsize, Mtyp, Mwid, Waitcfg, Waitpol};
use embassy_stm32::peripherals::{
    FSMC, PD0, PD1, PD10, PD14, PD15, PD4, PD5, PD8, PD9, PE10, PE11, PE12, PE13, PE14, PE15, PE7,
    PE8, PE9, PF0, PG10,
};
use embassy_stm32::rcc::low_level::RccPeripheral;
use embassy_stm32::{Peripheral, PeripheralRef};

pub struct FsmcLcd<'d> {
    _fsmc: PeripheralRef<'d, FSMC>,

    _d0: PeripheralRef<'d, PD14>,
    _d1: PeripheralRef<'d, PD15>,
    _d2: PeripheralRef<'d, PD0>,
    _d3: PeripheralRef<'d, PD1>,
    _d4: PeripheralRef<'d, PE7>,
    _d5: PeripheralRef<'d, PE8>,
    _d6: PeripheralRef<'d, PE9>,
    _d7: PeripheralRef<'d, PE10>,
    _d8: PeripheralRef<'d, PE11>,
    _d9: PeripheralRef<'d, PE12>,
    _d10: PeripheralRef<'d, PE13>,
    _d11: PeripheralRef<'d, PE14>,
    _d12: PeripheralRef<'d, PE15>,
    _d13: PeripheralRef<'d, PD8>,
    _d14: PeripheralRef<'d, PD9>,
    _d15: PeripheralRef<'d, PD10>,

    _a0: PeripheralRef<'d, PF0>,

    _noe: PeripheralRef<'d, PD4>,
    _nwe: PeripheralRef<'d, PD5>,
    _ne3: PeripheralRef<'d, PG10>,
}

impl<'d> FsmcLcd<'d> {
    pub fn new(
        fsmc: impl Peripheral<P = FSMC> + 'd,

        d0: impl Peripheral<P = PD14> + 'd,
        d1: impl Peripheral<P = PD15> + 'd,
        d2: impl Peripheral<P = PD0> + 'd,
        d3: impl Peripheral<P = PD1> + 'd,
        d4: impl Peripheral<P = PE7> + 'd,
        d5: impl Peripheral<P = PE8> + 'd,
        d6: impl Peripheral<P = PE9> + 'd,
        d7: impl Peripheral<P = PE10> + 'd,
        d8: impl Peripheral<P = PE11> + 'd,
        d9: impl Peripheral<P = PE12> + 'd,
        d10: impl Peripheral<P = PE13> + 'd,
        d11: impl Peripheral<P = PE14> + 'd,
        d12: impl Peripheral<P = PE15> + 'd,
        d13: impl Peripheral<P = PD8> + 'd,
        d14: impl Peripheral<P = PD9> + 'd,
        d15: impl Peripheral<P = PD10> + 'd,

        a0: impl Peripheral<P = PF0> + 'd,

        noe: impl Peripheral<P = PD4> + 'd,
        nwe: impl Peripheral<P = PD5> + 'd,

        ne3: impl Peripheral<P = PG10> + 'd,
    ) -> Self {
        embassy_stm32::into_ref!(fsmc);
        embassy_stm32::into_ref!(d0, d1, d2, d3, d4, d5, d6, d7, d8);
        embassy_stm32::into_ref!(d9, d10, d11, d12, d13, d14, d15);
        embassy_stm32::into_ref!(a0);
        embassy_stm32::into_ref!(noe, nwe);
        embassy_stm32::into_ref!(ne3);

        FSMC::enable_and_reset();

        d0.set_as_af(12, AFType::OutputPushPull);
        d1.set_as_af(12, AFType::OutputPushPull);
        d2.set_as_af(12, AFType::OutputPushPull);
        d3.set_as_af(12, AFType::OutputPushPull);
        d4.set_as_af(12, AFType::OutputPushPull);
        d5.set_as_af(12, AFType::OutputPushPull);
        d6.set_as_af(12, AFType::OutputPushPull);
        d7.set_as_af(12, AFType::OutputPushPull);
        d8.set_as_af(12, AFType::OutputPushPull);
        d9.set_as_af(12, AFType::OutputPushPull);
        d10.set_as_af(12, AFType::OutputPushPull);
        d11.set_as_af(12, AFType::OutputPushPull);
        d12.set_as_af(12, AFType::OutputPushPull);
        d13.set_as_af(12, AFType::OutputPushPull);
        d14.set_as_af(12, AFType::OutputPushPull);
        d15.set_as_af(12, AFType::OutputPushPull);

        a0.set_as_af(12, AFType::OutputPushPull);

        noe.set_as_af(12, AFType::OutputPushPull);
        nwe.set_as_af(12, AFType::OutputPushPull);

        ne3.set_as_af(12, AFType::OutputPushPull);

        embassy_stm32::pac::FSMC.bcr(2).modify(|reg| {
            // Copied from https://github.com/stm32-rs/stm32f4xx-hal/blob/81ea4477a52f3d4d152470a550f2594a189c028c/src/fsmc_lcd/mod.rs#L272-L304

            // Disable synchronous writes
            reg.set_cburstrw(false);
            // Don't split burst transactions (doesn't matter for LCD mode)
            reg.set_cpsize(Cpsize::NOBURSTSPLIT);
            // Ignore wait signal (asynchronous mode)
            reg.set_asyncwait(false);
            // Enable extended mode, for different read and write timings
            reg.set_extmod(true);
            // Ignore wait signal (synchronous mode)
            reg.set_waiten(false);
            // Allow write operations
            reg.set_wren(true);
            // Default wait timing
            reg.set_waitcfg(Waitcfg::BEFOREWAITSTATE);
            // Default wait polarity
            reg.set_waitpol(Waitpol::ACTIVELOW);
            // Disable burst reads
            reg.set_bursten(false);
            // Enable NOR flash operations
            reg.set_faccen(true);
            // 16-bit bus width
            reg.set_mwid(Mwid::BITS16);
            // NOR flash mode (compatible with LCD controllers)
            reg.set_mtyp(Mtyp::FLASH);
            // Address and data not multiplexed
            reg.set_muxen(false);
            // Enable this memory bank
            reg.set_mbken(true);
        });

        embassy_stm32::pac::FSMC.btr(2).modify(|reg| {
            reg.set_accmod(Accmod::C);
            reg.set_busturn(0);
            reg.set_datast(0);
            reg.set_addhld(0);
            reg.set_addset(1);
        });

        embassy_stm32::pac::FSMC.bwtr(2).modify(|reg| {
            reg.set_accmod(Accmod::C);
            reg.set_busturn(0);
            reg.set_datast(0);
            reg.set_addhld(0);
            reg.set_addset(1);
        });

        Self {
            _fsmc: fsmc,

            _d0: d0,
            _d1: d1,
            _d2: d2,
            _d3: d3,
            _d4: d4,
            _d5: d5,
            _d6: d6,
            _d7: d7,
            _d8: d8,
            _d9: d9,
            _d10: d10,
            _d11: d11,
            _d12: d12,
            _d13: d13,
            _d14: d14,
            _d15: d15,

            _a0: a0,

            _noe: noe,
            _nwe: nwe,

            _ne3: ne3,
        }
    }
}

unsafe fn write_buffer(ptr: *mut u16, buffer: DataFormat<'_>) -> Result<(), DisplayError> {
    match buffer {
        DataFormat::U8(slice) => write(ptr, slice.iter().copied().map(u16::from)),
        DataFormat::U8Iter(iter) => write(ptr, iter.map(u16::from)),
        DataFormat::U16(slice) => write(ptr, slice.iter().copied()),
        DataFormat::U16BE(slice) | DataFormat::U16LE(slice) => write(ptr, slice.iter().copied()),
        DataFormat::U16BEIter(iter) | DataFormat::U16LEIter(iter) => write(ptr, iter),
        _ => return Err(DisplayError::DataFormatNotImplemented),
    }

    Ok(())
}

unsafe fn write(addr: *mut u16, iter: impl Iterator<Item = u16>) {
    for w in iter {
        addr.write_volatile(w);
    }
}

impl WriteOnlyDataCommand for FsmcLcd<'_> {
    fn send_commands(&mut self, cmd: DataFormat<'_>) -> Result<(), DisplayError> {
        let ptr = 0x6800_0000 as *mut u16;

        unsafe { write_buffer(ptr, cmd) }
    }

    fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
        let ptr = 0x6800_0002 as *mut u16;

        unsafe { write_buffer(ptr, buf) }
    }
}
