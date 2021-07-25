#![no_main]
#![no_std]
#![allow(incomplete_features)]
#![feature(impl_trait_in_bindings)]
#![feature(min_type_alias_impl_trait)]
#![feature(type_alias_impl_trait)]

use arrayvec::ArrayString;
use core::fmt::Write;
use defmt_rtt as _;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Ticker, Timer};
use embassy::traits::gpio::WaitForFallingEdge;
use embassy_stm32::dma::NoDma;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Error as I2cError, I2c};
use embassy_stm32::peripherals::{I2C2, PA0, PE5};
use embassy_stm32::spi::{ByteOrder, Config, Spi, MODE_0};
use embassy_stm32::time::U32Ext;
use embassy_stm32::Peripherals;
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::watchdog::{Watchdog, WatchdogEnable};
use futures::StreamExt;
use panic_probe as _;
use scd30::Scd30;
use serde::Serialize;
use st7789::{Orientation, ST7789};
use stm32f4xx_hal::adc::config::{AdcConfig, SampleTime};
use stm32f4xx_hal::adc::Adc;
use stm32f4xx_hal::fsmc_lcd::{ChipSelect3, FsmcLcd, Lcd, LcdPins, SubBank3, Timing};
use stm32f4xx_hal::gpio::gpiob::PB13;
use stm32f4xx_hal::gpio::gpioc::PC0;
use stm32f4xx_hal::gpio::{Analog, GpioExt, PushPull};
use stm32f4xx_hal::pac::ADC1;
use stm32f4xx_hal::time::MilliSeconds;
use stm32f4xx_hal::watchdog::IndependentWatchdog;

use crate::hp206c::Hp206c;
use crate::ism43362::Ism43362;
use crate::network::Ipv4Addr;

mod hp206c;
mod ism43362;
mod network;
mod scd30;

#[embassy::task]
async fn button_task(mut button: ExtiInput<'static, PA0>, mut lcd_backlight: Output<'static, PE5>) {
    for powered in core::iter::successors(Some(false), |powered| Some(!*powered)) {
        button.wait_for_falling_edge().await;
        if powered {
            defmt::unwrap!(lcd_backlight.set_high());
        } else {
            defmt::unwrap!(lcd_backlight.set_low());
        }
    }
}

#[derive(Copy, Clone, defmt::Format, Serialize)]
struct Readouts {
    scd30: Option<Scd30Readout>,
    hp206c: Option<Hp206cReadout>,
    mix8410: Option<Mix8410Readout>,
}

#[derive(Clone, Copy, defmt::Format, Serialize)]
struct Scd30Readout {
    co2: f32,
    temperature: f32,
    humidity: f32,
}

#[derive(Clone, Copy, defmt::Format, Serialize)]
struct Hp206cReadout {
    pressure: u32,
    temperature: i32,
}

#[derive(Clone, Copy, defmt::Format, Serialize)]
struct Mix8410Readout {
    voltage: u16,
    concentration: f32,
}

fn with_readouts<T>(f: impl FnOnce(&mut Readouts) -> T) -> T {
    static mut READOUTS: Readouts = Readouts {
        scd30: None,
        hp206c: None,
        mix8410: None,
    };

    cortex_m::interrupt::free(|_| {
        // `bare-metal`'s Mutex sucks ass and we have to use a `static mut` anyway.
        // SAFETY:
        //  * we're on a single core CPU with interrupts disabled
        //  * the static is only visible in this function
        unsafe { f(&mut READOUTS) }
    })
}

fn with_network_info<T>(
    f: impl FnOnce(&mut Option<(ArrayString<{ 32 * 3 }>, Ipv4Addr)>) -> T,
) -> T {
    static mut NETWORK_INFO: Option<(ArrayString<{ 32 * 3 }>, Ipv4Addr)> = None;

    cortex_m::interrupt::free(|_| {
        // `bare-metal`'s Mutex sucks ass and we have to use a `static mut` anyway.
        // SAFETY:
        //  * we're on a single core CPU with interrupts disabled
        //  * the static is only visible in this function
        unsafe { f(&mut NETWORK_INFO) }
    })
}

fn retry_crc_errors<T>(
    mut f: impl FnMut() -> Result<T, crate::scd30::Error>,
) -> Result<T, crate::scd30::Error> {
    for _ in 0..3 {
        match f() {
            Err(crate::scd30::Error::Crc8) => continue,
            res => return res,
        }
    }
    Err(crate::scd30::Error::Crc8)
}

#[embassy::task]
async fn measurement_task(
    mut i2c2: I2c<'static, I2C2>,
    mut adc1: Adc<ADC1>,
    oxygen_sensor: PC0<Analog>,
    mut watchdog: IndependentWatchdog,
) {
    // Calibration measurement: 2633 mV
    //  * at a temperature of 18.69 °C
    //  * at a relative humidity of 60.81%
    //  * at a pressure of 100955 Pa
    // saturation vapor pressure of water (by the Arden Buck equation): 2155.213499875295 Pa
    // partial pressure of water: 1310.5853292741667 Pa
    // partial pressure of dry air: 99644.41467072583 Pa
    // partial pressure of O2, assuming a concentration of 20.95% in dry air: 20875.50487351706 Pa
    // concentration of O2 in wet air: 20.678029689977773%
    const O2_VOLTAGE_TO_CONCENTRATION: f32 = 20.678029689977773 / 2633.0;

    // Offset the measurements from the display task
    Timer::after(Duration::from_millis(500)).await;

    let mut scd30 = Scd30::new();

    let mut ticker = Ticker::every(Duration::from_secs(1)).enumerate();
    while let Some((i, ())) = ticker.next().await {
        watchdog.feed();

        let o2_sample = adc1.convert(&oxygen_sensor, SampleTime::Cycles_480);
        let o2_sample_mv = adc1.sample_to_millivolts(o2_sample);
        let o2_concentration = o2_sample_mv as f32 * O2_VOLTAGE_TO_CONCENTRATION;

        let (pressure, temperature) = loop {
            match Hp206c
                .measure_pressure_and_temperature(&mut i2c2, hp206c::Osr::Osr4096)
                .await
            {
                Ok(measurement) => break measurement,
                Err(I2cError::Nack) => {
                    Timer::after(Duration::from_millis(10)).await;
                    continue;
                }
                Err(err) => defmt::panic!("failed to read the HP206C barometer: {}", err),
            }
        };

        with_readouts(|readouts| {
            readouts.hp206c = Some(Hp206cReadout {
                pressure,
                temperature,
            });
            readouts.mix8410 = Some(Mix8410Readout {
                voltage: o2_sample_mv,
                concentration: o2_concentration,
            });
        });

        if i % 1024 == 0 {
            let pressure_in_mbar = ((pressure + 99) / 100) as u16;
            defmt::info!("SCD30: recalibrating to {} mBar", pressure_in_mbar);
            defmt::unwrap!(scd30.measure_continuously(&mut i2c2, pressure_in_mbar));
        }

        if defmt::unwrap!(retry_crc_errors(|| scd30.is_data_ready(&mut i2c2))) {
            let (co2, temperature, humidity) = defmt::unwrap!(scd30.read_measurement(&mut i2c2));

            with_readouts(|readouts| {
                readouts.scd30 = Some(Scd30Readout {
                    co2,
                    temperature,
                    humidity,
                });
            });
        }
    }
}

#[embassy::task]
async fn display_task(mut lcd: ST7789<Lcd<SubBank3>, PB13<stm32f4xx_hal::gpio::Output<PushPull>>>) {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    while let Some(()) = ticker.next().await {
        let measurement = with_readouts(|readouts| *readouts);
        let mut str = ArrayString::<256>::new();

        if let Some(measurement) = measurement.scd30 {
            writeln!(str, "SCD30: CO2: {:8.02} ppm", measurement.co2).unwrap();
            writeln!(str, "SCD30: T:   {:8.02} °C", measurement.temperature).unwrap();
            writeln!(str, "SCD30: RH:  {:8.02}%", measurement.humidity).unwrap();
        } else {
            writeln!(str, "SCD30: CO2: XXXXXXXX ppm").unwrap();
            writeln!(str, "SCD30: T:   XXXXXXXX °C").unwrap();
            writeln!(str, "SCD30: RH:  XXXXXXXX%").unwrap();
        }

        if let Some(measurement) = measurement.hp206c {
            writeln!(str, "HP206C: P:  {:8.02} Pa", measurement.pressure).unwrap();
            writeln!(
                str,
                "HP206C: T:  {:8.02} °C",
                (measurement.temperature as f32) / 100.0
            )
            .unwrap();
        } else {
            writeln!(str, "HP206C: P:  XXXXXXXX Pa").unwrap();
            writeln!(str, "HP206C: T:  XXXXXXXX °C").unwrap();
        }

        if let Some(measurement) = measurement.mix8410 {
            writeln!(str, "MIX8410-O2: {:5} mV", measurement.voltage).unwrap();
            writeln!(str, "MIX8410-O2: {:8.02}%", measurement.concentration).unwrap();
        } else {
            writeln!(str, "MIX8410-O2: XXXXX mV").unwrap();
            writeln!(str, "MIX8410-O2: XXXXXXXX%").unwrap();
        }

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(Rgb565::BLACK)
            .build();
        let text = Text::new(&str, Point::new(0, 20), text_style);

        text.draw(&mut lcd).unwrap();

        if let Some((ssid, ip)) = with_network_info(|info| *info) {
            str.clear();

            writeln!(str, "SSID: {}", ssid).unwrap();
            writeln!(str, "IP: {}", ip).unwrap();

            let text = Text::new(&str, Point::new(0, 219), text_style);
            text.draw(&mut lcd).unwrap();
        } else {
            lcd.fill_solid(
                &Rectangle::new(Point::new(0, 200), Size::new(240, 40)),
                Rgb565::BLACK,
            )
            .unwrap();
        }
    }
}

#[embassy::main]
async fn main(spawner: Spawner, device: Peripherals) {
    defmt::info!("System started");

    // NOTE: it's probably unwise to have two PAC crates...
    let hal = defmt::unwrap!(stm32f4xx_hal::pac::Peripherals::take());

    // Enable system clocks in low power mode
    hal.DBGMCU.cr.modify(|_, w| {
        w.dbg_sleep().set_bit();
        w.dbg_standby().set_bit();
        w.dbg_stop().set_bit()
    });

    // Enable DMA1 and GPIO clocks
    hal.RCC.ahb1enr.modify(|_, w| {
        w.dma1en().enabled();
        w.gpioaen().enabled();
        w.gpioben().enabled();
        w.gpiocen().enabled();
        w.gpioden().enabled();
        w.gpioeen().enabled();
        w.gpiofen().enabled();
        w.gpiogen().enabled();
        w.gpiohen().enabled()
    });

    // IMPORTANT: `GpioExt::split` resets the GPIO port so it must be called before the pins get configured by embassy.
    let gpiob = hal.GPIOB.split();
    let gpioc = hal.GPIOC.split();
    let gpiod = hal.GPIOD.split();
    let gpioe = hal.GPIOE.split();
    let gpiof = hal.GPIOF.split();
    let gpiog = hal.GPIOG.split();

    // Enable the system configuration controller clock
    hal.RCC.apb2enr.modify(|_, w| w.syscfgen().enabled());

    let button = ExtiInput::new(Input::new(device.PA0, Pull::Down), device.EXTI0);
    let lcd_backlight = Output::new(device.PE5, Level::High, Speed::Low);

    let i2c2 = I2c::new(device.I2C2, device.PB10, device.PB11, 100u32.khz());

    let adc1 = Adc::adc1(hal.ADC1, true, AdcConfig::default());
    let oxygen_sensor = gpioc.pc0.into_analog();

    let (lcd_pins, lcd_reset) = {
        let lcd_pins = LcdPins {
            data: (
                gpiod.pd14.into_alternate(),
                gpiod.pd15.into_alternate(),
                gpiod.pd0.into_alternate(),
                gpiod.pd1.into_alternate(),
                gpioe.pe7.into_alternate(),
                gpioe.pe8.into_alternate(),
                gpioe.pe9.into_alternate(),
                gpioe.pe10.into_alternate(),
                gpioe.pe11.into_alternate(),
                gpioe.pe12.into_alternate(),
                gpioe.pe13.into_alternate(),
                gpioe.pe14.into_alternate(),
                gpioe.pe15.into_alternate(),
                gpiod.pd8.into_alternate(),
                gpiod.pd9.into_alternate(),
                gpiod.pd10.into_alternate(),
            ),
            address: gpiof.pf0.into_alternate(),
            read_enable: gpiod.pd4.into_alternate(),
            write_enable: gpiod.pd5.into_alternate(),
            chip_select: ChipSelect3(gpiog.pg10.into_alternate()),
        };
        let lcd_reset = gpiob
            .pb13
            .into_push_pull_output()
            .set_speed(stm32f4xx_hal::gpio::Speed::VeryHigh);
        let mut _lcd_tearing = gpiob.pb14.into_floating_input();

        (lcd_pins, lcd_reset)
    };

    let lcd_write_timing = Timing::default().data(3).address_setup(3).bus_turnaround(0);
    let lcd_read_timing = Timing::default().data(8).address_setup(8).bus_turnaround(0);

    let (_fsmc, interface) = FsmcLcd::new(hal.FSMC, lcd_pins, &lcd_read_timing, &lcd_write_timing);

    let mut lcd = ST7789::new(interface, lcd_reset, 240, 240);
    lcd.init(&mut Delay).unwrap();
    lcd.set_orientation(Orientation::PortraitSwapped).unwrap();
    // The framebuffer is 320 pixels tall and in upside down portrait mode the bottom of the buffer is visible instead of the top.
    lcd.set_scroll_offset(320 - 240).unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();

    // SPI3: SCK = PB12, MISO = PB4, MOSI = PB5, SSN = PG11
    // RST = PH1, WKUP = PB15, DATARDY = PG12
    let mut spi3_config = Config::default();
    spi3_config.mode = MODE_0;
    spi3_config.byte_order = ByteOrder::MsbFirst;
    let wifi = Ism43362::new(
        Spi::new(
            device.SPI3,
            device.PB12,
            device.PB5,
            device.PB4,
            // TODO: DMA. 16-bit transfers are not (yet?) supported.
            NoDma,
            NoDma,
            // FIXME: supposedly supports up to 20 MHz but at over 16 MHz we get data errors
            16.mhz(),
            spi3_config,
        ),
        device.PH1,
        device.PG12,
        device.EXTI12,
        device.PG11,
    );

    // TODO: watchdog
    let mut watchdog = IndependentWatchdog::new(hal.IWDG);
    watchdog.stop_on_debug(&hal.DBGMCU, true);
    watchdog.start(MilliSeconds(2000));

    defmt::unwrap!(spawner.spawn(button_task(button, lcd_backlight)));
    defmt::unwrap!(spawner.spawn(measurement_task(i2c2, adc1, oxygen_sensor, watchdog)));
    defmt::unwrap!(spawner.spawn(display_task(lcd)));
    defmt::unwrap!(spawner.spawn(crate::network::network_task(wifi)));
}
