#![no_main]
#![no_std]

use arrayvec::ArrayString;
use core::fmt::Write;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::adc::{Adc, Resolution, SampleTime};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{Error as I2cError, ErrorInterruptHandler, EventInterruptHandler, I2c};
use embassy_stm32::peripherals::{ADC1, DMA1_CH2, DMA1_CH7, I2C2, IWDG, PA0, PB13, PC0, PE5};
use embassy_stm32::spi::{BitOrder, Config, Spi, MODE_0};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Ticker, Timer};
use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyleBuilder;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::text::Text;
use fsmc::FsmcLcd;
use futures::StreamExt;
use panic_probe as _;
use scd30::Scd30;
use serde::Serialize;
use sgp30::Sgp30;
use st7789::{Orientation, ST7789};

use crate::hp206c::Hp206c;
use crate::ism43362::Ism43362;
use crate::network::Ipv4Addr;

mod fsmc;
mod hp206c;
mod ism43362;
mod network;
mod scd30;
mod sgp30;

static DISPLAY_ENABLED: AtomicBool = AtomicBool::new(false);

#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static, PA0>, mut lcd_backlight: Output<'static, PE5>) {
    loop {
        button.wait_for_falling_edge().await;
        // `DISPLAY_ENABLED ^ true` is the same as `!DISPLAY_ENABLED`
        // `fetch_xor` returns the old value so invert that as well.
        let powered = !DISPLAY_ENABLED.fetch_xor(true, Ordering::SeqCst);
        if powered {
            lcd_backlight.set_high();
        } else {
            lcd_backlight.set_low();
        }
    }
}

#[derive(Copy, Clone, defmt::Format, Serialize)]
struct Readouts {
    scd30: Option<Scd30Readout>,
    hp206c: Option<Hp206cReadout>,
    mix8410: Option<Mix8410Readout>,
    sgp30: Option<Sgp30Readout>,
}

#[derive(Clone, Copy, defmt::Format, Serialize)]
struct Scd30Readout {
    co2: f32,
    temperature: f32,
    humidity: f32,
}

#[derive(Clone, Copy, defmt::Format, Serialize)]
struct Sgp30Readout {
    co2eq: u16,
    tvoc: u16,
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

static READOUTS: Mutex<CriticalSectionRawMutex, Readouts> = Mutex::new(Readouts {
    scd30: None,
    hp206c: None,
    mix8410: None,
    sgp30: None,
});

static NETWORK_INFO: Mutex<CriticalSectionRawMutex, Option<(ArrayString<{ 32 * 3 }>, Ipv4Addr)>> =
    Mutex::new(None);

#[embassy_executor::task]
async fn measurement_task(
    mut i2c2: I2c<'static, I2C2, DMA1_CH7, DMA1_CH2>,
    mut adc1: Adc<'static, ADC1>,
    mut oxygen_sensor: PC0,
    mut watchdog: IndependentWatchdog<'static, IWDG>,
) {
    // Internal reference voltage (V_REFINT) in millivolts.
    // From "Table 84. Embedded internal reference voltage" in the datasheet for STM32F413ZH (document DS11581).
    const V_REFINT_MV: u32 = 1210;

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
    let mut sgp30 = Sgp30::new();

    let mut vrefint = adc1.enable_vrefint();

    defmt::unwrap!(sgp30.init_air_quality(&mut i2c2).await);

    let mut ticker = Ticker::every(Duration::from_secs(1)).enumerate();
    while let Some((i, ())) = ticker.next().await {
        watchdog.pet();

        let v_refint_sample = adc1.read(&mut vrefint);
        let o2_sample = adc1.read(&mut oxygen_sensor);
        let o2_sample_mv = (o2_sample as u32 * V_REFINT_MV / (v_refint_sample as u32)) as u16;
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

        let (co2eq, tvoc) = defmt::unwrap!(sgp30.measure_air_quality(&mut i2c2).await);

        {
            let mut readouts = READOUTS.lock().await;

            readouts.hp206c = Some(Hp206cReadout {
                pressure,
                temperature,
            });
            readouts.mix8410 = Some(Mix8410Readout {
                voltage: o2_sample_mv,
                concentration: o2_concentration,
            });
            readouts.sgp30 = Some(Sgp30Readout { co2eq, tvoc })
        }

        if i % 1024 == 0 {
            let pressure_in_mbar = ((pressure + 99) / 100) as u16;
            defmt::info!("SCD30: recalibrating to {} mBar", pressure_in_mbar);
            defmt::unwrap!(
                scd30
                    .measure_continuously(&mut i2c2, pressure_in_mbar)
                    .await
            );
        }

        if defmt::unwrap!(scd30.is_data_ready(&mut i2c2).await) {
            let (co2, scd30_temperature, humidity) =
                defmt::unwrap!(scd30.read_measurement(&mut i2c2).await);

            READOUTS.lock().await.scd30 = Some(Scd30Readout {
                co2,
                temperature: scd30_temperature,
                humidity,
            });

            if i % 1024 == 0 {
                let absolute_humidity = Sgp30::relative_humidity_to_absolute(
                    humidity / 100.0,
                    temperature as f32 / 100.0,
                );
                defmt::info!(
                    "SGP30: recalibrating to {} + {}/256 g/m^3",
                    absolute_humidity / 256,
                    absolute_humidity % 256
                );
                defmt::unwrap!(sgp30.set_humidity(&mut i2c2, absolute_humidity).await);
            }
        }
    }
}

#[embassy_executor::task]
async fn display_task(
    mut lcd: ST7789<FsmcLcd<'static>, Output<'static, PB13>, Output<'static, PE5>>,
) {
    let mut ticker = Ticker::every(Duration::from_secs(1));

    loop {
        ticker.next().await;

        if !DISPLAY_ENABLED.load(Ordering::SeqCst) {
            continue;
        }

        let measurement = READOUTS.lock().await.clone();
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

        if let Some(measurement) = measurement.sgp30 {
            writeln!(str, "SGP30: CO2eq: {:5} ppm", measurement.co2eq).unwrap();
            writeln!(str, "SGP30: TVOC: {:6} ppb", measurement.tvoc).unwrap();
        } else {
            writeln!(str, "SGP30: CO2eq: XXXXX ppm").unwrap();
            writeln!(str, "SGP30: TVOC: XXXXXX pbm").unwrap();
        }

        let text_style = MonoTextStyleBuilder::new()
            .font(&FONT_10X20)
            .text_color(Rgb565::WHITE)
            .background_color(Rgb565::BLACK)
            .build();
        let text = Text::new(&str, Point::new(0, 20), text_style);

        text.draw(&mut lcd).unwrap();

        if let Some((ssid, ip)) = NETWORK_INFO.lock().await.clone() {
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

embassy_stm32::bind_interrupts!(struct I2C2Irqs {
    I2C2_EV => EventInterruptHandler<I2C2>;
    I2C2_ER => ErrorInterruptHandler<I2C2>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    defmt::info!("System started");

    let device = embassy_stm32::init(Default::default());

    let button = ExtiInput::new(Input::new(device.PA0, Pull::Down), device.EXTI0);
    let lcd_backlight = Output::new(
        device.PE5,
        if DISPLAY_ENABLED.load(Ordering::SeqCst) {
            Level::High
        } else {
            Level::Low
        },
        Speed::Low,
    );

    let i2c2 = I2c::new(
        device.I2C2,
        device.PB10,
        device.PB11,
        I2C2Irqs,
        device.DMA1_CH7,
        device.DMA1_CH2,
        embassy_stm32::time::khz(100),
        Default::default(),
    );

    let mut adc1 = Adc::new(device.ADC1, &mut Delay);
    adc1.set_sample_time(SampleTime::Cycles480);
    adc1.set_resolution(Resolution::TwelveBit);
    let oxygen_sensor = device.PC0;

    let lcd = FsmcLcd::new(
        device.FSMC,
        device.PD14,
        device.PD15,
        device.PD0,
        device.PD1,
        device.PE7,
        device.PE8,
        device.PE9,
        device.PE10,
        device.PE11,
        device.PE12,
        device.PE13,
        device.PE14,
        device.PE15,
        device.PD8,
        device.PD9,
        device.PD10,
        device.PF0,
        device.PD4,
        device.PD5,
        device.PG10,
    );

    let lcd_reset = Output::new(device.PB13, Level::High, Speed::VeryHigh);
    let _lcd_tearing = Input::new(device.PB14, Pull::None);

    let mut lcd = ST7789::new(lcd, Some(lcd_reset), None, 240, 240);
    lcd.init(&mut Delay).unwrap();
    lcd.set_orientation(Orientation::PortraitSwapped).unwrap();
    // The framebuffer is 320 pixels tall and in upside down portrait mode the bottom of the buffer is visible instead of the top.
    lcd.set_scroll_offset(320 - 240).unwrap();
    lcd.clear(Rgb565::BLACK).unwrap();

    // SPI3: SCK = PB12, MISO = PB4, MOSI = PB5, SSN = PG11
    // RST = PH1, WKUP = PB15, DATARDY = PG12
    let mut spi3_config = Config::default();
    spi3_config.mode = MODE_0;
    spi3_config.bit_order = BitOrder::MsbFirst;
    // FIXME: supposedly supports up to 20 MHz but at over 16 MHz we get data errors
    spi3_config.frequency = embassy_stm32::time::mhz(16);
    let wifi = Ism43362::new(
        Spi::new(
            device.SPI3,
            device.PB12,
            device.PB5,
            device.PB4,
            device.DMA1_CH5,
            device.DMA1_CH0,
            spi3_config,
        ),
        device.PH1,
        device.PG12,
        device.EXTI12,
        device.PG11,
    );

    let mut watchdog = IndependentWatchdog::new(device.IWDG, 32_000_000);
    watchdog.unleash();

    defmt::unwrap!(spawner.spawn(button_task(button, lcd_backlight)));
    defmt::unwrap!(spawner.spawn(measurement_task(i2c2, adc1, oxygen_sensor, watchdog)));
    defmt::unwrap!(spawner.spawn(display_task(lcd)));
    defmt::unwrap!(spawner.spawn(crate::network::network_task(wifi)));
}
