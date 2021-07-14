#![no_main]
#![no_std]

use panic_itm as _;

mod scd30;

#[rtic::app(device = stm32f4xx_hal::stm32, peripherals = true)]
mod app {
    use arrayvec::ArrayString;
    use core::fmt::Write;
    use embedded_graphics::mono_font::iso_8859_1::FONT_10X20;
    use embedded_graphics::mono_font::MonoTextStyleBuilder;
    use embedded_graphics::pixelcolor::Rgb565;
    use embedded_graphics::prelude::*;
    use embedded_graphics::text::Text;
    use st7789::{Orientation, ST7789};
    use stm32f4xx_hal::delay::Delay;
    use stm32f4xx_hal::fsmc_lcd::{ChipSelect3, FsmcLcd, Lcd, LcdPins, SubBank3, Timing};
    use stm32f4xx_hal::gpio::gpiob::{PB10, PB11, PB13};
    use stm32f4xx_hal::gpio::{AlternateOD, Output, PushPull, Speed};
    use stm32f4xx_hal::hal::spi::MODE_0;
    use stm32f4xx_hal::i2c::I2c;
    use stm32f4xx_hal::pac::{I2C2, TIM2};
    use stm32f4xx_hal::prelude::*;
    use stm32f4xx_hal::spi::Spi;
    use stm32f4xx_hal::timer::{CountDownTimer, Event, Timer};

    use crate::scd30::Scd30;

    #[shared]
    struct Shared {
        lcd: ST7789<Lcd<SubBank3>, PB13<Output<PushPull>>>,
        scd30: Scd30<I2c<I2C2, (PB10<AlternateOD<4>>, PB11<AlternateOD<4>>)>>,
        timer: CountDownTimer<TIM2>,
        measurement: Option<(f32, f32, f32)>,
    }

    #[local]
    struct Local {}

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let gpiob = ctx.device.GPIOB.split();
        let gpiod = ctx.device.GPIOD.split();
        let gpioe = ctx.device.GPIOE.split();
        let gpiof = ctx.device.GPIOF.split();
        let gpiog = ctx.device.GPIOG.split();

        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(100.mhz()).freeze();

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
            .set_speed(Speed::VeryHigh);
        let mut _lcd_tearing = gpiob.pb14.into_floating_input();
        // Enable LCD backlight
        gpioe.pe5.into_push_pull_output().set_high();

        let mut delay = Delay::new(ctx.core.SYST, &clocks);
        let lcd_write_timing = Timing::default().data(3).address_setup(3).bus_turnaround(0);
        let lcd_read_timing = Timing::default().data(8).address_setup(8).bus_turnaround(0);

        let (_fsmc, interface) = FsmcLcd::new(
            ctx.device.FSMC,
            lcd_pins,
            &lcd_read_timing,
            &lcd_write_timing,
        );

        let mut lcd = ST7789::new(interface, lcd_reset, 240, 240);
        lcd.init(&mut delay).unwrap();
        lcd.set_orientation(Orientation::PortraitSwapped).unwrap();
        // The framebuffer is 320 pixels tall and in upside down portrait mode the bottom of the buffer is visible instead of the top.
        lcd.set_scroll_offset(320 - 240).unwrap();
        lcd.clear(Rgb565::BLACK).unwrap();

        // I2C2: SCL = PB10, SDA = PB11
        let i2c2 = I2c::new(
            ctx.device.I2C2,
            (
                gpiob.pb10.into_alternate_open_drain(),
                gpiob.pb11.into_alternate_open_drain(),
            ),
            100.khz(),
            clocks,
        );
        let mut scd30 = Scd30::new(i2c2);
        scd30.measure_continuously(0).unwrap();

        let mut timer = Timer::new(ctx.device.TIM2, &clocks).start_count_down(2.hz());
        timer.listen(Event::TimeOut);

        // SPI3: SCK = PB12, MISO = PB4, MOSI = PB5, SSN = PG11
        // RST = PH1, WKUP = PB15, DATARDY = PG12
        let spi = Spi::new(
            ctx.device.SPI3,
            (
                gpiob.pb12.into_alternate(),
                gpiob.pb4.into_alternate(),
                gpiob.pb5.into_alternate(),
            ),
            MODE_0,
            20.mhz().into(),
            clocks,
        );

        (
            Shared {
                lcd,
                scd30,
                timer,
                measurement: None,
            },
            Local {},
            init::Monotonics(),
        )
    }

    #[task(binds = TIM2, shared = [lcd, scd30, timer, measurement])]
    fn measure(mut ctx: measure::Context) {
        ctx.shared
            .timer
            .lock(|timer| timer.clear_interrupt(Event::TimeOut));

        let measurement = ctx.shared.scd30.lock(|scd30| {
            scd30
                .is_data_ready()
                .unwrap()
                .then(|| scd30.read_measurement().unwrap())
        });
        if let Some((co2, temp, rh)) = measurement {
            ctx.shared.measurement.lock(|place| *place = measurement);

            let mut str = ArrayString::<256>::new();
            write!(
                str,
                "CO2:  {:8.02} ppm\nTemp: {:8.02} ºC\nRH:   {:8.02}%",
                co2, temp, rh
            )
            .unwrap();
            let text_style = MonoTextStyleBuilder::new()
                .font(&FONT_10X20)
                .text_color(Rgb565::WHITE)
                .background_color(Rgb565::BLACK)
                .build();
            let text = Text::new(&str, Point::new(0, 100), text_style);

            ctx.shared.lcd.lock(|lcd| text.draw(lcd).unwrap());
        }
    }
}
