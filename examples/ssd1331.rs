//! Displays live feed of the 8x8 pixels grid of pixels on an ssd1331
//!
//! grideye AMG8833
//! int  scl sda
//! p27  p26 p25
//!
//! ssd1306 in spi mode (r3,r4 on back)
//! https://github.com/jandelgado/arduino/wiki/SSD1306-based-OLED-connected-to-Arduino
//! D0      SCL,CLK,SCK     Clock   p21
//! D1      SDA,MOSI        Data    p17
//! N/A     MISO                    NC
//! RES     RST,RESET       Reset   p16
//! DC      A0    Data/Command      p15
//! CS      Chip Select             NC

#![no_main]
#![no_std]
#![feature(array_chunks)]

use hal::prelude::*;
use hal::stm32 as pac;
use panic_rtt as _;
use stm32f4xx_hal as hal;

use amg88::{raw_to_f32, Address, Amg88};
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::primitives::Rectangle;
use embedded_graphics::style::PrimitiveStyle;
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::spi::{Mode, Phase, Polarity, Spi};
use smart_leds::hsv::{hsv2rgb, Hsv};
use ssd1331::{DisplayRotation, Ssd1331};

const SIZE: i32 = 8;
const GLOBAL_X_OFFSET: i32 = 30;
const GLOBAL_Y_OFFSET: i32 = 0;

macro_rules! dbgprint {
    ($($arg:tt)*) => {
        {
            use core::fmt::Write;
            let mut stdout = jlink_rtt::Output::new();
            writeln!(stdout, $($arg)*).ok();
        }
    };
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Set up the system clock. We want to run at 48MHz for this one.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.use_hse(25.mhz()).sysclk(100.mhz()).freeze();

    let mut delay = Delay::new(cp.SYST, &clocks);

    let gpiob = dp.GPIOB.split();
    let scl = gpiob.pb8.into_alternate_open_drain();
    let sda = gpiob.pb9.into_alternate_open_drain();
    let i2c = I2c::new(dp.I2C1, (scl, sda), 400.khz(), clocks);

    let gpioa = dp.GPIOA.split();

    let sck = gpiob.pb13.into_alternate();
    let miso = gpiob.pb14.into_alternate();
    let mosi = gpiob.pb15.into_alternate();

    let mut rst = gpioa.pa8.into_push_pull_output();
    let dc = gpioa.pa9.into_push_pull_output();

    let spi = Spi::new(
        dp.SPI2,
        (sck, miso, mosi),
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        8.mhz(),
        clocks,
    );

    // Set up the display
    let mut display = Ssd1331::new(spi, dc, DisplayRotation::Rotate0);
    display.reset(&mut rst, &mut delay).unwrap();
    display.init().unwrap();

    let mut amg88 = Amg88::new(i2c, Address::Standard);
    // 50ms startup time
    delay.delay_ms(50_u16);

    // get the device temperature
    dbgprint!(
        "device temperature: {}",
        amg88.get_temperature_celcius().unwrap()
    );
    let mut pixels = [0u8; 128];

    loop {
        // get 128 bytes from sensor ~2ms blocking
        amg88.get_pixels_raw(&mut pixels).unwrap();

        // transform and draw bytes to screen ~3ms blocking
        pixels
            .array_chunks::<2>() // group by 2 u8s
            .map(|r| raw_to_f32(r) as u8) // sign extend and safe truncate
            .enumerate()
            .map(to_x_y_rgb) // map index to x,y and u8 val to rgb
            .for_each(|(x, y, rgb)| {
                Rectangle::new(Point::new(x, y), Point::new(x + SIZE, y + SIZE))
                    .into_styled(PrimitiveStyle::with_fill(Rgb565::new(rgb.r, rgb.g, rgb.b)))
                    .draw(&mut display)
                    .unwrap();
            });

        display.flush().unwrap(); // ~6ms blocking
    }
}

fn to_x_y_rgb((i, val): (usize, u8)) -> (i32, i32, smart_leds::RGB8) {
    let i = i as i32;
    let xindex = i / 8;
    let yindex = i % 8;

    let xoffset = xindex * SIZE + GLOBAL_X_OFFSET;
    let yoffset = yindex * SIZE + GLOBAL_Y_OFFSET;

    // invert it for more pleasant colors
    let val = 255 - val;

    let rgb = hsv2rgb(Hsv {
        hue: 128,
        sat: 128,
        val,
    });

    (xoffset, yoffset, rgb)
}
