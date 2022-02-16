#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embedded_hal::digital::v2::OutputPin;
use max7219::*;
use nb::block;
use panic_halt as _;
use stm32f1xx_hal::{
    pac,
    adc,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
    timer::Timer,
};


#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let mut rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc
        .cfgr
        .use_hse(8.mhz()) // переключились на кварц
        .sysclk(16.mhz()) 
        .adcclk(2.mhz()) // для ADC
        .freeze(&mut flash.acr);

    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);
    let mut ch0 = gpiob.pb0.into_analog(&mut gpiob.crl);
    let mut cha0 = gpioa.pa0.into_analog(&mut gpioa.crl);
    let mut cha2 = gpioa.pa2.into_analog(&mut gpioa.crl);

    let pins = (
        gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl), // sck
        gpioa.pa6.into_floating_input(&mut gpioa.crl),      // miso
        gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl), // mosi
    );
    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi1(
        dp.SPI1,
        pins,
        &mut afio.mapr,
        spi_mode,
        10.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer 
    let mut timer = Timer::syst(cp.SYST, &clocks).start_count_down(1000.hz());

    let cs = gpiob.pb10.into_push_pull_output(&mut gpiob.crh);
    let mut display = MAX7219::from_spi_cs(4, spi, cs).unwrap();
    display.power_on().unwrap();
    for i in 0..4 {
        display.set_intensity(i, 0x1).unwrap();
    }

    // отсчёт идёт на экране вниз с 0 до 3 и слева с 0 до 7
    // let mut frame: [[u8; 8]; 4] = [[0; 8]; 4];
    let bars: [u8; 9] = [
        0b00000000, 0b00000001, 0b00000011, 0b00000111, 0b00001111, 0b00011111, 0b00111111,
        0b01111111, 0b11111111,
    ];

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        let mut data: u16 = adc1.read(&mut ch0).unwrap(); //3.3d = 4095, 3.3/2 = 2036
        data = data - 248; // 0,2В считаем за 0
        data = data + data/2;
        if data > 4095 {
            data = 4095;
        }
        let val: u8 = (data / 16) as u8;

        data = adc1.read(&mut cha0).unwrap(); //3.3d = 4095, 3.3/2 = 2036
        let val1: u8 = (data / 16) as u8;
        data = adc1.read(&mut cha2).unwrap(); //3.3d = 4095, 3.3/2 = 2036
        let val2: u8 = (data / 16) as u8;

        let mut frame: [[u8; 8]; 4] = [[0; 8]; 4]; // обнуляем
        // индикатор поля
        if val > 64 + 7 {
            //71
            frame[3][0] = 0xFF;
            if val > 64 * 2 + 7 {
                //135
                frame[2][0] = 0xFF;
                if val > 64 * 3 + 7 {
                    // 199
                    frame[1][0] = 0xFF;
                    frame[0][0] = bars[((val / 8) - 24) as usize];
                } else {
                    frame[1][0] = bars[((val / 8) - 16) as usize];
                }
            } else {
                frame[2][0] = bars[((val / 8) - 8) as usize];
            }
        } else {
            frame[3][0] = bars[(val / 8) as usize];
        }
        // индикатор A0
        if val1 > 64 + 7 {
            //71
            frame[3][3] = 0xFF;
            if val1 > 64 * 2 + 7 {
                //135
                frame[2][3] = 0xFF;
                if val1 > 64 * 3 + 7 {
                    // 199
                    frame[1][3] = 0xFF;
                    frame[0][3] = bars[((val1 / 8) - 24) as usize];
                } else {
                    frame[1][3] = bars[((val1 / 8) - 16) as usize];
                }
            } else {
                frame[2][3] = bars[((val1 / 8) - 8) as usize];
            }
        } else {
            frame[3][3] = bars[(val1 / 8) as usize];
        }
        // индикатор A2
        if val2 > 64 + 7 {
            //71
            frame[3][6] = 0xFF;
            if val2 > 64 * 2 + 7 {
                //135
                frame[2][6] = 0xFF;
                if val2 > 64 * 3 + 7 {
                    // 199
                    frame[1][6] = 0xFF;
                    frame[0][6] = bars[((val2 / 8) - 24) as usize];
                } else {
                    frame[1][6] = bars[((val2 / 8) - 16) as usize];
                }
            } else {
                frame[2][6] = bars[((val2 / 8) - 8) as usize];
            }
        } else {
            frame[3][6] = bars[(val2 / 8) as usize];
        }

        for i in 0..4 {
            frame[i][1] = frame[i][0];
            frame[i][4] = frame[i][3];
            frame[i][7] = frame[i][6];
            display
                .write_raw(
                    i,
                    &frame[i],
                )
                .unwrap();
        }
        block!(timer.wait()).unwrap();
        let _ = led.set_high();
        block!(timer.wait()).unwrap();
        let _ = led.set_low();
    }
}
