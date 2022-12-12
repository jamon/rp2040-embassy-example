#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
use core::cell::RefCell;

use defmt::info;
use embassy_executor::Spawner;
use embassy_rp::gpio::{AnyPin, Level, Output, Pin};
use embassy_rp::pio::{Pio0, PioPeripherial, PioStateMachine, PioStateMachineInstance, Sm0};
use embassy_rp::relocate::RelocatedProgram;
use embassy_rp::spi::{Blocking, Spi};
use embassy_rp::{pio_instr_util, spi};
use embassy_time::{Delay, Duration, Timer};
use embedded_graphics::mono_font::ascii::FONT_10X20;
use embedded_graphics::mono_font::MonoTextStyle;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_graphics::text::Text;
use my_display_interface::SPIDeviceInterface;
use shared_spi::SpiDeviceWithCs;
use st7789::{Orientation, ST7789};
use {defmt_rtt as _, panic_probe as _};

mod my_display_interface;
mod shared_spi;
mod touch;

#[embassy_executor::task]
async fn pio_task_blink(mut sm: PioStateMachineInstance<Pio0, Sm0>, pin: AnyPin) {
    // Setup sm2

    // blink
    let prg = pio_proc::pio_asm!(
        ".origin 0",
        "set pindirs,1",
        ".wrap_target",
        "set pins,0 [31]",
        "set pins,0 [31]",
        "set pins,0 [31]",
        "set pins,0 [31]",
        "set pins,1 [31]",
        "set pins,1 [31]",
        "set pins,1 [31]",
        "set pins,1 [31]",
        ".wrap",
    );
    let relocated = RelocatedProgram::new(&prg.program);
    let out_pin = sm.make_pio_pin(pin);
    let pio_pins = [&out_pin];
    sm.set_set_pins(&pio_pins);
    sm.set_set_range(25, 1);

    sm.write_instr(relocated.origin() as usize, relocated.code());
    pio_instr_util::exec_jmp(&mut sm, relocated.origin());
    sm.set_clkdiv(0);
    // sm.set_clkdiv((125e6 / 20.0 / 2e2 * 256.0) as u32);

    let pio::Wrap { source, target } = relocated.wrap();
    sm.set_wrap(source, target);

    //     sm.set_clkdiv((125e6 / 20.0 / 2e2 * 256.0) as u32);
    sm.set_enable(true);
    info!("started");

    // loop {

    //     sm.wait_irq(3).await;
    //     info!("IRQ trigged");
    // }
}

// #[embassy_executor::task]
// async fn pio_task_sm0(mut sm: PioStateMachineInstance<Pio0, Sm0>, pin: AnyPin) {
//     // Setup sm0

//     // Send data serially to pin
//     let prg = pio_proc::pio_asm!(
//         ".origin 16",
//         "set pindirs, 1",
//         ".wrap_target",
//         "out pins,1 [19]",
//         ".wrap",
//     );

//     let relocated = RelocatedProgram::new(&prg.program);
//     let out_pin = sm.make_pio_pin(pin);
//     let pio_pins = [&out_pin];
//     sm.set_out_pins(&pio_pins);
//     sm.write_instr(relocated.origin() as usize, relocated.code());
//     pio_instr_util::exec_jmp(&mut sm, relocated.origin());
//     sm.set_clkdiv((125e6 / 20.0 / 2e2 * 256.0) as u32);
//     sm.set_set_range(0, 1);
//     let pio::Wrap { source, target } = relocated.wrap();
//     sm.set_wrap(source, target);

//     sm.set_autopull(true);
//     sm.set_out_shift_dir(ShiftDirection::Left);

//     sm.set_enable(true);

//     let mut v = 0x0f0caffa;
//     loop {
//         sm.wait_push(v).await;
//         v ^= 0xffff;
//         info!("Pushed {:032b} to FIFO", v);
//     }
// }

// #[embassy_executor::task]
// async fn pio_task_sm1(mut sm: PioStateMachineInstance<Pio0, Sm1>) {
//     // Setupm sm1

//     // Read 0b10101 repeatedly until ISR is full
//     let prg = pio_proc::pio_asm!(".origin 8", "set x, 0x15", ".wrap_target", "in x, 5 [31]", ".wrap",);

//     let relocated = RelocatedProgram::new(&prg.program);
//     sm.write_instr(relocated.origin() as usize, relocated.code());
//     pio_instr_util::exec_jmp(&mut sm, relocated.origin());
//     sm.set_clkdiv((125e6 / 2e3 * 256.0) as u32);
//     sm.set_set_range(0, 0);
//     let pio::Wrap { source, target } = relocated.wrap();
//     sm.set_wrap(source, target);

//     sm.set_autopush(true);
//     sm.set_in_shift_dir(ShiftDirection::Right);
//     sm.set_enable(true);
//     loop {
//         let rx = sm.wait_pull().await;
//         info!("Pulled {:032b} from FIFO", rx);
//     }
// }

// #[embassy_executor::task]
// async fn pio_task_sm2(mut sm: PioStateMachineInstance<Pio0, Sm2>) {
//     // Setup sm2

//     // Repeatedly trigger IRQ 3
//     let prg = pio_proc::pio_asm!(
//         ".origin 0",
//         ".wrap_target",
//         "set x,10",
//         "delay:",
//         "jmp x-- delay [15]",
//         "irq 3 [15]",
//         ".wrap",
//     );
//     let relocated = RelocatedProgram::new(&prg.program);
//     sm.write_instr(relocated.origin() as usize, relocated.code());

//     let pio::Wrap { source, target } = relocated.wrap();
//     sm.set_wrap(source, target);

//     pio_instr_util::exec_jmp(&mut sm, relocated.origin());
//     sm.set_clkdiv((125e6 / 2e3 * 256.0) as u32);
//     sm.set_enable(true);
//     loop {
//         sm.wait_irq(3).await;
//         info!("IRQ trigged");
//     }
// }

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    let pio = p.PIO0;

    let (_, sm0, ..) = pio.split();

    spawner.spawn(pio_task_blink(sm0, p.PIN_25.degrade())).unwrap();

    let rst = p.PIN_15;
    let display_cs = p.PIN_9;
    let dcx = p.PIN_8;
    let miso = p.PIN_12;
    let mosi = p.PIN_11;
    let clk = p.PIN_10;

    // create SPI
    let mut config = spi::Config::default();
    config.phase = spi::Phase::CaptureOnSecondTransition;
    config.polarity = spi::Polarity::IdleHigh;
    config.frequency = 50_000_000;
    let spi: Spi<'_, _, Blocking> = Spi::new_blocking(p.SPI1, clk, mosi, miso, config);
    let spi_bus = RefCell::new(spi);

    let display_spi = SpiDeviceWithCs::new(&spi_bus, Output::new(display_cs, Level::High));
    let dcx = Output::new(dcx, Level::Low);
    let rst = Output::new(rst, Level::Low);

    let di = SPIDeviceInterface::new(display_spi, dcx);
    let mut display = ST7789::new(di, rst, 240, 240);

    // initialize
    display.init(&mut Delay).unwrap();

    // set default orientation
    display.set_orientation(Orientation::Landscape).unwrap();

    display.clear(Rgb565::BLACK).unwrap();

    let style_green = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    let style_red = MonoTextStyle::new(&FONT_10X20, Rgb565::RED);
    let my_str = "MERRY CHRISTMAS";
    let mut x = 40;
    let mut alt_color = false;

    // Text::new("MERRY CHRISTMAS\n", Point::new(0, 20), style_green)
    //     .draw(&mut display)
    //     .unwrap();

    loop {
        // Text::new("MERRY CHRISTMAS\n", Point::new(0, 20), style_green)
        //     .draw(&mut display)
        //     .unwrap();
        // Timer::after(Duration::from_secs(1)).await;
        // Text::new("MERRY CHRISTMAS\n", Point::new(0, 20), style_red)
        //     .draw(&mut display)
        //     .unwrap();
        for (i, c) in my_str.chars().enumerate() {
            let mut tmp = [0u8; 4];
            let letter = c.encode_utf8(&mut tmp);
            Text::new(
                letter,
                Point::new(x, 20),
                if alt_color { style_red } else { style_green },
            )
            .draw(&mut display)
            .unwrap();
            alt_color = !alt_color;
            x += 10;
        }
        // alt_color = !alt_color;
        x = 40;

        // Timer::after(Duration::from_secs(1)).await;
    }
    // spawner.spawn(pio_task_sm0(sm0, p.PIN_0.degrade())).unwrap();
    // spawner.spawn(pio_task_sm1(sm1)).unwrap();
    // spawner.spawn(pio_task_sm2(sm2)).unwrap();
}
