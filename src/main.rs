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
use st7789::{Orientation, ST7789};
use {defmt_rtt as _, panic_probe as _};

use crate::my_display_interface::SPIDeviceInterface;
use crate::shared_spi::SpiDeviceWithCs;

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

    let style = MonoTextStyle::new(&FONT_10X20, Rgb565::GREEN);
    Text::new(
        "Hello embedded_graphics \n + embassy + RP2040!",
        Point::new(20, 200),
        style,
    )
    .draw(&mut display)
    .unwrap();

    loop {
        Timer::after(Duration::from_secs(2)).await;
    }
    // spawner.spawn(pio_task_sm0(sm0, p.PIN_0.degrade())).unwrap();
    // spawner.spawn(pio_task_sm1(sm1)).unwrap();
    // spawner.spawn(pio_task_sm2(sm2)).unwrap();
}

mod shared_spi {
    use core::cell::RefCell;
    use core::fmt::Debug;

    use embedded_hal_1::digital::OutputPin;
    use embedded_hal_1::spi;
    use embedded_hal_1::spi::SpiDevice;

    #[derive(Copy, Clone, Eq, PartialEq, Debug)]
    pub enum SpiDeviceWithCsError<BUS, CS> {
        #[allow(unused)] // will probably use in the future when adding a flush() to SpiBus
        Spi(BUS),
        Cs(CS),
    }

    impl<BUS, CS> spi::Error for SpiDeviceWithCsError<BUS, CS>
    where
        BUS: spi::Error + Debug,
        CS: Debug,
    {
        fn kind(&self) -> spi::ErrorKind {
            match self {
                Self::Spi(e) => e.kind(),
                Self::Cs(_) => spi::ErrorKind::Other,
            }
        }
    }

    pub struct SpiDeviceWithCs<'a, BUS, CS> {
        bus: &'a RefCell<BUS>,
        cs: CS,
    }

    impl<'a, BUS, CS> SpiDeviceWithCs<'a, BUS, CS> {
        pub fn new(bus: &'a RefCell<BUS>, cs: CS) -> Self {
            Self { bus, cs }
        }
    }

    impl<'a, BUS, CS> spi::ErrorType for SpiDeviceWithCs<'a, BUS, CS>
    where
        BUS: spi::ErrorType,
        CS: OutputPin,
    {
        type Error = SpiDeviceWithCsError<BUS::Error, CS::Error>;
    }

    impl<'a, BUS, CS> SpiDevice for SpiDeviceWithCs<'a, BUS, CS>
    where
        BUS: spi::SpiBusFlush,
        CS: OutputPin,
    {
        type Bus = BUS;

        fn transaction<R>(
            &mut self,
            f: impl FnOnce(&mut Self::Bus) -> Result<R, BUS::Error>,
        ) -> Result<R, Self::Error> {
            let mut bus = self.bus.borrow_mut();
            self.cs.set_low().map_err(SpiDeviceWithCsError::Cs)?;

            let f_res = f(&mut bus);

            // On failure, it's important to still flush and deassert CS.
            let flush_res = bus.flush();
            let cs_res = self.cs.set_high();

            let f_res = f_res.map_err(SpiDeviceWithCsError::Spi)?;
            flush_res.map_err(SpiDeviceWithCsError::Spi)?;
            cs_res.map_err(SpiDeviceWithCsError::Cs)?;

            Ok(f_res)
        }
    }
}

/// Driver for the XPT2046 resistive touchscreen sensor
mod touch {
    use embedded_hal_1::spi::{SpiBus, SpiBusRead, SpiBusWrite, SpiDevice};

    struct Calibration {
        x1: i32,
        x2: i32,
        y1: i32,
        y2: i32,
        sx: i32,
        sy: i32,
    }

    const CALIBRATION: Calibration = Calibration {
        x1: 3880,
        x2: 340,
        y1: 262,
        y2: 3850,
        sx: 320,
        sy: 240,
    };

    pub struct Touch<SPI: SpiDevice> {
        spi: SPI,
    }

    impl<SPI> Touch<SPI>
    where
        SPI: SpiDevice,
        SPI::Bus: SpiBus,
    {
        pub fn new(spi: SPI) -> Self {
            Self { spi }
        }

        pub fn read(&mut self) -> Option<(i32, i32)> {
            let mut x = [0; 2];
            let mut y = [0; 2];
            self.spi
                .transaction(|bus| {
                    bus.write(&[0x90])?;
                    bus.read(&mut x)?;
                    bus.write(&[0xd0])?;
                    bus.read(&mut y)?;
                    Ok(())
                })
                .unwrap();

            let x = (u16::from_be_bytes(x) >> 3) as i32;
            let y = (u16::from_be_bytes(y) >> 3) as i32;

            let cal = &CALIBRATION;

            let x = ((x - cal.x1) * cal.sx / (cal.x2 - cal.x1)).clamp(0, cal.sx);
            let y = ((y - cal.y1) * cal.sy / (cal.y2 - cal.y1)).clamp(0, cal.sy);
            if x == 0 && y == 0 {
                None
            } else {
                Some((x, y))
            }
        }
    }
}

mod my_display_interface {
    use display_interface::{DataFormat, DisplayError, WriteOnlyDataCommand};
    use embedded_hal_1::digital::OutputPin;
    use embedded_hal_1::spi::{SpiBusWrite, SpiDevice};

    /// SPI display interface.
    ///
    /// This combines the SPI peripheral and a data/command pin
    pub struct SPIDeviceInterface<SPI, DC> {
        spi: SPI,
        dc: DC,
    }

    impl<SPI, DC> SPIDeviceInterface<SPI, DC>
    where
        SPI: SpiDevice,
        SPI::Bus: SpiBusWrite,
        DC: OutputPin,
    {
        /// Create new SPI interface for communciation with a display driver
        pub fn new(spi: SPI, dc: DC) -> Self {
            Self { spi, dc }
        }
    }

    impl<SPI, DC> WriteOnlyDataCommand for SPIDeviceInterface<SPI, DC>
    where
        SPI: SpiDevice,
        SPI::Bus: SpiBusWrite,
        DC: OutputPin,
    {
        fn send_commands(&mut self, cmds: DataFormat<'_>) -> Result<(), DisplayError> {
            let r = self.spi.transaction(|bus| {
                // 1 = data, 0 = command
                if let Err(_) = self.dc.set_low() {
                    return Ok(Err(DisplayError::DCError));
                }

                // Send words over SPI
                send_u8(bus, cmds)?;

                Ok(Ok(()))
            });
            r.map_err(|_| DisplayError::BusWriteError)?
        }

        fn send_data(&mut self, buf: DataFormat<'_>) -> Result<(), DisplayError> {
            let r = self.spi.transaction(|bus| {
                // 1 = data, 0 = command
                if let Err(_) = self.dc.set_high() {
                    return Ok(Err(DisplayError::DCError));
                }

                // Send words over SPI
                send_u8(bus, buf)?;

                Ok(Ok(()))
            });
            r.map_err(|_| DisplayError::BusWriteError)?
        }
    }

    fn send_u8<T: SpiBusWrite>(spi: &mut T, words: DataFormat<'_>) -> Result<(), T::Error> {
        match words {
            DataFormat::U8(slice) => spi.write(slice),
            DataFormat::U16(slice) => {
                use byte_slice_cast::*;
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U16LE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_le();
                }
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U16BE(slice) => {
                use byte_slice_cast::*;
                for v in slice.as_mut() {
                    *v = v.to_be();
                }
                spi.write(slice.as_byte_slice())
            }
            DataFormat::U8Iter(iter) => {
                let mut buf = [0; 32];
                let mut i = 0;

                for v in iter.into_iter() {
                    buf[i] = v;
                    i += 1;

                    if i == buf.len() {
                        spi.write(&buf)?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i])?;
                }

                Ok(())
            }
            DataFormat::U16LEIter(iter) => {
                use byte_slice_cast::*;
                let mut buf = [0; 32];
                let mut i = 0;

                for v in iter.map(u16::to_le) {
                    buf[i] = v;
                    i += 1;

                    if i == buf.len() {
                        spi.write(&buf.as_byte_slice())?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i].as_byte_slice())?;
                }

                Ok(())
            }
            DataFormat::U16BEIter(iter) => {
                use byte_slice_cast::*;
                let mut buf = [0; 64];
                let mut i = 0;
                let len = buf.len();

                for v in iter.map(u16::to_be) {
                    buf[i] = v;
                    i += 1;

                    if i == len {
                        spi.write(&buf.as_byte_slice())?;
                        i = 0;
                    }
                }

                if i > 0 {
                    spi.write(&buf[..i].as_byte_slice())?;
                }

                Ok(())
            }
            _ => unimplemented!(),
        }
    }
}
