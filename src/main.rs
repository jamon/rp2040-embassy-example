#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "./example_common.rs"]
mod example_common;

use defmt::*;
use embassy::executor::Spawner;
use embassy::time::{Delay, Duration, Timer};
use embassy_lora::{sx127x::*, LoraTimer};
use embassy_rp::gpio::{Input, NoPin};
use embassy_rp::spi;
use embassy_rp::spi::Spi;

use embassy_rp::{gpio, Peripherals};
use gpio::{Level, Output};

struct DummyRadioSwitch {}
impl RadioSwitch for DummyRadioSwitch {
    fn set_tx(&mut self) {}
    fn set_rx(&mut self) {}
}

struct DummyIRQ {}
impl embedded_hal::digital::ErrorType for DummyIRQ {}
impl embedded_hal_async::digital::Wait for DummyIRQ {
    type WaitForHighFuture<'a>
    where
        Self: 'a;

    fn wait_for_high<'a>(&'a mut self) -> Self::WaitForHighFuture<'a> {
        todo!()
    }

    type WaitForLowFuture<'a>
    where
        Self: 'a;

    fn wait_for_low<'a>(&'a mut self) -> Self::WaitForLowFuture<'a> {
        todo!()
    }

    type WaitForRisingEdgeFuture<'a>
    where
        Self: 'a;

    fn wait_for_rising_edge<'a>(&'a mut self) -> Self::WaitForRisingEdgeFuture<'a> {
        todo!()
    }

    type WaitForFallingEdgeFuture<'a>
    where
        Self: 'a;

    fn wait_for_falling_edge<'a>(&'a mut self) -> Self::WaitForFallingEdgeFuture<'a> {
        todo!()
    }

    type WaitForAnyEdgeFuture<'a>
    where
        Self: 'a;

    fn wait_for_any_edge<'a>(&'a mut self) -> Self::WaitForAnyEdgeFuture<'a> {
        todo!()
    }
}
#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    info!("Hello World!");

    // Example for resistive touch sensor in Waveshare Pico-ResTouch

    let miso = p.PIN_16;
    let mosi = p.PIN_19;
    let clk = p.PIN_18;
    let lora_cs = p.PIN_17;
    let lora_reset = p.PIN_20;
    let lora_ready = p.PIN_21;

    // create SPI
    let mut config = spi::Config::default();
    config.frequency = 2_000_000;
    let mut spi = Spi::new(p.SPI0, clk, mosi, miso, NoPin, config);

    // Configure pins
    let mut lora_cs = Output::new(lora_cs, Level::Low);
    let mut lora_reset = Output::new(lora_reset, Level::Low);
    // let mut lora_ready = Output::new(lora_ready, Level::Low);
    let dummy_radio_switch = DummyRadioSwitch {};
    // let wait = embedded_hal_async::digital::Wait;
    let mut lora_ready = Input::new(lora_ready, gpio::Pull::None);
    let radio = Sx127xRadio::new(spi, lora_cs, lora_reset, lora_ready, dummy_radio_switch);

    loop {
        // cs.set_low();
        // let mut buf = [0x90, 0x00, 0x00, 0xd0, 0x00, 0x00];
        // spi.transfer(&mut buf);
        // cs.set_high();

        // let x = (buf[1] as u32) << 5 | (buf[2] as u32) >> 3;
        // let y = (buf[4] as u32) << 5 | (buf[5] as u32) >> 3;

        // info!("touch: {=u32} {=u32}", x, y);
        Timer::after(Duration::from_secs(1)).await;
    }
}
