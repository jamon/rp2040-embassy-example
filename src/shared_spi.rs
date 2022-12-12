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

    fn transaction<R>(&mut self, f: impl FnOnce(&mut Self::Bus) -> Result<R, BUS::Error>) -> Result<R, Self::Error> {
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
