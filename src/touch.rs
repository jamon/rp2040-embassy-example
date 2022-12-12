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
