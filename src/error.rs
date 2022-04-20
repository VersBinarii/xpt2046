#[cfg(feature = "with_defmt")]
use defmt::{write, Format, Formatter};

#[derive(Debug)]
pub enum BusError<SPIError, CSError> {
    Spi(SPIError),
    Pin(CSError),
}

#[derive(Debug)]
pub enum Error<E> {
    /// SPI bus error
    Bus(E),
}

#[cfg(feature = "with_defmt")]
impl<E> Format for Error<E> {
    fn format(&self, fmt: Formatter) {
        match self {
            Error::Bus(_) => write!(fmt, "Bus error"),
        }
    }
}
