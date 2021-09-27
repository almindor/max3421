use embedded_hal::{blocking::spi::{Operation, Transactional, Transfer, Write}, digital::v2::OutputPin};

use super::Error;

/// SPI interface
pub trait SpiInterface {    
    /// Execute the provided transactions
    fn exec_spi(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Error>;
}

impl<SPI> SpiInterface for SPI
where
    SPI: Transactional<u8>,
{
    fn exec_spi(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Error> {
        self.exec(operations).map_err(|_| Error::ExecSPI)
    }
}

pub struct SpiWithManualCS<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> SpiWithManualCS<SPI, CS> {
    pub fn new(spi: SPI, cs: CS) -> Self {
        Self {
            spi,
            cs,
        }
    }
}

impl<SPI, CS> SpiInterface for SpiWithManualCS<SPI, CS>
where
    SPI: Write<u8> + Transfer<u8>,
    CS: OutputPin,
{
    fn exec_spi(&mut self, operations: &mut [Operation<u8>]) -> Result<(), Error> {
        self.cs.set_low().map_err(|_| Error::PinCS)?;

        for op in operations {
            match op {
                Operation::Write(data) => self.spi.write(data).map_err(|_| Error::WriteSPI)?,
                Operation::Transfer(buf) => {
                    self.spi.transfer(buf).map_err(|_| Error::TransferSPI)?;
                },
            }
        }

        self.cs.set_high().map_err(|_| Error::PinCS)
    }
}

impl<SPI, CS> From<(SPI, CS)> for SpiWithManualCS<SPI, CS> {
    fn from(tuple: (SPI, CS)) -> Self {
        SpiWithManualCS {
            spi: tuple.0,
            cs: tuple.1,
        }
    }
}
