use core::cmp::min;
use embedded_hal::blocking::delay::DelayUs;
use embedded_hal::blocking::spi::Operation;
use embedded_hal::digital::v2::OutputPin;

use crate::spi_interface::SpiInterface;

use super::*;

pub struct Max3421<SPI, RES> {
    spi: SPI,
    res: RES,
    peraddr: u8,
    sndtog: bool,
    rcvtog: bool,
    max_packet_size_0: u8,
}

impl<SPI, RES> Max3421<SPI, RES>
where
    SPI: SpiInterface,
    RES: OutputPin,
{
    pub fn new(spi: SPI, res: RES) -> Self {
        Self {
            spi,
            res,
            peraddr: 0xff,
            sndtog: false,
            rcvtog: false,
            max_packet_size_0: 8,
        }
    }

    pub fn init(&mut self, delay: &mut dyn DelayUs<u8>) -> Result<(), Error> {
        self.reset(delay)?;
        self.config_host()
    }

    pub fn get_revision(&mut self) -> Result<u8, Error> {
        self.reg_read(Reg::Revision)
    }

    pub fn wait_for_connection(&mut self) -> Result<Speed, Error> {
        self.reg_clear(Reg::Mode, 0b00000010)?; // LOWSPEED
        self.reg_write(Reg::Hirq, Hirq::Condet as u8)?;

        let speed = loop {
            //self.wait_hirq(Hirq::Condet);
            self.reg_write(Reg::Hctl, 0b00000100)?; // SAMPLEBUS
            let hrsl = self.reg_read(Reg::Hrsl)?;

            match (hrsl >> 6) & 0b11 {
                0b10 => break Speed::High,
                0b01 => break Speed::Low,
                _ => continue, // delay.delay_ms(pause),
            }
        };

        if speed == Speed::Low {
            self.reg_set(Reg::Mode, 0b00000010)?; // LOWSPEED
        }

        self.reg_set(Reg::Hctl, 0b00000001)?; // BUSRST
        self.wait_hirq(Hirq::Busevent)?;
        self.reg_set(Reg::Mode, 0b00001000)?; // SOFKAENAB
        self.wait_start_of_frame()?;

        self.rcvtog = false;
        self.sndtog = false;
        self.peraddr = 0xff;
        self.reg_write(Reg::Hctl, 0b01010000)?; // SNDTOG0, RCVTOG0

        Ok(speed)
    }

    pub fn wait_start_of_frame(&mut self) -> Result<(), Error> {
        self.wait_hirq(Hirq::Frame)
    }

    pub fn set_max_packet_size_0(&mut self, max_packet_size_0: u8) {
        self.max_packet_size_0 = max_packet_size_0;
    }

    pub fn control_in(
        &mut self,
        addr: u8,
        data: Option<&mut [u8]>,
        mut req: Request,
    ) -> Result<usize, Error> {
        req.direction = UsbDirection::In;
        self.send_setup(addr, &req)?;

        let mut total = 0;

        if let Some(data) = data {
            self.reg_write(Reg::Hctl, 0b00100000)?; // RCVTOG1

            while total < data.len() {
                self.do_xfr_wait(0x00, Hxfr::BulkIn)?;

                let count = min(self.max_packet_size_0 as usize, data.len() - total);
                let nread = self.fifo_read(&mut data[total..total + count])?;

                total += nread;

                if nread < self.max_packet_size_0 as usize {
                    break;
                }
            }
        }

        self.do_xfr_wait(0x80, Hxfr::HsOut)?;

        Ok(total)
    }

    pub fn control_out(
        &mut self,
        addr: u8,
        data: Option<&[u8]>,
        mut req: Request,
    ) -> Result<(), Error> {
        req.direction = UsbDirection::Out;
        self.send_setup(addr, &req)?;

        if let Some(data) = data {
            self.reg_write(Reg::Hctl, 0b10000000)?; // SNDTOG1

            let mut total = 0;

            while total < data.len() {
                let count = min(self.max_packet_size_0 as usize, data.len() - total);

                self.fifo_write(Outfifo::Snd, &data[total..total + count])?;

                self.do_xfr_wait(0x00, Hxfr::BulkOut)?;

                total += count;
            }
        }

        self.do_xfr_wait(0x00, Hxfr::HsIn)
    }

    pub fn interrupt_in(
        &mut self,
        addr: u8,
        ep: &mut Endpoint,
        data: &mut [u8],
    ) -> Result<usize, Error> {
        self.set_peraddr(addr)?;

        if ep.tog != self.rcvtog {
            self.reg_write(Reg::Hctl, if ep.tog { 0b00100000 } else { 0b00010000 })?;
        }
        self.do_xfr_wait(ep.addr, Hxfr::BulkIn)?;
        ep.tog = self.rcvtog;

        self.fifo_read(data)
    }

    fn send_setup(&mut self, addr: u8, req: &Request) -> Result<(), Error> {
        let mut setup = [0u8; 8];
        req.serialize(&mut setup);

        self.set_peraddr(addr)?;

        self.fifo_write(Outfifo::Sud, &setup)?;

        self.do_xfr_wait(0x00, Hxfr::Setup)?;

        Ok(())
    }

    fn reset(&mut self, delay: &mut dyn DelayUs<u8>) -> Result<(), Error> {
        self.res.set_low().ok();
        delay.delay_us(10); // 200ns minimum delay, let's be sure
        self.res.set_high().ok();
        delay.delay_us(10); // 200ns minimum delay, let's be sure

        // FDUPSPI
        self.reg_write(Reg::Pinctl, 0b00010000)?;

        self.reg_read(Reg::Revision)?;

        self.reg_write(Reg::Usbctl, 0b00100000)?; // CHIPRES
        self.reg_write(Reg::Usbctl, 0)?; // Clear reset

        self.wait_usbirq(Usbirq::Oscok)?;

        Ok(())
    }

    fn config_host(&mut self) -> Result<(), Error> {
        // DPPULLDN, DMPULLDN, HOST
        self.reg_write(
            Reg::Mode,
            // 0b11000001
            0b11001001,
        )?;

        // Turn on host power (kinda circuit dependent but this is a test so)
        self.reg_set(Reg::Iopins1, 0b00000001)
    }

    fn wait_hirq(&mut self, irq: Hirq) -> Result<(), Error> {
        loop {
            let hirq = self.reg_read(Reg::Hirq)?;
            //writeln!(NonBlockingOutput::new(), "{:08b} {:08b}", hirq, self.reg_read(Reg::Hien)).ok();
            if hirq & (irq as u8) != 0 {
                self.reg_write(Reg::Hirq, irq as u8)?;
                break;
            }
        }

        Ok(())
    }

    fn wait_usbirq(&mut self, irq: Usbirq) -> Result<(), Error> {
        loop {
            let usbirq = self.reg_read(Reg::Usbirq)?;
            //writeln!(NonBlockingOutput::new(), "{:08b} {:08b}", hirq, self.reg_read(Reg::Hien)).ok();
            if usbirq & (irq as u8) != 0 {
                self.reg_write(Reg::Usbirq, irq as u8)?;
                break;
            }
        }

        Ok(())
    }

    fn do_xfr(&mut self, ep: u8, xfr: Hxfr) -> Result<Hrsl, Error> {
        self.reg_write(Reg::Hxfr, (xfr as u8) << 4 | (ep & 0x0f))?;

        loop {
            self.wait_hirq(Hirq::Hxfrdn)?;

            let hrsl = self.reg_read(Reg::Hrsl)?;

            let result = unsafe { core::mem::transmute(hrsl & 0x0f) };

            if result != Hrsl::Busy {
                self.sndtog = (hrsl & 0b00100000) != 0;
                self.rcvtog = (hrsl & 0b00010000) != 0;
                return Ok(result);
            }
        }
    }

    // fn do_xfr(&mut self, ep: u8, xfr: Hxfr) -> Result<Hrsl, Error> {
    //     self.reg_write(Reg::Hxfr, (xfr as u8) << 4 | (ep & 0x0f))?;

    //     self.wait_hirq(Hirq::Hxfrdn)?;

    //     let hrsl = self.reg_read(Reg::Hrsl)?;

    //     self.sndtog = (hrsl & 0b00100000) != 0;
    //     self.rcvtog = (hrsl & 0b00010000) != 0;

    //     if hrsl > 0x0f {
    //         return Err(Error::OutOfRange(hrsl));
    //     }

    //     let result: Hrsl = unsafe { core::mem::transmute(hrsl & 0x0f) };
    //     Ok(result)
    // }

    fn do_xfr_wait(&mut self, ep: u8, xfr: Hxfr) -> Result<(), Error> {
        loop {
            match self.do_xfr(ep, xfr)? {
                Hrsl::Success => return Ok(()),
                Hrsl::Nak => continue,
                err => return Err(Error::Hrsl(err)),
            }
        }
    }

    fn set_peraddr(&mut self, addr: u8) -> Result<(), Error> {
        if addr != self.peraddr {
            self.reg_write(Reg::Peraddr, addr)?;
            self.peraddr = addr;
        }

        Ok(())
    }

    fn fifo_write(&mut self, fifo: Outfifo, data: &[u8]) -> Result<(), Error> {
        let fifo_buf = [fifo as u8];

        let mut ops = [Operation::Write(&fifo_buf), Operation::Write(data)];
        self.exec_spi(&mut ops)?;

        if fifo == Outfifo::Snd {
            self.reg_write(Reg::Sndbc, data.len() as u8)
        } else {
            Ok(())
        }
    }

    // There is only one read fifo
    fn fifo_read(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        if self.reg_read(Reg::Hirq)? & Hirq::Rcvdav as u8 == 0 {
            return Ok(0);
        }

        let count = min(data.len(), self.reg_read(Reg::Rvcbc)? as usize);

        let mut ops = [
            Operation::Write(&[1 << 3]),
            Operation::Transfer(&mut data[..count]),
        ];
        self.exec_spi(&mut ops)?;

        self.reg_write(Reg::Hirq, Hirq::Rcvdav as u8)?;

        Ok(count)
    }

    fn reg_write(&mut self, reg: Reg, value: u8) -> Result<(), Error> {
        let buf = [((reg as u8) << 3) | 0x02, value];

        let mut ops = [Operation::Write(&buf)];
        self.exec_spi(&mut ops)
    }

    fn reg_read(&mut self, reg: Reg) -> Result<u8, Error> {
        let mut buf = [((reg as u8) << 3), 0x00];

        let mut ops = [Operation::Transfer(&mut buf)];
        self.exec_spi(&mut ops)?;

        Ok(buf[1])
    }

    fn reg_set(&mut self, reg: Reg, bits: u8) -> Result<(), Error> {
        let v = self.reg_read(reg)?;
        self.reg_write(reg, v | bits)
    }

    fn reg_clear(&mut self, reg: Reg, bits: u8) -> Result<(), Error> {
        let v = self.reg_read(reg)?;
        self.reg_write(reg, v & !bits)
    }

    fn exec_spi(&mut self, ops: &mut [Operation<u8>]) -> Result<(), Error> {
        self.spi.exec_spi(ops)
    }
}
