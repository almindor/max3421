use core::cmp::min;
use embedded_hal::delay::blocking::DelayUs;
use embedded_hal::digital::blocking::OutputPin;

use embedded_hal::spi::blocking::{SpiDevice, SpiBus, SpiBusWrite};
use hifive1::sprintln;

use super::*;

pub struct Max3421<SPI, RES> {
    spi: SPI,
    res: Option<RES>,
    peraddr: u8,
    sndtog: bool,
    rcvtog: bool,
    max_packet_size_0: u8,
    status: Option<u8>,
}

static mut PREV_HRSL: u8 = 0;

fn print_hrsl(hrsl: u8, prefix: &str) {
    let j = (0b1000_0000u8 & hrsl) >> 7;
    let k = (0b0100_0000u8 & hrsl) >> 6;
    let hrslt = 0b0000_1111u8 & hrsl;

    unsafe {
        if hrsl != PREV_HRSL {
            sprintln!(
                "\t{} HRSL RAW: {:#010b} J: {} K: {} HRSLT: {:#02x}",
                prefix,
                hrsl,
                j,
                k,
                hrslt
            );

            PREV_HRSL = hrsl;
        }
    }
}

impl<SPI, RES> Max3421<SPI, RES>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    RES: OutputPin,
{
    pub fn new(spi: SPI, res: Option<RES>) -> Self {
        Self {
            spi,
            res,
            peraddr: 0xff,
            sndtog: false,
            rcvtog: false,
            max_packet_size_0: 8,
            status: None,
        }
    }

    pub fn release(self) -> (SPI, Option<RES>) {
        (self.spi, self.res)
    }

    pub fn init<D>(&mut self, delay: &mut D) -> Result<(), Error>
    where
        D: DelayUs,
    {
        // FDUPSPI - set SPI full duplex
        self.reg_write(Reg::Pinctl, 0b00010000)?;

        self.reset(delay)?;

        // wait for internal clock to be ready
        match self.wait_usbirq(Usbirq::Oscok) {
            Err(Error::Timeout(_)) => sprintln!("\tOSCOK timeout, ignoring..."),
            Err(err) => return Err(err),
            Ok(()) => {}
        }

        // Turn off extra gpout pins
        self.reg_set(Reg::Iopins1, 0b00000000)?;
        self.reg_set(Reg::Iopins2, 0b00000000)?;

        // DPPULLDN, DMPULLDN, DELAYISO, HOST
        self.reg_write(Reg::Mode, 0b11100001)?; // mode setup

        Ok(())
    }

    pub fn revision(&mut self) -> Result<u8, Error> {
        self.reg_read(Reg::Revision)
    }

    fn set_guards(&mut self, hrsl: u8) {
        self.sndtog = (hrsl & 0b0010_0000) >> 5 > 0;
        self.rcvtog = (hrsl & 0b0001_0000) >> 4 > 0;
    }

    fn hctl(&self, mut val: u8) -> u8 {
        if self.sndtog {
            val |= 0b0100_0000;
        } else {
            val |= 0b1000_0000;
        }

        if self.rcvtog {
            val |= 0b0010_0000;
        } else {
            val |= 0b0001_0000;
        }

        val
    }

    fn query_jk(&mut self, prefix: &str) -> Result<u8, Error> {
        let hrsl = self.hrsl_read()?;
        print_hrsl(hrsl, prefix);

        Ok(hrsl >> 6)
    }

    pub fn wait_for_connection(&mut self) -> Result<Speed, Error> {
        // clear CONDETIRQ
        self.reg_write(Reg::Hirq, Hirq::Condet as u8)?;

        self.reg_write(Reg::Hctl, 0b00000100)?; // SAMPLEBUS
                                                // self.wait_samplebus()?;

        // query for J/K directly in case we have USB pre-inserted
        let jk = self.query_jk("[C]")?;
        if jk == 0 {
            // wait for CONDET
            self.wait_hirq(Hirq::Condet)?;
        }

        let speed = loop {
            // let hctl = self.hctl(0b00000100);
            // sprintln!("HCTL SAMPLEBUS: {:#010b}", hctl);

            let jk = self.query_jk("[C]")?;

            match jk {
                0b10 => break Speed::High,
                0b01 => break Speed::Low,
                0b11 => return Err(Error::InvalidJK(jk)),
                _ => {
                    continue;
                }
            }
        };

        let mode;
        let hctl;
        if speed == Speed::Low {
            mode = 0b0000_1010; // SOFKAENAB, LOWSPEED
            hctl = 0b0000_1000; // BUSRST
        } else {
            mode = 0b0000_1000; // SOFKAENAB
            hctl = 0b0000_1010; // BUSRST, FRMRST
        }

        self.reg_write(Reg::Hirq, 0b0000_0001)?; // clear BUSRST IRQ
        self.reg_set(Reg::Hctl, hctl)?; // BUSRST + FRMRST if high speed
        self.wait_hirq(Hirq::Busevent)?; // wait for BUSRST

        self.reg_set(Reg::Mode, mode)?; // SOFKAENAB + LOWSPEED if low speed
        self.wait_start_of_frame()?;

        self.rcvtog = false;
        self.sndtog = false;
        self.peraddr = 0xff;
        self.reg_write(Reg::Hctl, 0b01010000)?; // SNDTOG0, RCVTOG0

        Ok(speed)
    }

    pub fn wait_for_disconnect(&mut self) -> Result<(), Error> {
        // let hctl = self.hctl(0b00000100);
        // sprintln!("HCTL SAMPLEBUS: {:#010b}", hctl);
        self.reg_write(Reg::Hctl, 0b00000100)?; // SAMPLEBUS
        self.wait_hirq(Hirq::Condet)?;

        loop {
            let jk = self.query_jk("[DC]")?;
            if jk == 0 {
                break;
            } else {
                continue;
            }
        }

        self.reg_write(Reg::Mode, 0b1110_0001)?; // HOST mode reset to defaults

        Ok(())
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
                sprintln!("nread = {}", nread);

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

    fn reset<D>(&mut self, delay: &mut D) -> Result<(), Error>
    where
        D: DelayUs,
    {
        if let Some(res) = &mut self.res {
            res.set_low().map_err(|_| Error::PinRST)?;
            delay.delay_us(50).map_err(|_| Error::DelayError)?; // 200ns minimum delay, let's be sure
            res.set_high().map_err(|_| Error::PinRST)?;
            delay.delay_us(50).map_err(|_| Error::DelayError)?; // 200ns minimum delay, let's be sure
        }

        // SW CHIPRST
        self.reg_write(Reg::Usbctl, 0b00100000)?; // CHIPRES
        self.reg_write(Reg::Usbctl, 0)?; // Clear reset
        delay.delay_us(50).map_err(|_| Error::DelayError)?; // 200ns minimum delay, let's be sure

        Ok(())
    }

    fn wait_hirq(&mut self, irq: Hirq) -> Result<(), Error> {
        let mut prev_hirq = None;
        loop {
            let hirq = self.reg_read(Reg::Hirq)?;
            if prev_hirq != Some(hirq) {
                sprintln!("\tHIRQ: {:#010b}", hirq);
                prev_hirq = Some(hirq);
            }

            //writeln!(NonBlockingOutput::new(), "{:08b} {:08b}", hirq, self.reg_read(Reg::Hien)).ok();
            if hirq & (irq as u8) != 0 {
                self.reg_write(Reg::Hirq, irq as u8)?;
                sprintln!("\tHIRQ CLEARED: {:#010b}", irq as u8);
                break;
            }
        }

        Ok(())
    }

    fn wait_usbirq(&mut self, irq: Usbirq) -> Result<(), Error> {
        let mut prev_val = None;

        loop {
            let usbirq = self.reg_read(Reg::Usbirq)?;
            if prev_val.unwrap_or(!usbirq) != usbirq {
                sprintln!("\tUSBIRQ: {:#010b}", usbirq);
                prev_val = Some(usbirq);
            }

            if usbirq & (irq as u8) != 0 {
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

        // let mut ops = [Operation::Write(&fifo_buf), Operation::Write(data)];
        self.spi.transaction(|bus| {
            bus.write(&fifo_buf)?;
            bus.write(data)
        }).map_err(|_| Error::SpiError)?;

        if fifo == Outfifo::Snd {
            self.reg_write(Reg::Sndbc, data.len() as u8)?;
        }

        Ok(())
    }

    // There is only one read fifo
    fn fifo_read(&mut self, data: &mut [u8]) -> Result<usize, Error> {
        if self.reg_read(Reg::Hirq)? & Hirq::Rcvdav as u8 == 0 {
            return Ok(0);
        }

        let count = min(data.len(), self.reg_read(Reg::Rvcbc)? as usize);

        self.spi.transaction(|bus| {
            bus.write(&[1 << 3])?;
            bus.transfer_in_place(&mut data[..count])
        }).map_err(|_| Error::SpiError)?;

        self.reg_write(Reg::Hirq, Hirq::Rcvdav as u8)?;

        Ok(count)
    }

    fn reg_write(&mut self, reg: Reg, value: u8) -> Result<u8, Error> {
        let mut buf = [((reg as u8) << 3) | 0x02, value];

        // sprintln!("REG WRITE[{:?}]: {:#010b}", reg, value);

        // let mut ops = [Operation::Transfer(&mut buf)];
        self.spi.transaction(|bus| {
            bus.transfer_in_place(&mut buf)
        }).map_err(|_| Error::SpiError)?;

        self.set_status(buf[0]);
        Ok(buf[0]) // return status byte for every write after FDUPSPI = 1
    }

    fn hrsl_read(&mut self) -> Result<u8, Error> {
        let hrsl = self.reg_read(Reg::Hrsl)?;

        self.set_guards(hrsl);

        Ok(hrsl)
    }

    fn reg_read(&mut self, reg: Reg) -> Result<u8, Error> {
        let mut buf = [((reg as u8) << 3), 0x00];

        // let mut ops = [Operation::Transfer(&mut buf)];
        self.spi.transaction(|bus| {
            bus.transfer_in_place(&mut buf)
        }).map_err(|_| Error::SpiError)?;

        Ok(buf[1])
    }

    fn reg_set(&mut self, reg: Reg, bits: u8) -> Result<u8, Error> {
        let v = self.reg_read(reg)?;
        self.reg_write(reg, v | bits)
    }

    fn set_status(&mut self, val: u8) {
        if self.status != Some(val) {
            // sprintln!("\tSTATUS: {:#010b}", val);
            self.status = Some(val);
        }
    }
}
