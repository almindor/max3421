#![no_std]

mod control;
mod max;
mod spi_interface;

pub use control::*;
pub use max::*;
pub use spi_interface::*;

#[derive(Debug)]
pub enum Error {
    TransferSPI,
    WriteSPI,
    ExecSPI,
    PinRST,
    PinCS,
    Hrsl(Hrsl),
    OutOfRange(u8),
    Timeout(u32),
    InvalidJK(u8),
}

pub struct Endpoint {
    addr: u8,
    tog: bool,
}

impl Endpoint {
    pub fn new(addr: u8) -> Self {
        Self { addr, tog: false }
    }
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Reg {
    Rvcbc = 6,
    Sndbc = 7,
    Usbirq = 13,
    Usbien = 14,
    Usbctl = 15,
    Cpuctl = 16,
    Pinctl = 17,
    Revision = 18,
    Iopins1 = 20,
    Iopins2 = 21,
    Gpinirq = 22,
    Gpinien = 23,
    Gpinpol = 24,
    Hirq = 25,
    Hien = 26,
    Mode = 27,
    Peraddr = 28,
    Hctl = 29,
    Hxfr = 30,
    Hrsl = 31,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hirq {
    Busevent = 0x01,
    Rwu = 0x02,
    Rcvdav = 0x04,
    Sndbav = 0x08,
    Susdn = 0x10,
    Condet = 0x20,
    Frame = 0x40,
    Hxfrdn = 0x80,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Usbirq {
    Oscok = 0b01,
    Novbus = 0x20,
    Vbus = 0x40,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hrsl {
    Success = 0x00,
    Busy = 0x01,
    Badreq = 0x02,
    Undef = 0x03,
    Nak = 0x04,
    Stall = 0x05,
    Togerr = 0x06,
    Wrongpid = 0x07,
    Badbc = 0x08,
    Piderr = 0x09,
    Pkterr = 0x0a,
    Crcerr = 0x0b,
    Kerr = 0x0c,
    Jerr = 0x0d,
    Timeout = 0x0e,
    Babble = 0x0f,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Hxfr {
    Setup = 0b0001,
    BulkIn = 0b0000,
    BulkOut = 0b0010,
    HsIn = 0b1000,
    HsOut = 0b1010,
    IsoIn = 0b0100,
    IsoOut = 0b0110,
}

#[repr(u8)]
#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Outfifo {
    Snd = (2 << 3) | 0x02,
    Sud = (4 << 3) | 0x02,
}

#[derive(Eq, PartialEq, Copy, Clone, Debug)]
#[allow(unused)]
pub enum Speed {
    High,
    Low,
}
