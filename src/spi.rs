//! Serial Peripheral Interface
//!
//! You can use the `Spi` interface with these SPI instances
//!
//! # SPI1
//!
//! - NSS = PA4
//! - SCK = PA5
//! - MISO = PA6
//! - MOSI = PA7
//!
//! # SPI2
//!
//! - NSS = PB12
//! - SCK = PB13
//! - MISO = PB14
//! - MOSI = PB15

use core::any::{Any};
use core::ops::Deref;
use core::ptr;

use hal;
use nb;
use stm32f30x::{gpioa, SPI1, spi1, GPIOA, GPIOE, RCC};

/// SPI instance that can be used with the `Spi` abstraction
pub unsafe trait SPI: Deref<Target = spi1::RegisterBlock> {
    /// GPIO block associated to this SPI instance
    type GPIO: Deref<Target = gpioa::RegisterBlock>;
}

unsafe impl SPI for SPI1 {
    type GPIO = GPIOA;
}

/// SPI result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// SPI error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
    /// CRC error
    Crc,
    #[doc(hidden)]
    _Extensible,
}

/// Serial Peripheral Interface
pub struct Spi<'a, S>(pub &'a S)
where S: Any + SPI;

impl<'a, S> Spi<'a, S>
where S: Any + SPI,
{
    /// Initializes the SPI
    ///
    /// STEPS
    /// -----
    /// // Enable peripheral clock using RCC APB
        // Enable SCK, MOSI, MISO and NSS GPIO clocks using RCC AHB
        // Connect & configure the pin to the desired peripherals' Alternate Function (AF)
        // Program the Polarity, Phase, First Data, Baud Rate Prescaler, Slave Management, Peripheral Mode and CRC Polynomial values using the SPI_Init()

    pub fn init(&self, gpio: &S::GPIO, enable: &GPIOE, rcc: &RCC) {
        let spi = self.0;

        rcc.apb2enr.modify(|_, w| w.spi1en().set_bit());

        // GPIOA: configure PA5, PA6 and PA7 for SPI use
        // AFRL5 = 5 (SPI1_SCK)
        // AFRL6 = 5 (SPI1_MISO)
        // AFRL7 = 5 (SPI1_MOSI)
        // MODER* = 0b10 (Alternate function)
        rcc.ahbenr.modify(|_, w| w.iopaen().set_bit().iopeen().set_bit());
        gpio.afrl.modify(|_, w| unsafe {w.afrl5().bits(5).afrl6().bits(5).afrl7().bits(5)});
        gpio.moder.modify(|_, w| w.moder5().alternate().moder6().alternate().moder7().alternate());


        // GPIOE: configure PE3 as output and drive it low to enable SPI mode
        enable.moder.modify(|_, w| w.moder3().output());
        enable.bsrr.write(|w| w.bs3().set());

        /* Configure SPIx: direction, NSS management, first transmitted bit, BaudRate prescaler master/slave mode, CPOL and CPHA */
        /* Set BIDImode, BIDIOE and RxONLY bits according to SPI_Direction value */
        /* Set SSM, SSI and MSTR bits according to SPI_Mode and SPI_NSS values */
        /* Set LSBFirst bit according to SPI_FirstBit value */
        /* Set BR bits according to SPI_BaudRatePrescaler value */
        /* Set CPOL bit according to SPI_CPOL value */
        /* Set CPHA bit according to SPI_CPHA value */
        spi.cr1.write(|w| unsafe {
            w.bidimode()
             .clear_bit()
             .rxonly()
             .clear_bit()
             .ssm()
             .set_bit()
             .ssi()
             .set_bit()
             .lsbfirst()
             .clear_bit()
             .br()
             .bits(0b010)
             .mstr()
             .set_bit()
             .cpol()
             .clear_bit()
             .cpha()
             .clear_bit()
        });

        // FRXTH: RXNE threshold is 8-bit
        // DS: 8-bit data
        // SSOE: disable output on the NSS pin
        spi.cr2.write(|w| unsafe {
            w.frxth()
                .set_bit()
                .ds()
                .bits(0b0111)
                .frxth()
                .set_bit()
                .ssoe()
                .clear_bit()
        });

        spi.cr1.modify(|_, w| w.spe().set_bit());
    }

    /// Disables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin high
    pub fn disable(&self) {
        self.0.cr1.modify(|_, w| w.spe().clear_bit())
    }

    /// Enables the SPI bus
    ///
    /// **NOTE** This drives the NSS pin low
    pub fn enable(&self) {
        self.0.cr1.modify(|_, w| w.spe().set_bit())
    }
}


impl<'a, S> hal::Spi<u8> for Spi<'a, S>
where
    S: Any + SPI,
{
    type Error = Error;

    fn read(&self) -> Result<u8> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.rxne().bit_is_set() {
            Ok(unsafe {
                ptr::read_volatile(&spi1.dr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    fn send(&self, byte: u8) -> Result<()> {
        let spi1 = self.0;
        let sr = spi1.sr.read();

        if sr.ovr().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.modf().bit_is_set() {
            Err(nb::Error::Other(Error::ModeFault))
        } else if sr.crcerr().bit_is_set() {
            Err(nb::Error::Other(Error::Crc))
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            unsafe {
                ptr::write_volatile(&spi1.dr as *const _ as *mut u8, byte)
            }
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
