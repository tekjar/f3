//! Direct Memory Access (DMA)

use core::cell::{Cell, UnsafeCell};
use core::marker::PhantomData;
use core::ops;

use nb;
use stm32f30x::DMA1;

/// DMA error
#[derive(Debug)]
pub enum Error {
    /// DMA channel in use
    InUse,
    /// Previous data got overwritten before it could be read because it was
    /// not accessed in a timely fashion
    Overrun,
    /// Transfer error
    Transfer,
}

/// Channel 2 of DMA1
pub struct Dma1Channel2 {
    _0: (),
}

/// Channel 4 of DMA1
pub struct Dma1Channel4 {
    _0: (),
}

/// Channel 5 of DMA1
pub struct Dma1Channel5 {
    _0: (),
}

/// Buffer to be used with a certain DMA `CHANNEL`
// NOTE(packed) workaround for rust-lang/rust#41315
#[repr(packed)]
pub struct Buffer<T, CHANNEL> {
    data: UnsafeCell<T>,
    flag: Cell<BorrowFlag>,
    state: Cell<State>,
    _marker: PhantomData<CHANNEL>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    // A new `Buffer` starts in this state. We set it to zero to place this
    // buffer in the .bss section
    Unlocked = 0,

    Locked,
    MutLocked,
}

type BorrowFlag = usize;

const UNUSED: BorrowFlag = 0;
const WRITING: BorrowFlag = !0;

/// Wraps a borrowed reference to a value in a `Buffer`
pub struct Ref<'a, T>
where
    T: 'a,
{
    data: &'a T,
    flag: &'a Cell<BorrowFlag>,
}

impl<'a, T> ops::Deref for Ref<'a, T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.data
    }
}

impl<'a, T> Drop for Ref<'a, T> {
    fn drop(&mut self) {
        self.flag.set(self.flag.get() - 1);
    }
}

/// A wrapper type for a mutably borrowed value from a `Buffer``
pub struct RefMut<'a, T>
where
    T: 'a,
{
    data: &'a mut T,
    flag: &'a Cell<BorrowFlag>,
}

impl<'a, T> ops::Deref for RefMut<'a, T> {
    type Target = T;

    fn deref(&self) -> &T {
        self.data
    }
}

impl<'a, T> ops::DerefMut for RefMut<'a, T> {
    fn deref_mut(&mut self) -> &mut T {
        self.data
    }
}

impl<'a, T> Drop for RefMut<'a, T> {
    fn drop(&mut self) {
        self.flag.set(UNUSED);
    }
}

impl<T, CHANNEL> Buffer<T, CHANNEL> {
    /// Creates a new buffer
    pub const fn new(data: T) -> Self {
        Buffer {
            _marker: PhantomData,
            data: UnsafeCell::new(data),
            flag: Cell::new(0),
            state: Cell::new(State::Unlocked),
        }
    }

    /// Immutably borrows the wrapped value.
    ///
    /// The borrow lasts until the returned `Ref` exits scope. Multiple
    /// immutable borrows can be taken out at the same time.
    ///
    /// # Panics
    ///
    /// Panics if the value is currently mutably borrowed.
    pub fn borrow(&self) -> Ref<T> {
        assert_ne!(self.flag.get(), WRITING);

        self.flag.set(self.flag.get() + 1);

        Ref {
            data: unsafe { &*self.data.get() },
            flag: &self.flag,
        }
    }

    /// Mutably borrows the wrapped value.
    ///
    /// The borrow lasts until the returned `RefMut` exits scope. The value
    /// cannot be borrowed while this borrow is active.
    ///
    /// # Panics
    ///
    /// Panics if the value is currently borrowed.
    pub fn borrow_mut(&self) -> RefMut<T> {
        assert_eq!(self.flag.get(), UNUSED);

        self.flag.set(WRITING);

        RefMut {
            data: unsafe { &mut *self.data.get() },
            flag: &self.flag,
        }
    }

    pub(crate) fn lock(&self) -> &T {
        assert_eq!(self.state.get(), State::Unlocked);
        assert_ne!(self.flag.get(), WRITING);

        self.flag.set(self.flag.get() + 1);
        self.state.set(State::Locked);

        unsafe { &*self.data.get() }
    }

    pub(crate) fn lock_mut(&self) -> &mut T {
        assert_eq!(self.state.get(), State::Unlocked);
        assert_eq!(self.flag.get(), UNUSED);

        self.flag.set(WRITING);
        self.state.set(State::MutLocked);

        unsafe { &mut *self.data.get() }
    }

    unsafe fn unlock(&self, state: State) {
        match state {
            State::Locked => self.flag.set(self.flag.get() - 1),
            State::MutLocked => self.flag.set(UNUSED),
            _ => { /* unreachable!() */ }
        }

        self.state.set(State::Unlocked);
    }
}

// FIXME these `release` methods probably want some of sort of barrier
impl<T> Buffer<T, Dma1Channel2> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let state = self.state.get();

        if state == State::Unlocked {
            return Ok(());
        }

        if dma1.isr.read().teif2().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.isr.read().tcif2().bit_is_set() {
            unsafe { self.unlock(state) }
            dma1.ifcr.write(|w| w.ctcif2().set_bit());
            dma1.ccr2.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T> Buffer<T, Dma1Channel4> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let state = self.state.get();

        if state == State::Unlocked {
            return Ok(());
        }

        if dma1.isr.read().teif4().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.isr.read().tcif4().bit_is_set() {
            unsafe { self.unlock(state) }
            dma1.ifcr.write(|w| w.ctcif4().set_bit());
            dma1.ccr4.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl<T> Buffer<T, Dma1Channel5> {
    /// Waits until the DMA releases this buffer
    pub fn release(&self, dma1: &DMA1) -> nb::Result<(), Error> {
        let state = self.state.get();

        if state == State::Unlocked {
            return Ok(());
        }

        if dma1.isr.read().teif5().bit_is_set() {
            Err(nb::Error::Other(Error::Transfer))
        } else if dma1.isr.read().tcif5().bit_is_set() {
            unsafe { self.unlock(state) }
            dma1.ifcr.write(|w| w.ctcif5().set_bit());
            dma1.ccr5.modify(|_, w| w.en().clear_bit());
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
