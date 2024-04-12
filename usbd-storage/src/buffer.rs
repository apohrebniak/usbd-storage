use core::borrow::BorrowMut;
use core::cmp::min;

pub struct Buffer<T: BorrowMut<[u8]>> {
    inner: T,
    rpos: usize, // next byte to read from
    wpos: usize, // next byte to write into
}

impl<T: BorrowMut<[u8]>> Buffer<T> {
    pub fn new(inner: T) -> Buffer<T> {
        Buffer {
            inner,
            rpos: 0,
            wpos: 0,
        }
    }

    pub fn available_read(&self) -> usize {
        self.wpos - self.rpos
    }

    pub fn available_write(&self) -> usize {
        self.inner.borrow().len() - self.wpos
    }

    /// Returns number of bytes actually written
    pub fn write(&mut self, data: &[u8]) -> usize {
        if self.available_write() < data.len() {
            self.shift();
        }
        let count = min(self.available_write(), data.len());
        let inner = self.inner.borrow_mut();
        inner[self.wpos..(self.wpos + count)].copy_from_slice(&data[..count]);
        self.wpos += count;
        debug_assert!(self.wpos <= inner.len());
        count
    }

    pub fn write_all<E>(
        &mut self,
        max_count: usize,
        overflow_err: E,
        f: impl FnOnce(&mut [u8]) -> Result<usize, E>,
    ) -> Result<usize, E> {
        if self.available_write() < max_count {
            self.shift();
            if self.available_write() < max_count {
                return Err(overflow_err);
            }
        }

        let inner = self.inner.borrow_mut();

        f(&mut inner[self.wpos..(self.wpos + max_count)]).map(|count| {
            let advance_by = min(count, inner.len() - self.wpos);
            self.wpos += advance_by;
            debug_assert!(self.wpos <= inner.len());
            advance_by
        })
    }

    pub fn read<E>(&mut self, f: impl FnOnce(&mut [u8]) -> Result<usize, E>) -> Result<usize, E> {
        let boundary = self.rpos + self.available_read();
        let inner = self.inner.borrow_mut();
        f(&mut inner[self.rpos..boundary]).map(|count| {
            let advance_by = min(count, self.available_read());
            self.rpos += advance_by;
            debug_assert!(self.rpos <= self.wpos);
            advance_by
        })
    }

    pub fn clean(&mut self) {
        self.rpos = 0;
        self.wpos = 0;
    }

    fn shift(&mut self) {
        if self.rpos != self.wpos {
            unsafe {
                core::ptr::copy(
                    &self.inner.borrow()[self.rpos] as *const u8,
                    &mut self.inner.borrow_mut()[0] as *mut u8,
                    self.available_read(),
                )
            }
            self.wpos -= self.rpos;
            self.rpos = 0;
        } else {
            self.clean();
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::buffer::Buffer;

    const DATA: [u8; 10] = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9];

    #[test]
    fn write_when_space_available() {
        let mut buf = Buffer::new([0u8; 8]);
        assert_eq!(5, buf.write(&DATA[..5]));
        assert_eq!(5, buf.available_read());
        assert_eq!(3, buf.available_write());
    }

    #[test]
    fn write_shift() {
        let mut buf = Buffer::new([0u8; 10]);
        // write
        assert_eq!(8, buf.write(&DATA[..8]));
        assert_eq!(8, buf.available_read());
        assert_eq!(2, buf.available_write());

        // read some data
        assert_eq!(
            Ok::<usize, ()>(7),
            buf.read(|buf| {
                assert_eq!(8, buf.len());
                Ok(7)
            })
        );
        assert_eq!(1, buf.available_read());
        assert_eq!(2, buf.available_write());

        // write again
        assert_eq!(5, buf.write(&DATA[..5]));
        assert_eq!(6, buf.available_read());
        assert_eq!(4, buf.available_write());
    }

    #[test]
    fn write_all_shift() {
        let mut buf = Buffer::new([0u8; 10]);
        let wpos = 6;
        let rpos = 5;
        buf.wpos = wpos;
        buf.rpos = rpos;

        let res = buf.write_all(6, (), |_buf| Ok(6));
        assert_eq!(Ok(6), res);
        assert_eq!(0, buf.rpos);
        assert_eq!(7, buf.wpos);
    }

    #[test]
    fn write_full_read_full() {
        let mut buf = Buffer::new([0u8; 10]);
        // write full
        assert_eq!(10, buf.write(&DATA[..10]));
        assert_eq!(10, buf.available_read());
        assert_eq!(0, buf.available_write());

        // read full
        assert_eq!(
            Ok::<usize, ()>(10),
            buf.read(|buf| {
                assert_eq!(10, buf.len());
                Ok(10)
            })
        );
        assert_eq!(0, buf.available_read());
        assert_eq!(0, buf.available_write());

        // write full again
        assert_eq!(10, buf.write(&DATA[..10]));
        assert_eq!(10, buf.available_read());
        assert_eq!(0, buf.available_write());
    }
}
