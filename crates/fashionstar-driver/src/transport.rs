//! Byte transport under [`crate::FashionStarBus`]: a real serial port, or a
//! mock that replays canned replies so the bus logic can be tested without
//! hardware.

use std::collections::VecDeque;
use std::io;
use std::time::Duration;

/// Minimal byte-pipe the bus needs.
pub trait Transport {
    /// Write a whole frame.
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()>;

    /// Read whatever is available, waiting at most `timeout` for the first
    /// byte. "Nothing arrived" may be reported as `Ok(0)` or as an error of
    /// kind `TimedOut` / `WouldBlock`; the bus treats all three alike.
    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize>;

    /// Drop everything already received and not yet read. Must not block.
    fn discard_input(&mut self) -> io::Result<()>;
}

impl Transport for Box<dyn serialport::SerialPort> {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        io::Write::write_all(self, bytes)?;
        io::Write::flush(self)
    }

    fn read(&mut self, buf: &mut [u8], timeout: Duration) -> io::Result<usize> {
        // Setting the timeout per call lets the read block right up to the
        // transaction deadline and still wake the moment a byte lands,
        // instead of polling in fixed slices. On the `serialport` backends
        // this only stores the value — no syscall.
        self.set_timeout(timeout.max(Duration::from_micros(100)))
            .map_err(io::Error::other)?;
        io::Read::read(self, buf)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        // tcflush / PurgeComm: non-blocking, unlike reading until quiet.
        self.clear(serialport::ClearBuffer::Input)
            .map_err(io::Error::other)
    }
}

/// Produces the bytes the "servos" send back after each written frame.
pub type Responder = Box<dyn FnMut(&[u8]) -> Vec<u8> + Send>;

/// In-memory transport for tests.
///
/// Every [`Transport::write_all`] is recorded and handed to a responder,
/// whose return value becomes readable input — so replies only appear after
/// the request that caused them, as on a real bus. Reads hand out at most
/// [`Self::set_max_read`] bytes at a time so the partial-frame paths of the
/// parser get exercised too.
pub struct MockTransport {
    written: Vec<Vec<u8>>,
    rx: VecDeque<u8>,
    responder: Responder,
    max_read: usize,
}

impl MockTransport {
    /// A mock whose replies are computed from each request.
    pub fn with_responder(responder: impl FnMut(&[u8]) -> Vec<u8> + Send + 'static) -> Self {
        Self {
            written: Vec::new(),
            rx: VecDeque::new(),
            responder: Box::new(responder),
            max_read: usize::MAX,
        }
    }

    /// A mock that answers the n-th write with the n-th entry of `replies`
    /// (an empty entry means "no reply"), and nothing once they run out.
    pub fn scripted(replies: Vec<Vec<u8>>) -> Self {
        let mut queue: VecDeque<Vec<u8>> = replies.into();
        Self::with_responder(move |_| queue.pop_front().unwrap_or_default())
    }

    /// A mock that never answers.
    pub fn silent() -> Self {
        Self::with_responder(|_| Vec::new())
    }

    /// Cap each read at `n` bytes (default: unlimited).
    pub fn set_max_read(&mut self, n: usize) {
        self.max_read = n.max(1);
    }

    /// Inject bytes as if they had arrived unprompted (e.g. a late reply).
    pub fn inject(&mut self, bytes: &[u8]) {
        self.rx.extend(bytes);
    }

    /// Every frame written so far, in order.
    pub fn written(&self) -> &[Vec<u8>] {
        &self.written
    }

    /// Bytes received but not yet read.
    pub fn pending_input(&self) -> usize {
        self.rx.len()
    }
}

impl Transport for MockTransport {
    fn write_all(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.written.push(bytes.to_vec());
        let reply = (self.responder)(bytes);
        self.rx.extend(reply);
        Ok(())
    }

    fn read(&mut self, buf: &mut [u8], _timeout: Duration) -> io::Result<usize> {
        if self.rx.is_empty() {
            return Err(io::ErrorKind::TimedOut.into());
        }
        let n = buf.len().min(self.max_read).min(self.rx.len());
        for (slot, b) in buf.iter_mut().zip(self.rx.drain(..n)) {
            *slot = b;
        }
        Ok(n)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.rx.clear();
        Ok(())
    }
}
