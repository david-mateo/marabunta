from serial import Serial
import thread
import threading
from math import pi


def clean_angle(th):
    """Return the angle th
    wrapped into the range [-pi,pi].
    """
    while th > pi:
        th -= 2 * pi
    while th < -pi:
        th += 2 * pi
    return th


class SafeSerial(Serial):
    """Extension of serialSerial that uses a
    threading.Lock lock to make sure that reads
    and writes are thread-safe. The lock can be
    given at initialization so that different
    serial port can share the same lock.
    Sharing the lock is useful when dealing
    with multiplexed ports such as in the
    Raspberry Pi 2.
    """
    def __init__(self, *args, **kws):
        lock = kws.pop("lock", None)
        if isinstance(lock, thread.LockType):
            self.lock = lock
        else:
            self.lock = threading.Lock()
        super(SafeSerial, self).__init__(*args, **kws)
        return

    def readline(self):
        with self.lock:
            m = super(SafeSerial, self).readline()
        return m

    def write(self, *args, **kws):
        with self.lock:
            m = super(SafeSerial, self).write(*args, **kws)
        return m

    def flushInput(self):
        with self.lock:
            m = super(SafeSerial, self).flushInput()
        return m

    def flushOutput(self):
        with self.lock:
            m = super(SafeSerial, self).flushOutput()
        return m
