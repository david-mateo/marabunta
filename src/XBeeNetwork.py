from random import randint
from time import time, sleep
import serial
import threading, thread, Queue
import glob, sys
from BaseRobot import BaseNetwork

#...............................................................................................
#TODO Is there a better way that defining this class to
# make this library work on the Raspberry Pi?
class SafeSerial(serial.Serial):
    """Extension of serialSerial that uses a
    threading.Lock lock to make sure that reads
    and writes are thread-safe. The lock can be
    given at initialization so that different
    serial port can share the same lock.
    Sharing the lock can be useful in some cases
    such as when using a Raspberry Pi, where all
    the ports seem to be physically a single one
    (and hence threaded writes in different ports
    can give problems).
    """
    def __init__(self, *args, **kws):
        lock = kws.pop("lock",None)
        if isinstance(lock,thread.LockType):
            self.lock = lock
        else:
            self.lock = threading.Lock()
        super(SafeSerial, self).__init__(*args,**kws)
        return

    def readline(self):
        with self.lock:
            m = super(SafeSerial, self).readline()
        return m

    def write(self, *args,**kws):
        with self.lock:
            m = super(SafeSerial, self).write(*args,**kws)
        return m

    def flushInput(self):
        with self.lock:
            m = super(SafeSerial, self).flushInput()
        return m

    def flushOutput(self):
        with self.lock:
            m = super(SafeSerial, self).flushOutput()
        return m

#...............................................................................................
class XBeeNetwork(BaseNetwork):
    """Network class for communication using XBee series 1
    connected as a serial port in /dev/ttyUSB*.
    Messages are only sent at particular time slots specified
    by *window_start*, *window_end*, and *period*.
    The incoming data in the serial port is continually
    scanned for new messages.
    """
    #TODO More general connection than serial port to /dev/ttyUSB* should be considered?
    def __init__(self, window_start, window_end, period, ID=None, lock=None):
        assert period > 0.
        assert window_start >= 0. and window_start < period
        assert window_end > window_start and window_end <= period
        self.window_start = window_start
        self.window_end = window_end
        self.period = period
        if ID:
            self.ID = str(ID)
        else:
            self.ID = str(randint(0,999999))
        self.lock = lock
        self.broadcasting = False
        self.port = None
        self.positions = {}
        self.outbox = Queue.LifoQueue()
        return

    def start_broadcasting(self):
        """Open a serial connection to XBee and start
        one thread for sending messages and one thread
        for reading messages.
        The connection to XBee is done by opening
        a SafeSerial port connection to /dev/ttyUSB*
        This method does nothing if the network is
        already broadcasting (self.broadcasting=True).
        """
        if not self.broadcasting:
            port_paths = glob.glob('/dev/ttyUSB*')
            connected=False
            for port_path in port_paths:
                try:
                    port = SafeSerial(port_path,baudrate=115200, timeout=0.100, writeTimeout=0.100, lock=self.lock)
                    connected=True
                except:
                    connected=False
                if connected:
                    self.port = port
                    for i in range(3):
                        self.port.flushInput()
                        self.port.flushOutput()
                    break
            if not connected:
                self.port = None
                raise Exception("XBeeNetwork: could not connect to Xbee")
            self.broadcasting = True
            self.send_thread = threading.Thread(target = self.send_background)
            self.send_thread.start()
            self.read_thread = threading.Thread(target = self.read_background)
            self.read_thread.start()
        return self.port

    def stop_broadcasting(self):
        """Stop the send and read threads and turn off
        broadcasting. This function returns when the
        threads have been succesfully terminated or else
        raises and Exception.
        This method does nothing if the network is
        not broadcasting already (self.broadcasting=False).
        Returns the number of messages left to send.
        """
        if self.broadcasting:
            self.broadcasting = False
            if self.send_thread.is_alive():
                self.send_thread.join(5)
            if self.read_thread.is_alive():
                self.read_thread.join(5)
            if self.send_thread.is_alive() or self.read_thread.is_alive():
                raise Exception("XBeeNetwork: Could not stop background threads properly")
            self.port.close()
        return self.outbox.qsize()

    def send_state(self,pos,heading):
        """Send string of the form:
                "x  y   heading time    ID"
        """
        message = "%.5f\t%.5f\t%.5f\t%.5f\t%s\n"%(pos[0], pos[1], heading, time(), self.ID)
        self.outbox.put(message)
        return message

    def send_position(self,pos):
        """Send string of the form:
                "x  y   time    ID"
        """
        message = "%.5f\t%.5f\t%.5f\t%s\n"%(pos[0], pos[1], time(), self.ID)
        self.outbox.put(message)
        return message

    def send_heading(self, heading):
        """Send string of the form:
                "heading    time    ID"
        """
        message = "%.5f\t%.5f\t%s\n"%(heading, time(), self.ID)
        self.outbox.put(message)
        return message

    def get_agents_state(self):
        """Returns the dictionary with the
        data gathered from the network through the
        read_background thread.
        """
        return self.positions

#--- User shouldnt need to call any function below this point

    def send(self):
        """Write the last item put into
        the outbox into the serial port.
        Return the item.
        """
        m = self.outbox.get()
        self.port.write(m)
        self.outbox.task_done()
        return m

    def send_background(self):
        """Function meant to be called in a separate
        thread to continuosly check for the time and
        send the most recent message whenever the
        time slot is right.
        When the proper time slot is reached, this
        sends the last put item in the Queue
        and erase the rest.
        """
        while self.broadcasting:
            t = time()%self.period
            if t < self.window_start:
                sleep(self.window_start - t)
            elif t >= self.window_end:
                sleep( self.period + self.window_start - t)
            else:
                if self.outbox.empty():
                    sleep( (self.window_end-self.window_start)*0.2)
                else:
                    self.send()
                    # clear the queue:
                    while not self.outbox.empty():
                        try:
                            self.outbox.get(False)
                        except Empty:
                            continue
                        self.outbox.task_done()
                    # call again time() as the thread-safe operation
                    # can take a sizeable amount of time.
                    sleep( self.window_end - time()%self.period ) # make sure only one message per window is sent.
        return

    def read(self):
        """If there is an incoming message wait until
        a whole line is receive it, then parse it and
        store the processed data into self.position.
        A certain structure for the message is assumed,
        if the message fails to follow the structure a
        warning is sent to stderr and the message is
        ignored.
        Returns the received message.

        TODO Future implementation: have a first
        code in the message that informs what
        structure it is following out of a set
        a set of given pre-programmed structures.
        """
        message=''
        if self.port.inWaiting() > 0:
            message=self.port.readline()
            try:
                x , y , theta, time , ID = message.rstrip('\n').split()
                self.positions[ID] = (float(x), float(y), float(theta))
            except:
                sys.stderr.write("XBeeNetwork.read(): Bad data received:\n%s\n"%message)
        return message

    def read_background(self):
        """Function meant to be called in a separate
        thread to continuosly check for incoming
        messages. The frequency at which new
        messages are checked could require some
        tweaking depending on the hardware limitations.
        and the number of robots broadcasting.
        """
        while self.broadcasting:
            self.read()
            sleep(self.period/15.) # assuming this is enough to catch all messages
        return

#...............................................................................................
class XBeeExpirationNetwork(XBeeNetwork):
    """Extension of XBeeNetwork where the data
    received is ignored after *expiration_time*
    number of seconds since it was first sent
    (according to the sender).
    """
    def __init__(self, expiration_time, window_start, window_end, period=1, ID=None, lock=None):
        self.expiration_time = expiration_time
        self.expirations = {}
        XBeeNetwork.__init__(self,window_start, window_end, period, ID, lock)
        return

    def read(self):
        message=''
        if self.port.inWaiting() > 0:
            message=self.port.readline()
            try:
                x , y , theta, time , ID = message.rstrip('\n').split()
                self.positions[ID] = (float(x), float(y), float(theta))
                self.expirations[ID] = float(time) + self.expiration_time
            except:
                sys.stderr.write("XBeeNetwork.read(): Bad data: "+message)
        return message

    def get_agents_state(self):
        """Build a new dictionary that only contains the entries from
        self.positions that have not yet reached their expiration time.
        """
        t = time()
        ids = [ID for ID,exp_time in self.expirations.items() if t < exp_time]
        g = { ID:self.positions[ID] for ID in ids}
        return g
