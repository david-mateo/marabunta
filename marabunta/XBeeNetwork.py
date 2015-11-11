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
    When receiving a message, its content is assumed to have
    a certain structure defined by the first two characters.
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
        self.poses = {}
        self.inbox = Queue.LifoQueue()
        self.outbox = Queue.LifoQueue()
        self.awake = threading.Event()
        self.awake.set()
        self.parser = { "xx":self.parse_position ,
                        "tt":self.parse_heading ,
                        "oo":self.parse_obstacles ,
                        "xo":self.parse_position_obstacles ,
                        "up":self.parse_wakeup ,
                        "ss":self.parse_sleep ,
                        "mm":self.parse_message
                        }
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
                    for i in range(7):
                        self.port.flushInput()
                        self.port.flushOutput()
                    break
            if not connected:
                self.port = None
                raise Exception("marabunta.XBeeNetwork.start_broadcasting: could not connect to Xbee")
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
            self.awake.set() # force wakeup to finish the threads
            if self.send_thread.is_alive():
                self.send_thread.join(5)
            if self.read_thread.is_alive():
                self.read_thread.join(5)
            if self.send_thread.is_alive() or self.read_thread.is_alive():
                raise Exception("marabunta.XBeeNetwork.stop_broadcasting: Could not stop background threads properly")
            self.port.close()
        return self.outbox.qsize()

    def standby(self):
        """If the robot is asleep, check periodically
        for a wakeup signal (signal starting with "up").
        Ignores any other message received.
        Returns the time spent in standby mode.
        """
        init_time = time()
        while not self.is_awake():
            sleep(2)
            while self.port.inWaiting() > 0:
                new_message=self.port.readline()
                if len(new_message)>1 and new_message[0:2]=="up":
                    self.parse_wakeup("")
        return time() - init_time

    def is_awake(self):
        return self.awake.is_set()

# Sending methods:

    def send_state(self,pos,heading):
        """Send string of the form:
                "xx*x*  *y*   *heading* *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "xx%.5f\t%.5f\t%.5f\t%.5f\t%s\n"%(pos[0], pos[1], heading, time(), self.ID)
        if self.outbox.empty():
            self.outbox.put(message)
        return message

    def send_heading(self, heading):
        """Send string of the form:
                "tt*heading*    *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "tt%.5f\t%.5f\t%s\n"%(heading, time(), self.ID)
        if self.outbox.empty():
            self.outbox.put(message)
        return message

    def send_wakeup(self):
        """Send wakeup signal to everyone.
        Message includes the ID and the time.
        """
        message = "up%.5f\t%s\n"%(time(), self.ID)
        self.outbox.put(message)
        return message

    def send_sleep(self):
        """Send sleep signal to everyone.
        Message includes the ID and the time.
        """
        message = "ss%.5f\t%s\n"%(time(), self.ID)
        self.outbox.put(message)
        return message

    def send_message(self, text):
        """Sends a generic message given
        as input.
        """
        message="mm"+str(text)
        self.outbox.put(message)
        return message

# Processing incoming methods:

    def parse_position(self, message):
        """Parse a message containing x, y, theta, time, ID"""
        try:
            x , y , theta, time , ID = message.rstrip('\n').split()
            self.poses[ID] = (float(x), float(y), float(theta))
        except:
            sys.stderr.write("XBeeNetwork.parse_position(): Bad data received:\n%s\n"%message)
        return

    def parse_heading(self, message):
        """Parse a message containing theta, time, ID"""
        try:
            theta, time , ID = message.rstrip('\n').split()
            self.poses[ID] = float(theta)
        except:
            sys.stderr.write("XBeeNetwork.parse_heading(): Bad data received:\n%s\n"%message)
        return

    def parse_obstacles(self,message):
        """Parse a message containing a set of obstacle
        coordinates. Not implemented yet.
        """
        raise Exception("XBee.parse_obstacles: Not implemented yet")
        return

    def parse_position_obstacles(self,message):
        """Parse a message containing x, y, theta and
        a set of obstacle coordinates.
        Not implemented yet.
        """
        raise Exception("XBee.parse_position_obstacles: Not implemented yet")
        return

    def parse_wakeup(self, message):
        """Wakes up the device."""
        self.awake.set()
        return

    def parse_sleep(self, message):
        """If device is awake, set to sleep and
        put on standby mode. This method returns
        only after the device is awake again.
        """
        if self.is_awake():
            self.awake.clear()
            self.standby()
        return

    def parse_message(self,message):
        self.inbox.put(message)
        return

    def get_incomings(self):
        """Returns all incoming messages received since
        last call to this method. The messages are
        returned in a list sorted from newest to oldest
        (FILO stack).
        """
        incomings = []
        while not self.inbox.empty():
            incomings.append( self.inbox.get() )
            self.inbox.task_done()
        return incomings

    def get_agents_state(self):
        """Returns the dictionary with the
        data gathered from the network through the
        read_background thread.
        """
        return self.poses

#--- User shouldnt need to call any function below this point

    def send(self):
        """Write the most recent item put into
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
        sends the last item put in the Queue.
        and erases the rest.
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
                    # call again time() as the thread-safe operation
                    # can take a sizeable amount of time.
                    sleep( self.window_end - time()%self.period ) # make sure only one message per window is sent.
            self.awake.wait() # wait until the device is awake.
        return

    def read(self):
        """If there is an incoming message wait until
        a whole line is receive, then parse using
        the appropiate parser function according to
        the "key" of the message (first two characters).
        A certain structure for the message is assumed,
        if the message fails to follow the structure a
        warning is sent to stderr and the message is
        ignored.
        New keys should be added to the keys-to-parsers
        dict, self.parser.
        Returns the received message.
        """
        message=''
        if self.port.inWaiting() > 0:
            message=self.port.readline()
            if len(message)>1:
                key = message[0:2]
                try:
                    self.parser[key](message[2:])
                except KeyError:
                    sys.stderr.write("XBeeNetwork.read(): Received unknown key:\t%s\n"%key)
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
            self.awake.wait()
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

    def parse_position(self, message):
        """Parse a message containing x, y, theta, time, ID.
        Store the expiration date of the message in self.expirations."""
        try:
            x , y , theta, time , ID = message.rstrip('\n').split()
            self.poses[ID] = (float(x), float(y), float(theta))
            self.expirations[ID] = float(time) + self.expiration_time
        except:
            sys.stderr.write("XBeeNetwork.parse_pose(): Bad data received:\n%s\n"%message)
        return

    def get_agents_state(self):
        """Build a new dictionary that only contains the entries from
        self.poses that have not yet reached their expiration time.
        """
        t = time()
        ids = [ID for ID,exp_time in self.expirations.items() if t < exp_time]
        g = { ID:self.poses[ID] for ID in ids}
        return g
