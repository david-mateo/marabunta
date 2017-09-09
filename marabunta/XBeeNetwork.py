from random import randint
from time import time, sleep
import threading
import Queue
import glob
import sys
from BaseRobot import BaseNetwork
from utils import SafeSerial
from serial import Serial


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
    def __init__(self, window_start, window_end, period,
                 ID=None, lock=None, tty='/dev/ttyUSB*'):
        assert period > 0.
        assert window_start >= 0. and window_start < period
        assert window_end > window_start and window_end <= period
        self.window_start = window_start
        self.window_end = window_end
        self.period = period
        if ID:
            self.ID = str(ID)
        else:
            self.ID = str(randint(0, 999999))
        self.lock = lock
        self.tty = tty
        self.broadcasting = False
        self.port = None
        self.poses = {}
        self.obstacles = {}
        self.obstimes = {}
        self.inbox = Queue.LifoQueue()
        self.outbox = Queue.LifoQueue()
        self.awake = threading.Event()
        self.awake.set()
        self.parser = {"xx": self.parse_state,
                       "tt": self.parse_heading,
                       "oo": self.parse_obstacles,
                       "xo": self.parse_state_obstacles,
                       "up": self.parse_wakeup,
                       "ss": self.parse_sleep,
                       "mm": self.parse_message}
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
            connected = False
            if '*' in self.tty:
                port_paths = glob.glob(self.tty)
            else:
                port_paths = [self.tty]
            for port_path in port_paths:
                try:
                    if self.lock == 'no':
                        port = Serial(port_path, baudrate=115200,
                                      timeout=0.100, writeTimeout=0.100)
                    else:
                        port = SafeSerial(port_path, baudrate=115200,
                                          timeout=0.100, writeTimeout=0.100,
                                          lock=self.lock)
                    connected = True
                except:
                    connected = False
                if connected:
                    self.port = port
                    for i in range(7):
                        self.port.flushInput()
                        self.port.flushOutput()
                    break
            if not connected:
                self.port = None
                raise Exception("start_broadcasting: Xbee not found")
            self.broadcasting = True
            self.send_thread = threading.Thread(target=self.send_background)
            self.send_thread.daemon = True
            self.send_thread.start()
            self.read_thread = threading.Thread(target=self.read_background)
            self.read_thread.daemon = True
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
            self.awake.set()  # force wakeup to finish the threads
            if self.send_thread.is_alive():
                self.send_thread.join(5)
            if self.read_thread.is_alive():
                self.read_thread.join(5)
            if self.send_thread.is_alive() or self.read_thread.is_alive():
                raise Exception(
                    "stop_broadcasting: Could not stop background threads")
            self.port.close()
        return self.outbox.qsize()

    def __enter__(self):
        self.start_broadcasting()
        return self

    def __exit__(self, type, value, traceback):
        self.stop_broadcasting()
        return

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
                new_message = self.port.readline()
                if len(new_message) > 1 and new_message[0:2] == "up":
                    self.parse_wakeup("")
        return time() - init_time

    def is_awake(self):
        return self.awake.is_set()

# Sending methods:

    def send_state(self, pos, heading):
        """Send string of the form:
                "xx*x*  *y*   *heading* *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "xx{:.5f}\t{:.5f}\t{:.5f}\t{:.5f}\t{:}\n".format(
            pos[0], pos[1], heading, time(), self.ID)
        if self.outbox.empty():
            self.outbox.put(message)
        return message

    def send_heading(self, heading):
        """Send string of the form:
                "tt*heading*    *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "tt{:.5f}\t{:.5f}\t%{:}\n".format(heading, time(), self.ID)
        if self.outbox.empty():
            self.outbox.put(message)
        return message

    def send_obstacles(self, obstacles):
        """Send string of the form:
                "oo*x1*:*y1*    *x2*:*y2* (...) *time* *ID*"
        The message can contain an arbitrary number of obstacles
        (but it is not guaranteed to be sent correctly if there
        are too many).
        """
        obstacles_str = "".join("{:.2f}:{:.2f}".format(*o) for o in obstacles)
        message = "oo{:}{:.5f}\t{:}".format(obstacles_str, time(), self.ID)
        self.outbox.put(message)
        return message

    def send_state_obstacles(self, pos, heading, obstacles):
        """Send string of the form:
            "xo*x*  *y*   *heading* *x1*:*y1*    *x2*:*y2* (...) *time* *ID*"
        The message can contain an arbitrary number of obstacles
        (but it is not guaranteed to be sent correctly if there
        are too many).
        """
        obstacles_str = "".join("{:.2f}:{:.2f}".format(*o) for o in obstacles)
        message = "xo{:.5f}\t{:.5f}\t{:.5f}\t{:}\t{:.5f}\t{:}\n".format(
            pos[0], pos[1], heading, obstacles_str, time(), self.ID)
        if self.outbox.empty():
            self.outbox.put(message)
        return message

    def send_wakeup(self):
        """Send wakeup signal to everyone.
        Message includes the ID and the time.
        """
        message = "up{:.5f}\t{:}\n".format(time(), self.ID)
        self.outbox.put(message)
        return message

    def send_sleep(self):
        """Send sleep signal to everyone.
        Message includes the ID and the time.
        """
        message = "ss{:.5f}\t{:}\n".format(time(), self.ID)
        self.outbox.put(message)
        return message

    def send_message(self, text):
        """Sends a generic message given
        as input.
        """
        message = "mm" + str(text)
        self.outbox.put(message)
        return message

# Processing incoming methods:

    def parse_state(self, message):
        """Parse a message containing x, y, theta, time, ID"""
        try:
            x, y, theta, time, ID = message.rstrip('\n').split()
            self.poses[ID] = (float(x), float(y), float(theta))
        except:
            sys.stderr.write("parse_state(): Bad data:\n" + message + "\n")
        return

    def parse_heading(self, message):
        """Parse a message containing theta, time, ID"""
        try:
            theta, time, ID = message.rstrip('\n').split()
            self.poses[ID] = float(theta)
        except:
            sys.stderr.write("parse_heading(): Bad data:\n" + message + "\n")
        return

    def parse_obstacles(self, message):
        """Parse a message containing a set of obstacle
        coordinates. Not implemented yet.
        """
        try:
            data = message.rstrip('\n').split()
            ID = data.pop()
            time = float(data.pop())
            self.obstacles[ID] = [[float(p) for p in point.split(':')]
                                  for point in data]
            self.obstimes[ID] = time
        except:
            sys.stderr.write("parse_obstacles(): Bad data:\n" + message + "\n")
        return

    def parse_state_obstacles(self, message):
        """Parse a message containing x, y, theta and
        a set of obstacle coordinates.
        Not implemented yet.
        """
        try:
            data = message.rstrip('\n').split()
            ID = data.pop()
            time = float(data.pop())
            x, y, theta = data[:3]
            self.poses[ID] = (float(x), float(y), float(theta))
            self.obstacles[ID] = [[float(p) for p in point.split(':')]
                                  for point in data]
            self.obstimes[ID] = time
        except:
            sys.stderr.write(
                "parse_state_obstacles(): Bad data:\n" + message + "\n")
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

    def parse_message(self, message):
        self.inbox.put(message)
        return

    def get_messages(self):
        """Returns all incoming messages received since
        last call to this method. The messages are
        returned in a list sorted from newest to oldest
        (FILO stack).
        """
        incomings = []
        while not self.inbox.empty():
            incomings.append(self.inbox.get())
            self.inbox.task_done()
        return incomings

    def get_agents_state(self):
        """Returns the dictionary with the
        data gathered from the network through the
        read_background thread regarding the
        state (position and heading) of the agents.
        """
        return self.poses

    def get_obstacles(self):
        """Returns the dictionary with the
        data gathered from the network through the
        read_background thread regarding the
        obstacles detected by other agents.
        """
        return self.obstacles, self.obstimes


# User should not need to call any function below this point

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
            t = time() % self.period
            if t < self.window_start:
                sleep(self.window_start - t)
            elif t >= self.window_end:
                sleep(self.period + self.window_start - t)
            else:
                if self.outbox.empty():
                    sleep((self.window_end - self.window_start) * 0.2)
                else:
                    self.send()
                    # make sure only one message per window is sent:
                    sleep(self.window_end - time() % self.period)
            self.awake.wait()  # wait until the device is awake.
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
        message = ''
        if self.port.inWaiting() > 0:
            message = self.port.readline()
            if len(message) > 1:
                key = message[0:2]
                try:
                    self.parser[key](message[2:])
                except KeyError:
                    sys.stderr.write("read(): unknown key:\n" + key + "\n")
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
            sleep(self.period / 15.)  # assuming is enough to get all messages
            self.awake.wait()
        return


class XBeeExpirationNetwork(XBeeNetwork):
    """Extension of XBeeNetwork where the data
    received is ignored after *expiration_time*
    number of seconds since it was first sent
    (according to the sender).
    """
    def __init__(self, expiration_time, window_start, window_end,
                 period=1, ID=None, lock=None):
        self.expiration_time = expiration_time
        self.expirations = {}
        XBeeNetwork.__init__(self, window_start, window_end, period, ID, lock)
        return

    def parse_state(self, message):
        """Parse a message containing x, y, theta, time, ID.
        Store the expiration date of the message in self.expirations."""
        try:
            x, y, theta, time, ID = message.rstrip('\n').split()
            self.poses[ID] = (float(x), float(y), float(theta))
            self.expirations[ID] = float(time) + self.expiration_time
        except:
            sys.stderr.write("parse_state(): Bad data:\n" + message + "\n")
        return

    def get_agents_state(self):
        """Build a new dictionary that only contains the entries from
        self.poses that have not yet reached their expiration time.
        """
        t = time()
        ids = [ID for ID, exp_time in self.expirations.items() if t < exp_time]
        g = {ID: self.poses[ID] for ID in ids}
        return g
