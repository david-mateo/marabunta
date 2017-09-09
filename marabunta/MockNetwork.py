from random import randint
from time import time
from BaseRobot import BaseNetwork
import glob
import sys


class MockNetwork(BaseNetwork):
    """Simulate communication between
    agents by using the filesystem.
    All the agents should share a common
    file where they check who is currently
    broadcasting and where.
    Each agent uses its own file to broadcast
    its state.
    """
    basechannel = "radio_{:}.net"

    def __init__(self, ID=None):
        """Start MockNetwork.
        If an ID is not given, just assign a
        random number.
        """
        if ID:
            self.ID = str(ID)
        else:
            self.ID = str(randint(0, 999999))
        self.parser = {"xx": self.parse_position,
                       "tt": self.parse_heading,
                       "oo": self.parse_obstacles,
                       "xo": self.parse_position_obstacles,
                       "mm": self.parse_message}
        self.poses = {}
        self.obstacles = {}
        self.inbox = []
        return

    def start_broadcasting(self):
        """Open a file named *logname* to
        send messages to other agents in
        the network. Write a line containing
        its *ID* and *logname* so that other
        agents know where to look.
        """
        self.logname = self.basechannel.format(self.ID)
        self.log = open(self.logname, 'w', 0)  # 0 buffsize = dont buffer
        return self.log

    def stop_broadcasting(self):
        """Close the broadcasting file and
        erase this agent's signature in the
        common file *global_log*.
        This sends some string to the log to
        simulate the possibility that fractions
        of a message or other kinds of garbage
        data could be sent when the network
        device is turned off abruptly.

        To erase the agent presence in the common
        file, it reads lines one by one and
        re-writes all lines that do not contain
        its own ID. This is NOT thread-safe, and
        may cause strange behavior if other agent
        is accessing the data.
        """
        self.log.write("#End of transmission")
        self.log.close()
        return

    # Sending methods:

    def send_state(self, pos, heading):
        """Send string of the form:
                "xx*x*  *y*   *heading* *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "xx\t{:.5f}\t{:.5f}\t{:.5f}\t{:.5f}\t{:}\n".format(
            pos[0], pos[1], heading, time(), self.ID)
        self.log.write(message)
        return message

    def send_heading(self, heading):
        """Send string of the form:
                "tt*heading*    *time*    *ID*"
        This is a low priority message, it is only scheduled
        to send if there is no other message in the stack.
        """
        message = "tt\t{:.5f}\t{:.5f}\t{:}\n".format(heading, time(), self.ID)
        self.log.write(message)
        return message

    def send_obstacles(self, obstacles):
        """Send string of the form:
            ""oo*x1*:*y1*    *x2*:*y2* (...) *time* *ID*"
        The message can contain an arbitrary number of obstacles
        (but it is not guaranteed to be sent correctly if there
        are too many).
        """
        obstacles_str = "".join("{:.2f}:{:.2f}".format(*o) for o in obstacles)
        message = "oo{:}{:.5f}\t{:}".format(obstacles_str, time(), self.ID)
        self.log.write(message)
        return message

    def send_wakeup(self):
        """Send wakeup signal to everyone.
        Message includes the ID and the time.
        """
        message = "up\t{:.5f}\t{:}\n".format(time(), self.ID)
        self.log.write(message)
        return message

    def send_sleep(self):
        """Send sleep signal to everyone.
        Message includes the ID and the time.
        """
        message = "ss\t{:.5f}\t{:}\n".format(time(), self.ID)
        self.log.write(message)
        return message

    def send_message(self, text):
        """Sends a generic message given
        as input.
        """
        message = "mm" + str(text)
        self.log.write(message)
        return message

    # Processing incoming methods:

    def parse_position(self, message):
        """Parse a message containing x, y, theta, time, ID"""
        try:
            x, y, theta, time, ID = message.rstrip('\n').split()
            self.poses[ID] = (float(x), float(y), float(theta))
        except:
            sys.stderr.write("parse_position(): Bad data:\n" + message + "\n")
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
            self.obstacles[ID] = [[float(x) for x in point.split(':')]
                                  for point in data]
        except:
            sys.stderr.write("parse_obstacles(): Bad data:\n" + message + "\n")
        return

    def parse_position_obstacles(self, message):
        """Parse a message containing x, y, theta and
        a set of obstacle coordinates.
        Not implemented yet.
        """
        raise Exception("parse_position_obstacles: Not implemented")
        return

    def parse_message(self, message):
        self.inbox.append(message)
        return

    def read_all(self):
        """Read the last 5 lines broadcasted by
        each agent and parse the contents.
        """
        logfiles = glob.glob(self.basechannel.format("*"))
        self.poses = {}
        self.obstacles = {}
        for logfile in logfiles:
            if logfile != self.logname:
                with open(logfile, 'r') as f:
                    lines = f.readlines()
                    if len(lines) > 5:
                        lines = lines[-5:]
                    for line in lines:
                        try:
                            key = line[0:2]
                            message = line[2:]
                        except:
                            key = None
                        if key in self.parser:
                            self.parser[key](message)
        return

    def get_agents_state(self):
        """Gathers all the agents' state.
        Returns a dictionary of the form:
          { ID: [x, y, heading] }
        """
        self.read_all()
        return self.poses

    def get_obstacles(self):
        """Gathers all the agents' detected obtacles.
        Returns a dictionary of the form:
          { ID: [ [x1, y1], [x2, y2], [x3, y3], ...] }
        """
        self.read_all()
        return self.obstacles

    def get_messages(self):
        incomings = list(reversed(self.inbox))
        self.inbox = []
        return incomings
