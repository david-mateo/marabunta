from random import randint
from time import time
from BaseRobot import BaseNetwork

class MockNetwork(BaseNetwork):
    """Simulate communication between
    agents by using the filesystem.
    All the agents should share a common
    file where they check who is currently
    broadcasting and where.
    Each agent uses its own file to broadcast
    its state.
    """
    def __init__(self, global_log, ID=None):
        """Start MockNetwork. Requires a
        file-like object *global_log* where
        the agent can write where is it
        currently broadcasting. Note that this
        requires afile-like instance,
        NOT a filename.
        If an ID is not given, just assign a
        random number.
        """
        self.global_log = global_log
        if ID:
            self.ID = str(ID)
        else:
            self.ID = str(randint(0,999999))
        return

    def start_broadcasting(self):
        """Open a file named *logname* to
        send messages to other agents in
        the network. Write a line containing
        its *ID* and *logname* so that other
        agents know where to look.
        """
        logname = 'radio_'+self.ID+'.net'
        self.log = open(logname, 'w', 0) # 0 buffsize = dont buffer results
        self.global_log.write('Robot ID and broadcast: %s\t%s\n'%( self.ID , logname ))
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
        self.log.write("#This simulates garbage data")
        self.log.close()
        self.global_log.seek(0)
        lines = self.global_log.readlines()
        self.global_log.seek(0)
        for line in lines:
            if line.split()[4]!=self.ID:
                self.global_log.write(line)
        self.global_log.truncate()
        return

    def send_state(self,pos,heading):
        """Send string of the form:
                "x  y   heading time"
        """
        self.log.write( '%s\t%s\t%s\t%s\n'%(pos[0],pos[1],heading,time()) )
        return

    def send_position(self,pos):
        """Send string of the form:
                "x  y   time"
        """
        self.log.write( '%s\t%s\t%s\n'%(pos[0],pos[1],time()) )
        return

    def get_agents_state(self):
        """Gathers all the agent's state.
        Reads the common file to check
        which agents are broadcasting, then
        read the latest message sent by
        each one in their own channels.
        Returns a dictionary with the form:
            ID: [x, y, heading]
        """
        self.global_log.seek(0)
        lines = self.global_log.readlines()
        robots = [ line.split()[4:6] for line in lines]
        positions = {}
        for ID,logfile in robots:
            if ID is not self.ID:
                with open(logfile,'r') as f:
                    lines = f.readlines()
                    if lines:
                        pos = [float(x) for x in lines[-1].split()[:3]]
                        positions[ID] = pos
        return positions
