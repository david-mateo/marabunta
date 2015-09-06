from BaseRobot import BaseRobot
from ebotBody import ebotBody
from XBeeNetwork import *
from math import *
from time import time,sleep
import sys
from threading import Lock
from settings import s

class Leader(BaseRobot):
    def update(self, deltat, v=None):
        self.broadcast_state()
        if self.obstacle_infront():
            v = 0.0
        self.move_forward(deltat, v)
        return

dt=0.05
total_time = 50
speed=0.15

try:
    num_friends = float(sys.argv[1])-1
except:
    num_friends = 4-1

# These parameters have to be manually
# set for each robot.
init_pos = s["position"]
init_heading = s["heading"]
ID=s["ID"]
slot = s["slot"]


mylock = Lock()
body = ebotBody( init_pos , init_heading, mylock)
network = XBeeExpirationNetwork( 1.4, slot, slot+0.1, 1, ID , mylock)
robot = Leader( body, network )
robot.turn_on()
robot.broadcast_state()

friends = len(robot.get_agents())
patience = 50

while friends < num_friends and patience>0:
    patience -= 1
    print "# Only %i friends detected so far"%friends
    print "#", "\t".join(robot.get_agents().keys())
    friends = len(robot.get_agents())
    robot.broadcast_state()
    sleep(0.2)

# MAIN LOOP
rotate = 3
start_time = time()
end_time = time() + total_time
while time() < end_time:
    if rotate==3 and time()>start_time + 5.:
        rotate -=1
        robot.align( [0, -1])
    if rotate==2 and time()>start_time + 20:
        rotate -=1
        robot.align([1,0])
    if rotate==1 and time()>start_time + 30:
        rotate -=1
        robot.align([-1,0])
    robot.update(dt, speed)
    pos = robot.body.get_position()
    print pos[0] , pos[1] , robot.body.get_heading()
    sleep(dt)

robot.turn_off()

