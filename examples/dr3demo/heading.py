from HeadingConsensusRobot import HeadingConsensusRobot
from ebotBody import ebotBody
from XBeeNetwork import *
from math import *
from time import time,sleep
import sys
from threading import Lock
from settings import s

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
robot = HeadingConsensusRobot( body, network )
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
end_time = time() + total_time
while time() < end_time:
    robot.update(dt, speed)
    #print robot.get_agents()
    pos = robot.body.get_position()
    print pos[0] , pos[1] , robot.body.get_heading()
    sleep(dt)

robot.turn_off()
