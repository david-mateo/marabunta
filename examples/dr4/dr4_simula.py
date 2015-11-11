from marabunta import MockBody, MockNetwork
from marabunta.models import PerimeterDefenseRobot
from math import *
from time import time,sleep
import sys
import threading
#from settings import s

dt=0.2
total_time = 5 * 60
speed=0.15

log = open("the_cloud.dat","w+")
settings = []
settings.append( {"ID":1 , "position":[  0.0 ,  0.0 ] , "heading":0.} )
settings.append( {"ID":2 , "position":[  0.5 ,  0.0 ] , "heading":0.} )
settings.append( {"ID":3 , "position":[  0.0 ,  0.5 ] , "heading":0.} )
settings.append( {"ID":4 , "position":[ -0.5 ,  0.0 ] , "heading":0.} )
settings.append( {"ID":5 , "position":[  0.0 , -0.5 ] , "heading":0.} )
settings.append( {"ID":6 , "position":[  0.5 ,  0.5 ] , "heading":0.} )
settings.append( {"ID":7 , "position":[  0.5 , -0.5 ] , "heading":0.} )
settings.append( {"ID":8 , "position":[ -0.5 , -0.5 ] , "heading":0.} )
settings.append( {"ID":9 , "position":[ -0.5 ,  0.5 ] , "heading":0.} )

robots=[ PerimeterDefenseRobot( MockBody(s.get("position") ,s.get("heading")), MockNetwork(log, s.get("ID")) , 1.e-8) for s in settings]

[robot.body.load_obstacles("map_data.dat") for robot in robots]
[robot.turn_on() for robot in robots]

try:
    [robot.broadcast_state() for robot in robots]

    #robot.start_printing(0.5)

    # MAIN LOOP
    init_time = time()
    end_time = init_time + total_time
    for it in range(int(total_time/dt)):

        # Check for a message ordering to go somewhere or to stop
        messages = [] # network.get_incomings()
        for message in messages:
            if len(message)>3 and message[:4]=="stop":
                raise Exception("Stop!")
            if len(message)>3 and message[:4]=="goto":
                try:
                    gotoxy = message.split()
                    point = (float(gotoxy[1]), float(gotoxy[2]))
                except:
                    print "Weird message received: ",message
                    point = None
                if point:
                    robot.network.send_message(message) # relay message
                    robot.go_to(point)
                    print "#Task completed at time %f"%(time()-init_time)
                    raise Exception("Task completed")
                    break
        lights = [robot.update(dt, speed) for robot in robots]
        if any(lights):
            print "#Light detected at time %f"%(time()-init_time)
            #robot.move_forward( 0., 0. )
            #sleep(4)
            raise Exception("I see the light at the end of the tunnel")
        #sleep(dt)
finally:
    [robot.turn_off() for robot in robots]
print "#Finish"
#robot.turn_off()
