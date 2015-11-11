from marabunta import *
from marabunta.models import PerimeterDefenseRobot
from math import *
from time import time,sleep
import sys
import threading
from settings import s


class CustomRobot(PerimeterDefenseRobot):
    def update(self, deltat, v=None):
        """Same as PerimeterDefenseRobot except
        that if there's an obstacle the speed
        is capped at 0.08 instead of /2.
        """
        self.broadcast_state()
        # Perform swarming
        target = self.spread_target()
        if not target:
            h= self.body.get_heading()
            target = [1.5*sqrt(self.threshold)*cos(h) ,1.5*sqrt(self.threshold)*sin(h)]
        # Avoid obstacles
        target = self.correct_target_rotating(target)
        obstacle = self.obstacle_near()
        if obstacle:
            v = min(v,0.08)
        self.move_to_target(target, deltat, v, obstacle)
        light = self.light_detected()
        return light

dt=0.1
total_time = 4 * 60
speed=0.15

# These parameters have to be manually
# set for each robot.
init_pos = s["position"]
init_heading = s["heading"]
ID=s["ID"]
slot = s["slot"]
calibration = s.get("calibration")

mylock = threading.Lock() # required for devices with virtual ports like Raspberry.
body = ebotBody( init_pos , init_heading, mylock)
network = XBeeNetwork( slot, slot+0.1, 1.0, ID , mylock)

with CustomRobot( body, network, 1.e-8 ) as robot:
    if calibration:
        LS , RS = calibration
        body.calibration(LS, RS)

    robot.broadcast_state()

    # Wait for XBee signal
    while True:
        messages = network.get_incomings()
        if any( m[:4]=="stop" for m in messages if len(m)>1):
            raise Exception("Stop!")
        if any( m[:2]=="up" for m in messages if len(m)>1):
            robot.broadcast_state()
            sleep(1)
            break
        sleep(1)

    robot.start_printing(0.5)

    # MAIN LOOP
    init_time = time()
    end_time = init_time + total_time
    while time() < end_time:
        # Slow initial warmup
        if time() < init_time + 5:
            speed = 0.01 * (time() - init_time)
        else:
            speed = 0.15

        # Check for a message ordering to go somewhere or to stop
        messages = network.get_incomings()
        for message in messages:
            if len(message)>3 and message[:4]=="stop":
                raise Exception("Stop!")
            if len(message)>3 and message[:4]=="goto":
                try:
                    gotoxy = message.split()
                    point = (float(gotoxy[1]), float(gotoxy[2]))
                except:
                    print("Weird message received: ",message)
                    point = None
                if point:
                    robot.network.send_message(message) # relay message
                    robot.go_to(point)
                    print("#Task completed at time %f"%(time()-init_time))
                    raise Exception("Task completed")
                    break

        light = robot.update(dt, speed)
        if light:
            print("#Light detected at time %f"%(time()-init_time))
            robot.move_forward( 0., 0. )
            sleep(4)
            raise Exception("I see the light at the end of the tunnel")
        sleep(dt)

print("#Finish")
