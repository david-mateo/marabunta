from marabunta import MockBody, MockNetwork
from marabunta.models import AreaCoverageRobot
import random

# for visualization
import numpy as np
import pylab as pl


class myRobot(AreaCoverageRobot):
    pass

num_robots = 50
dt=0.5
total_time = 100
speed = 0.4
map_file = "map_points.dat"
obstacles = np.loadtxt(map_file)


#--------------------------------------------------------
def random_position(x0, y0, R0):
    """Return a random vector uniformly distributed
    in a circle of radius R0 centered on (x0,y0).
    """
    r2 = float('inf')
    while r2 > 1.0:
        x , y = 2.0*(random.random()-0.5) , 2.0*(random.random()-0.5)
        r2 = x*x + y*y
    x = x0 + x*R0
    y = y0 + y*R0
    return [x,y]

def plot(robots, obstacles):
    """Plots the current configuration of robots."""
    xs = [robot.body.get_position()[0] for robot in robots]
    ys = [robot.body.get_position()[1] for robot in robots]
    pl.plot(obstacles[:,0],obstacles[:,1],'gs',markersize=20.)
    pl.plot(xs,ys,'ro')
    pl.show(block=False)
    pl.pause(0.0000001)
    pl.close()
    return
#-----------------------------------------------------

settings = [ {"ID":"R%02i"%i, "position":random_position(1.5,0.5,0.2), "heading":0.} for i in range(num_robots)]
robots=[ myRobot( MockBody(s.get("position") ,s.get("heading")), MockNetwork(s.get("ID")) , 5.) for s in settings]

[robot.body.load_obstacles(map_file) for robot in robots]
[robot.turn_on() for robot in robots]

try:
    [robot.broadcast_state() for robot in robots]

    # MAIN LOOP
    for it in range(int(total_time/dt)):
        for robot in robots:
            if robot.is_working():
                robot.update(dt, speed)
        plot(robots, obstacles)
finally:
    [robot.turn_off() for robot in robots]
