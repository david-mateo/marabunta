from marabunta import BaseRobot, MockBody, MockNetwork
import random

# for visualization
import numpy as np
import pylab as pl

class myRobot(BaseRobot):
    def __init__(self, setting):
        body = MockBody(setting.get("position") ,setting.get("heading"))
        network = MockNetwork(setting.get("ID"))
        BaseRobot.__init__(self,body, network)
        return

    def spread_target(self):
        neis = self.get_agents().values()
        pos = self.body.get_position()
        # Get both neighbors and obstacles, in relative coordinates
        obstacles = self.body.obstacle_coordinates()
        points = [ [nei[0]-pos[0], nei[1]-pos[1]] for nei in neis] + obstacles
        if points:
            target = [0.,0.]
            for p in points:
                d2 = p[0]**2 + p[1]**2
                weight = (1.0/d2 )**1.5
                if d2>0:
                    target[0] -= p[0]*weight
                    target[1] -= p[1]*weight
        else:
            target= None
        return target

    def move_towards_target(self, target, deltat, v):
        threshold = 5
        if target[0]**2 + target[1]**2 > threshold*threshold:
            self.align(target)
            self.move_forward(deltat, v)
        else:
            self.move_forward(deltat, 0)
        return

    def update(self, deltat, v):
        self.broadcast_state()
        # Perform swarming
        target = self.spread_target()
        if not target:
            h= self.body.get_heading()
            target = [10.*cos(h) ,10.*sin(h)]
        # Avoid obstacles
        target = self.correct_target(target)
        self.move_towards_target(target, deltat, v)
        return


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

robots = [ myRobot(s) for s in settings]

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
