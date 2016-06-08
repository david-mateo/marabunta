from math import *
from BaseRobot import BaseBody
from Map import Map2D

class MockBody(BaseBody):
    """Simulation of a body. Locomotion
    with this body is simply updating
    the values of *pos* and *heading*.
    Sensors simulated through a Map instance
    that contains the obstacles to be detected.
    """
    def __init__(self, pos, heading, max_speed = 0.15, LRdist = 0.1, aperture = 0.7854):
        # State
        self.pos = [ pos[0], pos[1] ]
        self.heading = heading
        # Parameters
        self.max_speed = max_speed
        self.LRdist  = LRdist
        self.aperture = aperture
        return

    def move_forward(self, dt, v = None):
        """Move in current direction for dt time.
        """
        if v==None or v > self.max_speed:
            v = self.max_speed
        self.pos[0] += v * cos(self.heading) * dt
        self.pos[1] += v * sin(self.heading) * dt
        return

    def rotate(self, dtheta):
        """Rotate robot an angle dtheta.
        """
        time = self.LRdist * abs(dtheta) / (2*self.max_speed)
        self.heading += dtheta
        return time

    def move(self, dt, v, omega):
        """Move the robot with linear velocity v
        and angular velocity omega.
        """
        dtheta = omega * dt
        self.rotate(0.5 * dtheta)
        self.move_forward(dt, v)
        self.rotate(0.5 * dtheta)
        return

    def get_position(self):
        """Return current estimate for position.
        """
        return (self.pos[0], self.pos[1])

    def get_heading(self):
        """Return current estimate for heading.
        """
        return self.heading


    def load_obstacles(self, filename):
        """Load the obstacles stored in *filename*
        using a Map2D instance. Using a Map2D will
        automatically store the obstacles in a grid
        for fast access to nearby obstacles.
        """
        self.obstacles = Map2D(filename, 0.5)
        return

    def get_ultrasound(self):
        """Return the distance to all the
        nearby obstacles (as defined by the
        Map2D instance). If no instance is
        stored in self.obstacles, return []
        """
        try:
            obs = self.obstacles.obstacles_near(self.pos)
        except:
            return []
        x , y = self.pos
        return [ sqrt( (o[0]-x)**2 + (o[1]-y)**2 ) for o in obs]

    def obstacle_coordinates(self):
        """Return the relative position of all the
        nearby obstacles (as defined by the
        Map2D instance). If no instance is
        stored in self.obstacles, return []
        """
        try:
            obs = self.obstacles.obstacles_near(self.pos)
        except:
            return []
        x , y = self.pos
        return [ [o[0]-x, o[1]-y] for o in obs if (o[0]-x)**2 + (o[1]-y)**2 < 1.4*1.4]

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        return any( d<0.02 for d in self.get_ultrasound() if d)


    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        return any( d<0.30 for d in self.get_ultrasound() if d)


    def get_wheel_distance(self):
        return self.LRdist

    def get_sensor_aperture(self):
        return self.aperture # = 72o in radians

