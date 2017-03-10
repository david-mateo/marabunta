from math import  *
from time import sleep, time
from BaseRobot import BaseBody
from ePuck import ePuck

class ePuckBody(BaseBody,ePuck):
    """Example of a minimal Body implementation
    for the ePuck using its Python API, see
        https://github.com/mmartinortiz/pyePuck .
    This is meant to illustrate how to implement a
    Body class for commercial robots.
    This implementation is UNTESTED.
    """

    def __init__(self, mac):
        ePuck.__init__(self, mac)
        self.max_speed = 0.18
        self.last_update = -float('inf')
        self.turn_on()
        return

    def update_sensors(self):
        """Connect to the ePuck to get the
        sensors. Avoids calling step()
        too often.
        """
        dtime = time() - self.last_update
        if dtime > 0.25:
            self.step()
            self.last_update = time()
        return dtime

    def turn_on(self):
        self.connect() # from pyePuck
        self.enable('proximity','light')
        return

    def get_position(self):
        """Returns the x,y coordinates of the robot.
        """
        # missing localization
        return [0., 0.]

    def get_heading(self):
        """Returns the angle between the robot orientation
        and the x-axis in radians between [-pi/2,pi/2].
        """
        # missing localization
        return 0.

    def wheel_speed(self, v):
        """Convert velocity v from m/s
        to the parameter between -1000 and 1000
        that set_motors_speed expects.
        """
        wheel = int( 1000. * v / self.max_speed)
        wheel = max(-1000, wheel)
        wheel = min( wheel, 1000)
        return wheel


    def move_forward(self,time,v=None):
        """Move forward the robot during *time* seconds at
        a speed *v* (m/s). If no speed is given, use a
        the max speed.
        """
        if v is None:
            v = self.max_speed
        wheel = self.wheel_speed(v)
        self.set_motors_speed(wheel, wheel)
        sleep(time)
        return

    def rotate(self,dtheta):
        """Rotate the robot an angle *dtheta* in radians between [-pi/2,pi/2].
        Return the time in seconds it took to perform the rotation.
        """
        # naive rotate, don't expect to be very precise
        time = self.LRdist * abs(dtheta) / (2*self.max_speed)
        if dtheta>0:
            self.set_motor_speed(-1000, 1000)
        else:
            self.set_motor_speed(1000, -1000)
        sleep(time)
        self.set_motors_speed(0,0)
        return time

    def get_ultrasound(self):
        """Return an array with the distances of the obstacles
        detected. Right now, it is assumed this will return at least 3
        values (this may change).
        """
        self.update_sensors()
        return self.get_proximity()

    def obstacle_coordinates(self):
        """Return a list with the coordinates of the obstacles detected
        in relation to the robot. This coordinates are relative to the
        robot but using the global orientation of the axis, i.e. this
        returns the displacement vector from the robot to the obstacles
        using global coordinates. (in other words, sets the [0.,0.]
        at the position of the robot but ignores its heading.)
        """
        self.update_sensors()
        h = self.get_heading()
        ps = self.get_proximity()
        p0 = [ ps[0]*cos(h + 0*pi/4.), ps[0]*sin(h + 0*pi/4.) ]
        p1 = [ ps[1]*cos(h + 1*pi/4.), ps[1]*sin(h + 1*pi/4.) ]
        p2 = [ ps[2]*cos(h + 2*pi/4.), ps[2]*sin(h + 2*pi/4.) ]
        p3 = [ ps[3]*cos(h + 3*pi/4.), ps[3]*sin(h + 3*pi/4.) ]
        p4 = [ ps[4]*cos(h + 4*pi/4.), ps[4]*sin(h + 4*pi/4.) ]
        p5 = [ ps[5]*cos(h + 5*pi/4.), ps[5]*sin(h + 5*pi/4.) ]
        p6 = [ ps[6]*cos(h + 6*pi/4.), ps[6]*sin(h + 6*pi/4.) ]
        p7 = [ ps[7]*cos(h + 7*pi/4.), ps[7]*sin(h + 7*pi/4.) ]
        return [p0, p1, p2, p3, p4, p5, p6, p7]

    def obstacle_global_coordinates(self):
        """Same as obstacle_coordinates but setting
        the origin of coordinates to the ground truth
        origin and not relative to the robot position.
        """
        pos = self.get_position()
        return [ (ob[0]+pos[0], ob[1]+pos[1]) for ob in self.obstacle_coordinates()]

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        ps = self.get_proximity()
        return ps[0] < 0.3 or ps[7] < 0.3 or ps[1] < 0.15 or ps[6] < 0.15

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        ps = self.get_proximity()
        return ps[0] < 0.6 or ps[7] < 0.6 or \
               ps[1] < 0.5 or ps[6] < 0.5 or \
               ps[2] < 0.2 or ps[5] < 0.2
