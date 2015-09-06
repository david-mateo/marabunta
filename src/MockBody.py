from math import sin,cos,pi
from BaseRobot import BaseBody

class MockBody(BaseBody):
    """Simulation of a body. Locomotion
    with this body is simply updating
    the values of *pos* and *heading*.
    No sensors simulated, the obstacle
    detection returns nothing.
    """
    def __init__(self, pos, heading, max_speed = 0.15, LRdist = 0.1, aperture = 0.7854):
        # State
        self.pos = [ pos[0], pos[1] ]
        self.heading = heading
        # Parameters
        self.max_speed = max_speed
        self.LRdist  = LRdist
        self.aperture = aperture
        # Sensors
        default = None
        self.ultrasL = default
        self.ultrasC = default
        self.ultrasR = default
        self.gyro = default
        return

    def move_forward(self, dt, v = None):
        """Move in current direction for dt time.
        """
        if not v:
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

    def get_ultrasound(self):
        """No sensor simulation right now.
        Returns an array with infinities.
        """
        return [float("inf"), float("inf"), float("inf"), float("inf"), float("inf")]

    def obstacle_coordinates(self):
        """No sensor simulation right now.
        Returns None.
        """
        return None

    def get_wheel_distance(self):
        return self.LRdist

    def get_sensor_aperture(self):
        return self.aperture # = 72o in radians

