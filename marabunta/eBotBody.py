from math import atan2, sin,cos,pi,sqrt
from eBot import eBot
from BaseRobot import BaseBody
import threading
from time import time,sleep
import sys

class eBotBody(BaseBody,eBot.eBot):
    """Body class for controlling an eBot
    (see https://github.com/EdgeBotix/docs )
    by using its python API.
    ( TODO link to stable API)
    A background movement is implemented where
    a separate threads continuously check and
    moves the robot at a linear speed set by
    *self.target_speed* while correcting the
    robot heading to align it with *self.target_heading*.
    The background movement is started when the
    body is turned on, but it can be stopped at any
    point and the class is still completely usable
    without this background thread.

    This class assumes the eBot-API provides
    a position() method that returns (x,y,theta)
    with x,y in meters and theta in degrees
    between 0 and 360, and a robot_uS() method
    that returns the readings from the
    ultrasound sensors.
    """
    def __init__(self, pos, heading, lock=None, max_speed = 0.15, LRdist = 0.10, aperture = 0.7854):
        # State (in robot coordinates)
        self.pos = [ pos[0], pos[1] ]
        self.heading = heading
        # Parameters
        self.max_speed = max_speed
        self.LRdist  = LRdist
        self.aperture = aperture

        self.awake = threading.Event()
        self.aligned = threading.Event()
        self.awake.set()
        self.moving_background = False
        self.target_heading = None
        self.target_speed = None
        eBot.eBot.__init__(self, pos, heading, lock)
        return

    def start_move_background(self):
        """Start a background thread controlling
        the speed and the heading of the robot.
        If the thread is alive (self.moving_background=True)
        several other functions rely on this
        instead of implementing the movement
        themselves.
        """
        if not self.moving_background:
            self.moving_background = True
            self.thread = threading.Thread(target = self.move_background)
            self.thread.start()
        return

    def stop_move_background(self):
        """Stop background movement.
        The function returns when the thread
        has been succesfully finished.
        Else an Exception is raised.
        """
        if self.moving_background:
            self.moving_background = False
            self.awake.set() # force wakeup to finish the thread
            self.thread.join(10)
            if self.thread.is_alive():
                raise Exception("marabunta.eBotBody.stop_move_background: Could not stop move_background thread properly")
        return

    def turn_on(self):
        """Connect to the eBot using the
        API method and start the background movement.
        """
        self.connect() # from eBot.eBot
        self.target_heading = self.get_heading()
        self.aligned.set()
        self.target_speed = 0.
        self.start_move_background()
        return

    def turn_off(self):
        """Stop the background movement and
        disconnect using the API method.
        """
        self.stop_move_background()
        self.halt() # from eBot.eBot
        self.disconnect() # from eBot.eBot
        return

    def wakeup(self):
        if not self.awake.is_set():
            self.awake.set()
        return

    def sleep(self):
        self.awake.clear()
        self.halt()
        return

#### Movement ####

    def wheel_speed(self, v):
        """Convert velocity v from m/s
        to the parameter between -1 and 1
        that the wheels method from the
        API expects.
        """
        wheel = v / self.max_speed
        wheel = max(-1., wheel)
        wheel = min(wheel, 1.)
        return wheel

    def move_forward(self, dt, v = None):
        """Move in current direction for *dt* time
        at a speed of *v* if given, or *max_speed*
        if not.
        If the background move is activated,
        *dt* is ignored and this returns inmediately.
        """
        if self.moving_background:
            if v != None:
                self.target_speed = v
        else:
            if v==None:
                v = self.max_speed
            wheel = self.wheel_speed(v)
            self.wheels(wheel, wheel)
            sleep(dt)
            self.wheels(0,0)
        return

    def rotate(self, dtheta):
        """Rotate robot an angle dtheta
        If the background move is activated,
        this returns inmediately.
        """
        if self.moving_background:
            self.target_heading = self.get_heading() + dtheta
            time = 0.
        else:
            time = self.LRdist * abs(dtheta) / (2*self.max_speed)
            if dtheta>0:
                self.wheels(-1,  1)
            else:
                self.wheels( 1, -1)
            sleep(time)
            self.wheels(0,0)
        return time

    def align(self, direction, block=False):
        """Align the heading of the robot to
        a given vector *direction*.
        Use self_target_heading if the background
        move is active, if not use rotate() instead.
        If block is set to true, wait until the robot
        is aligned before returning.
        """
        if self.moving_background:
            self.target_heading = atan2( direction[1], direction[0])
            if block:
                self.aligned.wait()
        else:
            dtheta = atan2( direction[1], direction[0]) - self.get_heading()
            if dtheta > pi:
                dtheta -= 2*pi
            elif dtheta < -pi:
                dtheta += 2*pi
            self.rotate(dtheta)
        return

    def move(self, dt, v, omega, stop_flag = True):
        """Move the robot with linear velocity v
        and angular velocity omega. In the case
        that max wheel speed doesnt allow to fulfill
        both requirements, the angular velocity omega
        is given priority over the linear velocity,
        i.e. if |v|+|omega| > max_speed, then *v* is
        reduced so that |v|+|omega| = max_speed.

        The stop_flag (default True) controls whether
        the robot should stop after *dt* or keep
        the current speed.
        """
        vrot = 0.5 * omega * self.LRdist
        vrot = max(-self.max_speed , min( self.max_speed, vrot) )
        if abs(v)>0 and abs(v)+abs(vrot) > self.max_speed:
            v = (v/abs(v)) * ( self.max_speed - abs(vrot) )

        left_wheel  = self.wheel_speed( v - vrot )
        right_wheel = self.wheel_speed( v + vrot )
        self.wheels(left_wheel, right_wheel)
        sleep(dt)
        if stop_flag:
            self.wheels(0,0)
        return

    def move_background(self):
        """Continuosly adjust orientation
        to self.target_heading while
        cruising at self.target_speed if possible.
        """
        dt = 0.1
        # Factor between omega and dtheta.
        # Too big and the robot oscillates before reaching
        # the target heading.
        gain = 2.5 / pi
        while self.moving_background:
            self.update_state()
            dtheta = self.target_heading - self.get_heading()
            if dtheta > pi:
                dtheta -= 2*pi
            if dtheta < -pi:
                dtheta += 2*pi

            if abs(dtheta) > 0.1:
                self.aligned.clear()
            else:
                self.aligned.set()

            if abs(dtheta) > 0.01:
                omega = gain * dtheta
            else:
                omega = 0.0
            self.move( dt , self.target_speed , omega , stop_flag=False) # this sleeps
            self.awake.wait()
        self.halt()
        return

# -- Sensors

    def light_detected(self):
        """Check if light is detected.
        If so, sound the buzzer.
        """
        light = self.light()[1]>0.9 or self.light()[0]>0.9 # from eBot.eBot
        if light:
            self.buzzer(500,2000) # from eBot.eBot
        return light

    def get_position(self):
        """Return the position in
        global coordinates.
        """
        if not self.moving_background:
            self.update_state()
        return self.pos

    def get_heading(self):
        """Return the heading in
        global coordinates.
        """
        if not self.moving_background:
            self.update_state()
        return self.heading

    def update_state(self):
        """Update the values of self.pos and self.heading
        according to the robot's readings.
        If an error occurs when calling eBot.position(),
        write to stderr and set the self.pos and self.heading
        to None so that any threard accesing them will fail
        and complain loudly (or at least get nonsense).
        """
        try:
            p = self.position()
            self.pos[0] , self.pos[1] = float(p[0]), float(p[1])
            h =  pi * float(p[2]) / 180.
            if h > pi:
                self.heading = h - 2*pi
            elif h < -pi:
                self.heading = h + 2*pi
            else:
                self.heading = h
        except:
            sys.stderr.write("eBotBody.update_state(): problem calling eBot.eBot.position\n")
            self.pos = None
            self.heading = None
        return

    def get_ultrasound(self):
        """Ultrasound readings, ignoring the
        sensor on the back.
        If the sensors dont detect anything,
        return 2.5 or so (eBot inner workings.)
        """
        return self.robot_uS()[0:5]

    def obstacle_coordinates(self):
        """Coordinates of the five obstacle points
        with respect to the robot (ignore the back).
        """
        h = self.get_heading()
        theta = self.aperture
        [dLL, dL, dC, dR, dRR] = self.get_ultrasound()

        xLL = [dLL*cos(h + 2.*theta) , dLL*sin(h + 2.*theta) ]
        xL  = [dL *cos(h + theta)    , dL *sin(h + theta)    ]
        xC  = [dC *cos(h)            , dC *sin(h)            ]
        xR  = [dR *cos(h - theta)    , dR *sin(h - theta)    ]
        xRR = [dRR*cos(h - 2.*theta) , dRR*sin(h - 2.*theta) ]
        return xLL, xL, xC, xR, xRR

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        #TODO Explore the optimal ranges where of each sensor.
        us = self.get_ultrasound()
        return us[2]<0.30 or us[1]<0.30 or us[3]<0.30

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        #TODO Explore the optimal ranges where of each sensor.
        #return any( d<0.4 for d in self.get_ultrasound() if d)
        us = self.get_ultrasound()
        return us[2]<0.40 or us[1]<0.40 or us[3]<0.40 or us[0]<0.40 or us[4]<0.40


    def get_wheel_distance(self):
        """Return the distance between
        the wheels of the eBot.
        """
        return self.LRdist

    def get_sensor_aperture(self):
        """Return the angle between the
        sensors in the eBot.
        """
        return self.aperture

