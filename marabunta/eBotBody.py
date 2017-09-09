from math import atan2, sin, cos, pi
from eBot import eBot
from BaseRobot import BaseBody
import threading
from time import time, sleep
import sys
from utils import clean_angle


class eBotBody(BaseBody, eBot.eBot):
    """Body class for controlling an eBot (see https://github.com/EdgeBotix/docs)
    by using its python API.
    A background movement is implemented where a separate thread continuously
    moves the robot at a linear speed set by *self.target_speed* while aligning
    the heading to *self.target_heading*.
    Alternativaly, if a *self.target_omega* is set, the thread uses a PID
    controller to rotate at frequency omega.
    The background movement is started when the body is turned on, but it can
    be stopped at any point and the class is still completely usable without
    this background thread.

    This class assumes the eBot-API provides a position() method that
    returns (x, y, theta) with x, y in meters and theta in degrees between 0
    and 360, and a robot_uS() method that returns the readings from the
    ultrasound sensors.
    """
    def __init__(self, pos, heading,
                 lock=None, max_speed=0.15, LRdist=0.10, aperture=0.7854):
        # State (in robot coordinates)
        self.pos = [pos[0], pos[1]]
        self.heading = heading
        # Parameters
        self.max_speed = max_speed
        self.LRdist = LRdist
        self.aperture = aperture

        # Controller parameters
        self.control_Kp = 3.
        self.control_Ti = 11.
        self.control_Td = 0.01

        self.awake = threading.Event()
        self.aligned = threading.Event()
        self.awake.set()
        self.moving_background = False
        self.target_heading = None
        self.target_speed = None
        self.target_omega = None
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
            self.thread = threading.Thread(target=self.move_background)
            self.thread.daemon = True
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
            self.awake.set()  # force wakeup to finish the thread
            self.thread.join(10)
            if self.thread.is_alive():
                raise Exception("Unable to stop move_background thread.")
        return

    def turn_on(self):
        """Connect to the eBot using the
        API method and start the background movement.
        """
        self.connect()  # from eBot.eBot
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
        self.halt()  # from eBot.eBot
        self.disconnect()  # from eBot.eBot
        return

    def __enter__(self):
        self.turn_on()
        return self

    def __exit__(self, type, value, traceback):
        self.turn_off()
        return

    def wakeup(self):
        if not self.awake.is_set():
            self.awake.set()
        return

    def sleep(self):
        self.awake.clear()
        self.halt()
        return

# Movement

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

    def move_forward(self, dt, v=None):
        """Move in current direction for *dt* time
        at a speed of *v* if given, or *max_speed*
        if not.
        If the background move is activated,
        *dt* is ignored and this returns inmediately.
        """
        if self.moving_background:
            if v is not None:
                self.target_speed = v
        else:
            if v is None:
                v = self.max_speed
            wheel = self.wheel_speed(v)
            self.wheels(wheel, wheel)
            sleep(dt)
            self.wheels(0, 0)
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
            time = self.LRdist * abs(dtheta) / (2 * self.max_speed)
            if dtheta > 0:
                self.wheels(-1, 1)
            else:
                self.wheels(1, -1)
            sleep(time)
            self.wheels(0, 0)
        return time

    def align(self, direction, block=False):
        """Align the heading of the robot to
        a given vector *direction*.
        Use self_target_heading if the background
        move is active, if not use rotate() instead.
        If block is set to true, wait until the robot
        is aligned before returning.
        """
        h = atan2(direction[1], direction[0])
        if self.moving_background:
            self.target_heading = h
            if block:
                self.aligned.wait()
        else:
            dtheta = clean_angle(h - self.get_heading())
            self.rotate(dtheta)
        return

    def move(self, dt, v, omega, stop_flag=True):
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
        if vrot > self.max_speed:
            vrot = self.max_speed
        if vrot < -self.max_speed:
            vrot = -self.max_speed
        if abs(v) > 0 and abs(v) + abs(vrot) > self.max_speed:
            v = (v / abs(v)) * (self.max_speed - abs(vrot))

        left_wheel = self.wheel_speed(v - vrot)
        right_wheel = self.wheel_speed(v + vrot)
        self.wheels(left_wheel, right_wheel)
        sleep(dt)
        if stop_flag:
            self.wheels(0, 0)
        return

    def move_background(self):
        """Continuosly adjust orientation
        to self.target_heading while
        cruising at self.target_speed if possible.
        Uses PID to align.
        """
        dt = 0.1
        dtheta = 0.0
        time_now = time()
        errorint = 0.
        while self.moving_background:
            self.update_state()
            omega = self.target_omega
            if omega is None:
                # Update old values
                old_dtheta = dtheta
                old_time = time_now
                # Update new values
                dtheta = clean_angle(self.target_heading - self.get_heading())
                time_now = time()

                if abs(dtheta) > 0.1:
                    self.aligned.clear()
                else:
                    self.aligned.set()

                # Proportional
                omega = self.control_Kp * dtheta
                # Integral
                real_dt = time_now - old_time
                if self.control_Ti:
                    errorint += real_dt * dtheta
                    omega += (self.control_Kp * errorint) / self.control_Ti
                # Differential
                if self.control_Td:
                    derrordt = clean_angle(dtheta - old_dtheta) / real_dt
                    omega += self.control_Kp * self.control_Td * derrordt

            self.move(dt, self.target_speed, omega, stop_flag=False)  # sleeps
            self.awake.wait()
        self.halt()
        return

# Sensors

    def light_detected(self):
        """Check if light is detected.
        If so, sound the buzzer.
        """
        lfront, ltop = self.light()  # from eBot.eBot
        light = ltop > 0.99 or lfront > 0.60  # from eBot.eBot
        if light:
            self.buzzer(200, 2000)  # from eBot.eBot
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
            self.pos[0], self.pos[1] = float(p[0]), float(p[1])
            self.heading = clean_angle(pi * float(p[2]) / 180.)
        except:
            sys.stderr.write("problem calling eBot.eBot.position\n")
            self.pos = None
            self.heading = None
        return

    def get_ultrasound(self):
        """Ultrasound readings, ignoring the
        sensor on the back.
        If the sensors dont detect anything,
        return 2.5 or so (eBot inner workings.)
        """
        return self.robot_uS()[0:5]  # from eBot.eBot

    def obstacle_coordinates(self):
        """Coordinates of the five obstacle points
        with respect to the robot (ignore the back).
        """
        h = self.get_heading()
        theta = self.aperture
        [dLL, dL, dC, dR, dRR] = self.get_ultrasound()
        obs = []
        if 0.15 < dLL < 1.0:
            obs.append((dLL * cos(h + 2. * theta), dLL * sin(h + 2. * theta)))
        if 0.15 < dL < 1.0:
            obs.append((dL * cos(h + theta), dL * sin(h + theta)))
        if 0.15 < dC < 1.0:
            obs.append((dC * cos(h), dC * sin(h)))
        if 0.15 < dR < 1.0:
            obs.append((dR * cos(h - theta), dR * sin(h - theta)))
        if 0.15 < dRR < 1.0:
            obs.append((dRR * cos(h - 2. * theta), dRR * sin(h - 2. * theta)))
        return obs

    def obstacle_infront(self):
        """Return True if an obstacle is "in front", meaning
        that extreme measures such as stopping the movement
        have to be taken to avoid a collision.
        Used by move_forward() to stop the robot in case
        something is too close.
        """
        # TODO Explore the optimal ranges where of each sensor.
        us = self.get_ultrasound()
        return us[2] < 0.20 or us[1] < 0.25 or us[3] < 0.20

    def obstacle_near(self):
        """Return True if an obstacle is "near" meaning
        that the robot should be aware of the existence
        of the obstacle even if it may not collide
        directly.
        """
        # TODO Explore the optimal ranges where of each sensor.
        us = self.get_ultrasound()
        # return us[2] < 0.50 or us[1] < 0.45 or \
        #     us[3] < 0.45 or us[0] < 0.40 or us[4] < 0.40
        return us[2] < 0.70 or us[1] < 0.70 or \
            us[3] < 0.70 or us[0] < 0.70 or us[4] < 0.70

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
